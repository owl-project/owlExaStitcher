#include "Frame.h"
#include "Renderer.h"
#include <OWLRenderer.h>
using namespace anari;

namespace exa {

template <typename R, typename TASK_T>
static std::future<R> async(TASK_T &&fcn)
{
  auto task = std::packaged_task<R()>(std::forward<TASK_T>(fcn));
  auto future = task.get_future();

  std::thread([task = std::move(task)]() mutable { task(); }).detach();

  return future;
}

template <typename R>
static bool is_ready(const std::future<R> &f)
{
  return !f.valid()
      || f.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
}

Frame::Frame(ExaStitchGlobalState *s)
  : helium::BaseFrame(s)
{}

Frame::~Frame()
{
  cudaFree(d_pixelBuffer);
}

bool Frame::getProperty(
    const std::string_view &name, ANARIDataType type, void *ptr, uint32_t flags)
{
  if (type == ANARI_FLOAT32 && name == "duration") {
    if (flags & ANARI_WAIT)
      wait();
    helium::writeToVoidP(ptr, m_duration);
    return true;
  }

  return 0;
}

void Frame::commit()
{
  m_renderer = getParamObject<Renderer>("renderer");
  if (!m_renderer) {
    std::cerr << "Frame::commit(): param \"renderer\" must bet set\n";
    return;
  }

  m_camera = getParamObject<Camera>("camera");
  if (!m_camera) {
    std::cerr << "Frame::commit(): param \"camera\" must bet set\n";
    return;
  }

  m_colorType = getParam<anari::DataType>("channel.color", ANARI_UNKNOWN);
  m_depthType = getParam<anari::DataType>("channel.depth", ANARI_UNKNOWN);

  m_frameData.size = getParam<anari::uint2>("size", anari::uint2(10));
  m_frameData.invSize = 1.f / anari::float2(m_frameData.size);

  const auto numPixels = m_frameData.size.x * m_frameData.size.y;

  m_perPixelBytes = 4 * (m_colorType == ANARI_FLOAT32_VEC4 ? 4 : 1);
  m_pixelBuffer.resize(numPixels * m_perPixelBytes);

  m_depthBuffer.resize(m_depthType == ANARI_FLOAT32 ? numPixels : 0);

  cudaFree(d_pixelBuffer);
  cudaMalloc(&d_pixelBuffer,m_pixelBuffer.size());

  deviceState()->owlRenderer->resize({(int)m_frameData.size.x,(int)m_frameData.size.y});
  m_camera->apply();
  deviceState()->owlRenderer->resetAccum();

  m_frameChanged = true;
}

void Frame::renderFrame()
{
  auto *state = deviceState();
  if (state->currentFrame)
    state->currentFrame->wait();

  auto start = std::chrono::steady_clock::now();

  state->commitBuffer.flush();

  state->currentFrame = this;

  //m_future = async<void>([&, state, start, c, w, r]() {
  m_future = async<void>([this,&start]() {
    //std::cout << "Frame::renderFrame()\n";
    if (d_pixelBuffer) {
      deviceState()->owlRenderer->render((uint32_t *)d_pixelBuffer);
    }
    auto end = std::chrono::steady_clock::now();
    m_duration = std::chrono::duration<float>(end - start).count();
  });
}

void *Frame::map(std::string_view channel,
    uint32_t *width,
    uint32_t *height,
    ANARIDataType *pixelType)
{
  wait();

  *width = m_frameData.size.x;
  *height = m_frameData.size.y;

  if (channel == "color" || channel == "channel.color") {
    *pixelType = m_colorType;
    return mapColorBuffer();
  } else if (channel == "depth" || channel == "channel.depth") {
    *pixelType = ANARI_FLOAT32;
    return mapDepthBuffer();
  } else {
    *width = 0;
    *height = 0;
    *pixelType = ANARI_UNKNOWN;
    return nullptr;
  }
}

void Frame::unmap(std::string_view channel)
{
  // no-op
}

int Frame::frameReady(ANARIWaitMask m)
{
  if (m == ANARI_NO_WAIT)
    return ready();
  else {
    wait();
    return 1;
  }
}

void Frame::discard()
{
  // no-op
}

void *Frame::mapColorBuffer()
{
  cudaMemcpy(m_pixelBuffer.data(),d_pixelBuffer,
             m_pixelBuffer.size()*sizeof(m_pixelBuffer[0]),
             cudaMemcpyDeviceToHost);
  return m_pixelBuffer.data();
}

void *Frame::mapDepthBuffer()
{
  // TODO: d2h copy?!
  // (do we want to support this?)
  return m_depthBuffer.data();
}

bool Frame::ready() const
{
  return is_ready(m_future);
}

void Frame::wait() const
{
  if (m_future.valid())
    m_future.get();
}

ExaStitchGlobalState *Frame::deviceState() const
{
  return (ExaStitchGlobalState *)helium::BaseFrame::m_state;
}

} // ::exa

EXA_ANARI_TYPEFOR_DEFINITION(exa::Frame *);

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

