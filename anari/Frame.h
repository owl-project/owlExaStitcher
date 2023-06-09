
#pragma once

#include <future>
#include <anari/anari_cpp/ext/linalg.h>
#include "helium/BaseFrame.h"
#include "Camera.h"
#include "ExaStitchGlobalState.h"
#include "Object.h"
#include "Renderer.h"

namespace exa {

struct Frame : helium::BaseFrame
{
  Frame(ExaStitchGlobalState *s);
 ~Frame();

  bool getProperty(const std::string_view &name,
      ANARIDataType type,
      void *ptr,
      uint32_t flags) override;

  void commit() override;

  void renderFrame() override;

  void *map(std::string_view channel,
      uint32_t *width,
      uint32_t *height,
      ANARIDataType *pixelType) override;
  void unmap(std::string_view channel) override;
  int frameReady(ANARIWaitMask m) override;
  void discard() override;

  void *mapColorBuffer();
  void *mapDepthBuffer();

  bool ready() const;
  void wait() const;

  ExaStitchGlobalState *deviceState() const;

private:
  bool m_valid{false};
  int m_perPixelBytes{1};

  struct FrameData
  {
    int frameID{0};
    anari::uint2 size;
    anari::float2 invSize;
  } m_frameData;

  anari::DataType m_colorType{ANARI_UNKNOWN};
  anari::DataType m_depthType{ANARI_UNKNOWN};

  std::vector<uint8_t> m_pixelBuffer;
  std::vector<float> m_depthBuffer;

  uint8_t *d_pixelBuffer{nullptr};

  helium::IntrusivePtr<Renderer> m_renderer;
  helium::IntrusivePtr<Camera> m_camera;
  helium::IntrusivePtr<World> m_world;

  float m_duration{0.f};

  bool m_frameChanged{false};

  mutable std::future<void> m_future;
};

} // ::exa

EXA_ANARI_TYPEFOR_SPECIALIZATION(exa::Frame *, ANARI_FRAME);

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

