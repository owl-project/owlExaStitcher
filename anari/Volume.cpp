#include "Volume.h"
#include <OWLRenderer.h>
using namespace anari;

namespace exa {

Volume *Volume::createInstance(std::string_view subtype, ExaStitchGlobalState *d)
{
  if (subtype=="scivis")
    return new Volume(d);
  else {
    std::cerr << "ANARIVolume subtype: " << subtype << " not supported\n";
    return (Volume *)new Object(d);
  }
}

Volume::Volume(ExaStitchGlobalState *s)
  : Object(s)
{}

void Volume::commit()
{
  field = getParamObject<SpatialField>("field");
  if (!field) {
    std::cerr << "Volume::commit(): no field provided\n";
    return;
  }

  valueRange = getParam<box1f>("valueRange", box1f{0.f,1.f});

  color = getParamObject<Array1D>("color");
  if (!color) {
    std::cerr << "Volume::commit(): no color provided\n";
    return;
  }

  opacity = getParamObject<Array1D>("opacity");
  if (!opacity) {
    std::cerr << "Volume::commit(): no opacity provided\n";
    return;
  }

  densityScale = getParam<float>("densityScale", 100.f); // 100 as reasonable default for apth tracer..

  if (!deviceState()->owlRenderer) {
    // this should never happen though
    std::cerr <<"Volume::commit(): internal error\n";
    return;
  }

  std::vector<owl::vec4f> xf(opacity->totalSize());
  for (size_t i=0; i<xf.size(); ++i) {
    anari::float3 rgb = *(color->beginAs<anari::float3>()+i);
    float a = *(opacity->beginAs<float>()+i);
    xf[i] = owl::vec4f(rgb.x,rgb.y,rgb.z,a);
  }

  deviceState()->owlRenderer->setColorMap(xf);
  deviceState()->owlRenderer->setRange({valueRange.lower,valueRange.upper});
  deviceState()->owlRenderer->setOpacityScale(densityScale);

  markUpdated();
}

} // ::exa

EXA_ANARI_TYPEFOR_DEFINITION(exa::Volume *);

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

