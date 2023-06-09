#include "Light.h"
using namespace anari;

namespace exa {

Light *Light::createInstance(std::string_view subtype, ExaStitchGlobalState *d)
{
  if (subtype=="point")
    return new Light(d);
  else {
    std::cerr << "ANARILight subtype: " << subtype << " not supported\n";
    return (Light *)new Object(d);
  }
}

Light::Light(ExaStitchGlobalState *s)
  : Object(s)
{}

void Light::commit()
{
  color = getParam<float3>("color", float3(1.f));
  pos = getParam<float3>("position", float3(0.f));
  intensity = getParam<float>("intensity", 1.f);

  //std::cout << "Light::commit() " << color << ',' << pos << ',' << intensity << '\n';

  markUpdated();
}

} // ::exa

EXA_ANARI_TYPEFOR_DEFINITION(exa::Light *);

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

