
#pragma once

#include <anari/anari_cpp/ext/linalg.h>
#include "Object.h"

namespace exa {

struct Light : Object
{
  static Light *createInstance(std::string_view subtype, ExaStitchGlobalState *d);

  Light(ExaStitchGlobalState *s);
  virtual void commit() override;
private:
  anari::float3 color;
  // lights's are always point lights
  anari::float3 pos;
  float intensity;
};

} // ::exa

EXA_ANARI_TYPEFOR_SPECIALIZATION(exa::Light *, ANARI_LIGHT);

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

