
#pragma once

#include <anari/anari_cpp/ext/linalg.h>
#include "Object.h"

namespace exa {

struct Camera : Object
{
  static Camera *createInstance(std::string_view subtype, ExaStitchGlobalState *d);

  Camera(ExaStitchGlobalState *s);
  virtual void commit() override;
  void apply();
private:
  anari::float3 pos, dir, up;
  // camera's are always perspective
  anari::float3 dir_du, dir_dv, dir_00;
};

} // ::exa

EXA_ANARI_TYPEFOR_SPECIALIZATION(exa::Camera *, ANARI_CAMERA);

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

