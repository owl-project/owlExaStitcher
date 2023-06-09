
#pragma once

#include "Object.h"

namespace exa {

struct Renderer : Object
{
  static Renderer *createInstance(std::string_view subtype, ExaStitchGlobalState *d);

  Renderer(ExaStitchGlobalState *s);
};

} // ::exa

EXA_ANARI_TYPEFOR_SPECIALIZATION(exa::Renderer *, ANARI_RENDERER);

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

