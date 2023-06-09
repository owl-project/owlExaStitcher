#include "Renderer.h"
#include <OWLRenderer.h>

namespace exa {

Renderer *Renderer::createInstance(std::string_view subtype, ExaStitchGlobalState *d)
{
  return new Renderer(d);
}

Renderer::Renderer(ExaStitchGlobalState *s)
  : Object(s)
{}

} // ::exa

EXA_ANARI_TYPEFOR_DEFINITION(exa::Renderer *);

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

