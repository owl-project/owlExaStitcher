
#pragma once

#include "array/Array1D.h"
#include "math.h"
#include "Object.h"
#include "SpatialField.h"

namespace exa {

struct Volume : Object
{
  static Volume *createInstance(std::string_view subtype, ExaStitchGlobalState *d);

  Volume(ExaStitchGlobalState *s);
  virtual void commit() override;
private:

  box1f valueRange;
  helium::IntrusivePtr<SpatialField> field;

  helium::IntrusivePtr<Array1D> color;
  helium::IntrusivePtr<Array1D> opacity;
  float densityScale;
};

} // ::exa

EXA_ANARI_TYPEFOR_SPECIALIZATION(exa::Volume *, ANARI_VOLUME);

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

