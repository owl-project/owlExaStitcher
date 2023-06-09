
#pragma once

#include "array/Array1D.h"
#include "array/ObjectArray.h"
#include "math.h"
#include "Object.h"

namespace exa {

struct SpatialField : Object
{
  static SpatialField *createInstance(std::string_view subtype, ExaStitchGlobalState *d);
  SpatialField(ExaStitchGlobalState *s);
  virtual void commit() override;
private:

  // only supported field type is "amr"
  helium::IntrusivePtr<Array1D> blockBounds;
  helium::IntrusivePtr<Array1D> blockLevel;
  helium::IntrusivePtr<ObjectArray> blockData;
};

} // ::exa

EXA_ANARI_TYPEFOR_SPECIALIZATION(exa::SpatialField *, ANARI_SPATIAL_FIELD);

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

