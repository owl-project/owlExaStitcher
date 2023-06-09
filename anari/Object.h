
#pragma once

#include <iostream>
#include <anari/anari_cpp/ext/linalg.h>
#include "helium/BaseObject.h"
#include "ExaStitchGlobalState.h"

namespace exa {

struct Object : public helium::BaseObject
{
  static Object *createInstance(std::string_view subtype, ExaStitchGlobalState *d);

  Object(ExaStitchGlobalState *s);
  Object(ANARIDataType type, ExaStitchGlobalState *s);
  virtual ~Object() = default;

  // BaseObject interface
  virtual bool getProperty(const std::string_view &name,
      ANARIDataType type,
      void *ptr,
      uint32_t flags);
  virtual void commit();
  virtual bool isValid() const;
  ExaStitchGlobalState *deviceState() const;
};

typedef Object Surface;
//typedef Object Light;
//typedef Object Camera;
typedef Object Geometry;
//typedef Object SpatialField;
//typedef Object Volume;
typedef Object Material;
//typedef Object Sampler;
typedef Object Group;
typedef Object Instance;
typedef Object World;
//typedef Object Frame;

} // ::exa

#define EXA_ANARI_TYPEFOR_SPECIALIZATION(type, anari_type)                     \
  namespace anari {                                                            \
  ANARI_TYPEFOR_SPECIALIZATION(type, anari_type);                              \
  }

#define EXA_ANARI_TYPEFOR_DEFINITION(type)                                     \
  namespace anari {                                                            \
  ANARI_TYPEFOR_DEFINITION(type);                                              \
  }

EXA_ANARI_TYPEFOR_SPECIALIZATION(exa::Object *, ANARI_OBJECT);

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

