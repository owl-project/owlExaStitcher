#include "Object.h"

namespace exa {

Object *Object::createInstance(std::string_view subtype, ExaStitchGlobalState *d)
{
  std::cerr << "Unknown object/type not supported: " << subtype << '\n';
  return nullptr;
}

Object::Object(ExaStitchGlobalState *s)
  : helium::BaseObject(ANARI_UNKNOWN, s)
{}

Object::Object(ANARIDataType type, ExaStitchGlobalState *s)
  : helium::BaseObject(type, s)
{}

bool Object::getProperty(const std::string_view &name,
  ANARIDataType type,
  void *ptr,
  uint32_t flags)
{ return false; }

void Object::commit()
{}

bool Object::isValid() const
{
  return false;
}

ExaStitchGlobalState *Object::deviceState() const
{
  return (ExaStitchGlobalState *)helium::BaseObject::m_state;
}

} // ::exa 

EXA_ANARI_TYPEFOR_DEFINITION(exa::Object *);

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

