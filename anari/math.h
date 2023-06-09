#pragma once

#include <ostream>
#include <anari/anari_cpp/ext/linalg.h>

namespace exa {

struct box1f {
  float lower, upper;
};

struct box3i {
  anari::int3 lower, upper;
};

} // ::exa

namespace anari {

inline std::ostream &operator<<(std::ostream &out, float3 v)
{
  out << '(' << v.x << ',' << v.y << ',' << v.z << ')';
  return out;
}

ANARI_TYPEFOR_SPECIALIZATION(exa::box1f, ANARI_FLOAT32_BOX1);
ANARI_TYPEFOR_SPECIALIZATION(exa::box3i, ANARI_INT32_BOX3);

#ifdef EXA_ANARI_DEFINITIONS
ANARI_TYPEFOR_DEFINITION(exa::box1f);
ANARI_TYPEFOR_DEFINITION(exa::box3i);
#endif

} // ::anari

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

