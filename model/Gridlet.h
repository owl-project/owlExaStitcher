// ======================================================================== //
// Copyright 2019 Ingo Wald                                                 //
//                                                                          //
// Licensed under the Apache License, Version 2.0 (the "License");          //
// you may not use this file except in compliance with the License.         //
// You may obtain a copy of the License at                                  //
//                                                                          //
//     http://www.apache.org/licenses/LICENSE-2.0                           //
//                                                                          //
// Unless required by applicable law or agreed to in writing, software      //
// distributed under the License is distributed on an "AS IS" BASIS,        //
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. //
// See the License for the specific language governing permissions and      //
// limitations under the License.                                           //
// ======================================================================== //

#pragma once

#include <common.h>

namespace exa {
  struct Gridlet {
    vec3i  lower;
    int    level;
    vec3i  dims;
    /*! offset into the scalar data index buffer (in which all gridlets
        are stored sequentially) */
    uint32_t begin;

    inline __both__
    box3f getBounds() const
    {
      const float cellWidth = 1<<level;
      return box3f((vec3f(lower) + 0.5f) * cellWidth,
                   (vec3f(lower) + vec3f(dims) + 0.5f) * cellWidth);
    }
  };
} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

