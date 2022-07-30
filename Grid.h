// ======================================================================== //
// Copyright 2022-2022 Stefan Zellmann                                      //
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

#include <owl/owl.h>
#include <owl/common/math/box.h>
#include <owl/common/math/vec.h>

namespace exa {

  struct Grid
  {
    //
    void build(OWLContext       owl,
               OWLBuffer        vertices,   /* umesh verts; w is value */
               OWLBuffer        indices,    /* umesh indices */
               OWLBuffer        gridlets,   /* gridlet buffer */
               OWLBuffer        amrCells,   /* AMR cells */
               OWLBuffer        amrScalars, /* scalars used with AMR cells */
               const owl::vec3i numMCs,
               const owl::box3f bounds);

    //
    void computeMaxOpacities(OWLContext owl, OWLBuffer colorMap, range1f xfRange);

    // min/max value ranges
    OWLBuffer  valueRanges;

    // Majorants
    OWLBuffer  maxOpacities { 0 };

    // Number of MCs
    owl::vec3i dims;

    // World bounds the grid spans
    owl::box3f worldBounds;
  };

} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

