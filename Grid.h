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
#include "AMRCellModel.h"
#include "ExaBrickModel.h"
#include "ExaStitchModel.h"

namespace exa {

  struct Grid
  {
    // Build from simple cell model
    void build(OWLContext       owl,
               AMRCellModel::SP model,
               const owl::vec3i numMCs,
               const owl::box3f bounds);

    // Build from exa brick model
    void build(OWLContext        owl,
               ExaBrickModel::SP model,
               const owl::vec3i  numMCs,
               const owl::box3f  bounds);

    // Build from exa stitch model
    void build(OWLContext         owl,
               ExaStitchModel::SP model,
               const owl::vec3i   numMCs,
               const owl::box3f   bounds);

    void build(OWLContext       owl, OWLModule module,
               OWLBuffer        vertices,       /* umesh verts; w is value */
               OWLBuffer        indices,        /* umesh indices */
               OWLBuffer        gridlets,       /* gridlet buffer */
               OWLBuffer        gridletScalars, /* scalars referenced by gridlets */
               OWLBuffer        amrCells,       /* AMR cells */
               OWLBuffer        amrScalars,     /* scalars used with AMR cells */
               OWLBuffer        abrs,           /* ExaBrick ABRs */
               const owl::vec3i numMCs,
               const owl::box3f bounds);

    // Build a BVH, in case we decide to traverse it with OptiX (bnechmark!)
    bool initGPU(OWLContext owl, OWLModule module);

    //
    void computeMaxOpacities(OWLContext owl, OWLBuffer colorMap, range1f xfRange);

    GridTraversableHandle deviceTraversable;

    // min/max value ranges
    OWLBuffer  valueRanges;

    // Majorants
    OWLBuffer  maxOpacities { 0 };

    // Number of MCs
    owl::vec3i dims;

    // World bounds the grid spans
    owl::box3f worldBounds;

    // MC BVH
    OWLGeomType geomType;
    OWLGroup    blas;
    OWLGroup    tlas;

  };

} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

