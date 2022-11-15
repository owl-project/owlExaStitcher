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

#include <memory>
#include <owl/owl.h>
#include <owl/common/math/box.h>
#include <owl/common/math/vec.h>
#include "common.h"
#include "Grid.cuh"

namespace exa {

  class AMRCellSampler;
  class ExaBrickSampler;
  class ExaStitchSampler;
  class QuickClustersSampler;

  struct Grid
  {
    typedef std::shared_ptr<Grid> SP;

    // Build from simple cell sampler
    void build(OWLContext owl,
               std::shared_ptr<AMRCellSampler> sampler,
               const owl::vec3i numMCs,
               const owl::box3f bounds);

    // Build from exa brick sampler
    void build(OWLContext owl,
               std::shared_ptr<ExaBrickSampler> sampler,
               const owl::vec3i numMCs,
               const owl::box3f bounds);

    // Build from exa stitch model
    void build(OWLContext owl,
               std::shared_ptr<ExaStitchSampler> sampler,
               const owl::vec3i  numMCs,
               const owl::box3f  bounds);

    // Build from quick clusters model
    void build(OWLContext           owl,
                std::shared_ptr<QuickClustersSampler> sampler,
                   const owl::vec3i     numMCs,
                   const owl::box3f     bounds);

    // Build a BVH, in case we decide to traverse it with OptiX (bnechmark!)
    bool buildOptixBVH(OWLContext owl, OWLModule module);

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

