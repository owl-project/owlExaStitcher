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

#include <vector>
#include "ABRs.h"
#include "deviceCode.h"
#include "Grid.h"
#include "Model.h"

namespace exa {

  struct ExaBrickModel : Model,
                         std::enable_shared_from_this<ExaBrickModel>
  {
    typedef std::shared_ptr<ExaBrickModel> SP;

    static ExaBrickModel::SP load(const std::string brickFileName,
                                  const std::string scalarFileName,
                                  const std::string kdTreeFileName);

    // How the volume density is sampled ("sampleBVH")
    void setSamplingMode(SamplingMode mode);

    // How space is being skipped ("majorantBVH")
    void setTraversalMode(TraversalMode mode);

    // Set the number of macro cells; the grid is built on initGPU (!)
    void setNumGridCells(const vec3i numMCs);

    std::vector<ExaBrick> bricks;
    std::vector<float>    scalars;
    ABRs                  abrs;
    KDTree::SP            kdtree; // optional kd-tree over bricks
    Grid::SP              grid;   // optional grid for space skipping

    // owl
    OWLGeomType abrGeomType;
    OWLGroup    abrBlas;
    OWLGroup    abrTlas;

    OWLGeomType extGeomType;
    OWLGroup    extBlas;
    OWLGroup    extTlas;

    OWLGeomType brickGeomType;
    OWLGroup    brickBlas;
    OWLGroup    brickTlas;

    OWLBuffer   abrBuffer;
    OWLBuffer   brickBuffer;
    OWLBuffer   scalarBuffer;
    OWLBuffer   abrLeafListBuffer;

    OWLBuffer   abrMaxOpacities;
    OWLBuffer   brickMaxOpacities;

    bool initGPU(OWLContext, OWLModule module);

    // Compute per-ABR max opacities on the GPU
    void computeMaxOpacities(OWLContext owl, OWLBuffer colorMap, range1f xfRange);

    // Statistics
    void memStats(size_t &bricksBytes,
                  size_t &scalarsBytes,
                  size_t &abrsBytes,
                  size_t &abrLeafListBytes);
  
  private:

    SamplingMode samplingMode = EXA_BRICK_SAMPLER_ABR_BVH;
    TraversalMode traversalMode = EXABRICK_ABR_TRAVERSAL;

    // sets model's accels etc. based on traversal and sampling mode
    void initBaseModel();

  };

} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

