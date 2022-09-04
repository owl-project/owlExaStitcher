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
#include "KDTree.h"
#include "Model.h"

#include "KDTree.h"

namespace exa {

  struct ExaBrickModel : Model {
    typedef std::shared_ptr<ExaBrickModel> SP;

    static ExaBrickModel::SP load(const std::string brickFileName,
                                  const std::string scalarFileName,
                                  const std::string kdTreeFileName);

    std::vector<ExaBrick> bricks;
    std::vector<float>    scalars;
    ABRs                  abrs;
    KDTree::SP            kdtree; // optional kd-tree over bricks

    // owl
    OWLGeomType abrGeomType;
    OWLGroup    abrBlas;
    OWLGroup    abrTlas;

    OWLGeomType extGeomType;
    OWLGroup    extBlas;
    OWLGroup    extTlas;

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
  };

} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

