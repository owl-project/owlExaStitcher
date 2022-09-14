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
#include "deviceCode.h"
#include "Model.h"

namespace exa {

  struct ExaStitchModel : Model,
                          std::enable_shared_from_this<ExaStitchModel>
  {
    typedef std::shared_ptr<ExaStitchModel> SP;

    static ExaStitchModel::SP load(const std::string umeshFileName,
                                   const std::string gridsFileName,
                                   const std::string scalarFileName);

    std::vector<int>     indices;
    std::vector<vec4f>   vertices;
    std::vector<Gridlet> gridlets;
    // The scalars referenced by gridlet; umesh scalars
    // are stored in the respectivep vertices w coordinate
    std::vector<float>   gridletScalars;

    // owl
    struct {
      OWLGeomType geomType;
      OWLGroup blas;
    } gridletGeom;

    struct {
      OWLGeomType geomType;
      OWLGroup blas;
    } stitchGeom;

    OWLGroup tlas;

    OWLBuffer indexBuffer;
    OWLBuffer vertexBuffer;
    OWLBuffer gridletBuffer;
    OWLBuffer gridletScalarBuffer;

    bool initGPU(OWLContext context, OWLModule module);

    // Statistics
    size_t numScalarsTotal;
    size_t numEmptyScalars;
    void memStats(size_t &elemVertexBytes,
                  size_t &elemIndexBytes,
                  size_t &gridletBytes,
                  size_t &emptyScalarsBytes,
                  size_t &nonEmptyScalarsBytes);
  };

} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

