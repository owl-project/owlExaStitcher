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

  struct AMRCellModel : Model,
                        std::enable_shared_from_this<AMRCellModel>
  {
    typedef std::shared_ptr<AMRCellModel> SP;

    static AMRCellModel::SP load(const std::string cellFileName,
                                 const std::string scalarFileName);

    std::vector<AMRCell> cells;
    std::vector<float>   scalars;

    // owl
    OWLGeomType geomType;
    OWLGroup    blas;
    OWLGroup    tlas;
    OWLBuffer   cellBuffer;
    OWLBuffer   scalarBuffer;

    bool initGPU(OWLContext context, OWLModule module);

    // Statistics
    void memStats(size_t &cellsBytes, size_t &scalarsBytes);
  };

} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

