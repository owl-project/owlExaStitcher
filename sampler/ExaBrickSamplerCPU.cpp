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

#include "ExaBrickSamplerCPU.h"

namespace exa {

  bool ExaBrickSamplerCPU::build(ExaBrickModel::SP model)
  {
    using namespace visionaray;

    this->model = model;
    brickBuffer = model->bricks.data();
    scalarBuffer = model->scalars.data();

    std::vector<ABRPrimitive> prims(model->abrs.value.size());

    for (unsigned i=0; i<prims.size(); ++i) {
      prims[i].prim_id = i;
      prims[i].domain = model->abrs.value[i].domain;
      prims[i].valueRange = model->abrs.value[i].valueRange;
      prims[i].leafListBegin = model->abrs.value[i].leafListBegin;
      prims[i].leafListSize = model->abrs.value[i].leafListSize;
      prims[i].finestLevelCellWidth = model->abrs.value[i].finestLevelCellWidth;
    }

    binned_sah_builder builder;
    builder.enable_spatial_splits(false);
    abrBVH = builder.build(index_bvh<ABRPrimitive>{}, prims.data(), prims.size());

    return true;
  }

} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

