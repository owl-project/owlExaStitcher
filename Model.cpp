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

#include "Model.h"

namespace exa {

  Model::~Model()
  {
  }

  bool Model::initGPU(OWLContext owl, OWLModule module)
  {
    return false;
  }

  void Model::setVoxelSpaceTransform(const box3f remap_from, const box3f remap_to)
  {
    affine3f voxelSpaceCoordSys
      = affine3f::translate(remap_from.lower)
      * affine3f::scale(remap_from.span());

    affine3f worldSpaceCoordSys
      = affine3f::translate(remap_to.lower)
      * affine3f::scale(remap_to.span());

    voxelSpaceTransform
      = voxelSpaceCoordSys
      * rcp(worldSpaceCoordSys);
  }

  box3f Model::getBounds() const
  {
    box3f bounds = cellBounds;
    bounds.lower = xfmPoint(rcp(voxelSpaceTransform),bounds.lower);
    bounds.upper = xfmPoint(rcp(voxelSpaceTransform),bounds.upper);
    return bounds;
  }
} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

