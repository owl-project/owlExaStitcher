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
#include <owl/common/math/AffineSpace.h>
#include <owl/common/math/box.h>
#include <owl/owl.h>
#include "deviceCode.h"

namespace exa {

  struct Model
  {
    typedef std::shared_ptr<Model> SP;

    virtual ~Model();

    virtual bool initGPU(OWLContext owl, OWLModule module);

    void setVoxelSpaceTransform(const box3f remap_from, const box3f remap_to);

    /*! return proper WORLD SPACE bounds, AFTER transformign voxel
      bounds back from voxel space to world space */
    box3f getBounds() const;

    box3f    cellBounds;
    range1f  valueRange;
    affine3f voxelSpaceTransform;
    /*! usually  a transform that scales from voxel space
      to something smaller. E.g., a transform that scales from
      voxel space to a  [0,1] coordinate system */
    affine3f lightSpaceTransform;
    vec3f    mirrorAxis;
  };

} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

