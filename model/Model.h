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
#include "common.h"
#include "Grid.h"
#include "KDTree.h"

namespace exa {

  struct Model : std::enable_shared_from_this<Model>
  {
    typedef std::shared_ptr<Model> SP;

    virtual ~Model();

    template <typename T>
    inline std::shared_ptr<T> as()
    {
      if (!this) return {};
      return std::dynamic_pointer_cast<T>(shared_from_this());
    }

    // Set the number of macro cells; the grid is built on Sampler::build() (!)
    void setNumGridCells(const vec3i numMCs);

    void setVoxelSpaceTransform(const box3f remap_from, const box3f remap_to);

    /*! optional grid for space skipping */
    Grid::SP grid = nullptr;

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
#ifdef EXA_STITCH_MIRROR_EXAJET
    owl4x3f  mirrorTransform;
    void initMirrorExajet();
#endif
  };

} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

