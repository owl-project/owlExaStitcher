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

  /* This function makes assumptions that are only useful
    for exajet; we eventually want to move mirroring/other
    scene graph functionalilty out of the model class and
    make the renderer responsible for performing these */
  void Model::setMirrorXZ(bool mirror)
  {
    doMirror = mirror;

    if (doMirror) {
      affine3f transform =
              affine3f::translate(cellBounds.upper)
              * affine3f::scale(vec3f(1,-1,1))
              * affine3f::translate(-cellBounds.upper);

      owl4x3f tfm;
      tfm.t = owl3f{0.f, transform.p.y, 0.f};
      tfm.vx = owl3f{1.f, 0.f, 0.f};
      tfm.vy = owl3f{0.f, transform.l.vy.y, 0.f};
      tfm.vz = owl3f{0.f, 0.f, 1.f};
      mirrorTransform = tfm;
    }
  }

  box3f Model::getBounds() const
  {
    box3f bounds = cellBounds;
    bounds.lower = xfmPoint(rcp(voxelSpaceTransform),bounds.lower);
    bounds.upper = xfmPoint(rcp(voxelSpaceTransform),bounds.upper);

    if (doMirror) {
      const affine3f &tfm = (const affine3f &)mirrorTransform;
      vec3f lower = xfmPoint(tfm,cellBounds.lower);
      vec3f upper = xfmPoint(tfm,cellBounds.upper);
      box3f mirrorBounds{
        min(lower,upper),
        max(lower,upper)
      };
      mirrorBounds.lower = xfmPoint(rcp(voxelSpaceTransform),mirrorBounds.lower);
      mirrorBounds.upper = xfmPoint(rcp(voxelSpaceTransform),mirrorBounds.upper);
      bounds.extend(mirrorBounds);
    }

    return bounds;
  }
} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

