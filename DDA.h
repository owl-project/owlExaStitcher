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

#include <owl/owl.h>

namespace exa {

  typedef vec3i GridIterationState;
  
  template <typename Func, int RT=0, int NRT=1>
  inline __device__
  void dda3(const owl::RayT<RT,NRT> &ray,
            const owl::vec3i        &gridDims,
            const owl::box3f        &modelBounds,
            const Func              &func)
  {
    using namespace owl;

    const vec3f rcp_dir = rcp(ray.direction);

    const vec3f lo = (modelBounds.lower - ray.origin) * rcp_dir;
    const vec3f hi = (modelBounds.upper - ray.origin) * rcp_dir;

    const vec3f tnear = min(lo,hi);
    const vec3f tfar  = max(lo,hi);

    vec3i cellID = projectOnGrid(ray.origin,gridDims,modelBounds);

    // Distance in world space to get from cell to cell
    const vec3f dist((tfar-tnear)/vec3f(gridDims));

    // Cell increment
    const vec3i step = {
      ray.direction.x > 0.f ? 1 : -1,
      ray.direction.y > 0.f ? 1 : -1,
      ray.direction.z > 0.f ? 1 : -1
    };

    // Stop when we reach grid borders
    const vec3i stop = {
      ray.direction.x > 0.f ? gridDims.x : -1,
      ray.direction.y > 0.f ? gridDims.y : -1,
      ray.direction.z > 0.f ? gridDims.z : -1
    };

    // Increment in world space
    vec3f tnext = {
      ray.direction.x > 0.f ? tnear.x + float(cellID.x+1) * dist.x
                            : tnear.x + float(gridDims.x-cellID.x) * dist.x,
      ray.direction.y > 0.f ? tnear.y + float(cellID.y+1) * dist.y
                            : tnear.y + float(gridDims.y-cellID.y) * dist.y,
      ray.direction.z > 0.f ? tnear.z + float(cellID.z+1) * dist.z
                            : tnear.z + float(gridDims.z-cellID.z) * dist.z
    };


    float t0 = max(ray.tmin,0.f);

    while (1) { // loop over grid cells
      const float t1 = min(reduce_min(tnext),ray.tmax);
      if (!func(GridIterationState(cellID),t0,t1))
        return;

#if 0
      int axis = arg_min(tnext);
      tnext[axis] += dist[axis];
      cellID[axis] += step[axis];
      if (cellID[axis]==stop[axis]) {
        break;
      }
#else
      const float t_closest = reduce_min(tnext);
      if (tnext.x == t_closest) {
        tnext.x += dist.x;
        cellID.x += step.x;
        if (cellID.x==stop.x) {
          break;
        }
      }
      if (tnext.y == t_closest) {
        tnext.y += dist.y;
        cellID.y += step.y;
        if (cellID.y==stop.y) {
          break;
        }
      }
      if (tnext.z == t_closest) {
        tnext.z += dist.z;
        cellID.z += step.z;
        if (cellID.z==stop.z) {
          break;
        }
      }
#endif
      t0 = t1;
    }
  }
} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

