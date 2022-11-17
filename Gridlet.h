// ======================================================================== //
// Copyright 2019 Ingo Wald                                                 //
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

#include <common.h>

namespace exa {
  struct Gridlet {
    vec3i  lower;
    int    level;
    vec3i  dims;
    /*! offset into the scalar data index buffer (in which all gridlets
        are stored sequentially) */
    uint32_t begin;

    inline __both__
    box3f getBounds() const
    {
      const float cellWidth = (float)(1<<level);
      return box3f((vec3f(lower) + 0.5f) * cellWidth,
                   (vec3f(lower) + vec3f(dims) + 0.5f) * cellWidth);
    }
  };

  inline __both__
  bool intersectGridlet(float &value,
                        int &cellID,
                        const vec3f pos,
                        const Gridlet &gridlet,
                        const float *scalars)
  {
    const box3f &bounds = gridlet.getBounds();

    if (bounds.contains(pos)) {
      vec3i numScalars = gridlet.dims+1;
      vec3f posInLevelCoords = (pos-bounds.lower) / (1<<gridlet.level);
      vec3i imin(posInLevelCoords);
      vec3i imax = min(imin+1,numScalars-1);

      auto linearIndex = [numScalars](const int x, const int y, const int z) {
        return z*numScalars.y*numScalars.x + y*numScalars.x + x;
      };

      float f1 = scalars[gridlet.begin + linearIndex(imin.x,imin.y,imin.z)];
      float f2 = scalars[gridlet.begin + linearIndex(imax.x,imin.y,imin.z)];
      float f3 = scalars[gridlet.begin + linearIndex(imin.x,imax.y,imin.z)];
      float f4 = scalars[gridlet.begin + linearIndex(imax.x,imax.y,imin.z)];

      float f5 = scalars[gridlet.begin + linearIndex(imin.x,imin.y,imax.z)];
      float f6 = scalars[gridlet.begin + linearIndex(imax.x,imin.y,imax.z)];
      float f7 = scalars[gridlet.begin + linearIndex(imin.x,imax.y,imax.z)];
      float f8 = scalars[gridlet.begin + linearIndex(imax.x,imax.y,imax.z)];

      if (!isnan(f1) && !isnan(f2) && !isnan(f3) && !isnan(f4) &&
          !isnan(f5) && !isnan(f6) && !isnan(f7) && !isnan(f8)) {

        vec3f frac = posInLevelCoords-vec3f(vec3i(posInLevelCoords));

        // if (debug()) printf("%f,%f,%f -- %f,%f,%f -- %f,%f,%f\n",
        //                     pos.x,pos.y,pos.z,
        //                     posInLevelCoords.x,
        //                     posInLevelCoords.y,
        //                     posInLevelCoords.z,
        //                     frac.x,frac.y,frac.z);

        float f12 = lerp(f1,f2,frac.x);
        float f56 = lerp(f5,f6,frac.x);
        float f34 = lerp(f3,f4,frac.x);
        float f78 = lerp(f7,f8,frac.x);

        float f1234 = lerp(f12,f34,frac.y);
        float f5678 = lerp(f56,f78,frac.y);

        value = lerp(f1234,f5678,frac.z);

        cellID = imin.z*gridlet.dims.y*gridlet.dims.x
                             + imin.y*gridlet.dims.x
                             + imin.x;
        return true;
      }
    }

    return false;
  }
} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

