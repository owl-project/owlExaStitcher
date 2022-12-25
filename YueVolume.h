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

#include <vector>
#include "sampler/ExaBrickSamplerCPU.h"

namespace exa {

  // Wrapper for ExaBricks volume to compute Yue majorant kd-trees
  struct YueVolume
  {
    box3i cellBounds;
    range1f valueRange;
    ExaBrickSamplerCPU::SP sampler = nullptr;

    box3i getBounds()
    {
      return cellBounds;
    }

    void iterationRange(int axis, int s, int t, int &begin, int &end, int &step)
    {
      begin = clamp(s,cellBounds.lower[axis],cellBounds.upper[axis]);
      end   = clamp(t,cellBounds.lower[axis],cellBounds.upper[axis]);
      step  = 1;
    }

    void min_max(box3i V, int axis, int plane, float &minValue, float &maxValue,
                 const std::vector<float> *rgbaCM)
    {
      minValue =  1e31f;
      maxValue = -1e31f;
      int cmSize = rgbaCM ? rgbaCM->size()/4 : 0;

      vec3i tc;
      tc[axis]=plane;
      int a1 = (axis+1)%3;
      int a2 = (axis+2)%3;
      for (tc[a1]=V.lower[a1]; tc[a1]<V.upper[a1]; ++tc[a1]) {
        for (tc[a2]=V.lower[a2]; tc[a2]<V.upper[a2]; ++tc[a2]) {
          float val = value(tc.x,tc.y,tc.z);
          val -= valueRange.lower;
          val /= valueRange.upper-valueRange.lower;
          if (rgbaCM) {
            int rgbaID = val*(cmSize-1);
            float a = (*rgbaCM)[rgbaID];
            minValue = fminf(minValue,a);
            maxValue = fmaxf(maxValue,a);
          } else {
            minValue = fminf(minValue,val);
            maxValue = fmaxf(maxValue,val);
          }
        }
      }
      // std::cout << V << ',' << axis << ',' << plane << ": " << minValue << ',' << maxValue << '\n';
    }

    float value(float x, float y, float z)
    {
      Sample s = sample(*sampler,{},{x,y,z});
      return s.value;
    }
  };
} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0
