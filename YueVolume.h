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

    void iterationRange(box3i V, int axis, int &begin, int &end, int &step)
    {
      begin = clamp(V.lower[axis],cellBounds.lower[axis],cellBounds.upper[axis]);
      end   = clamp(V.upper[axis],cellBounds.lower[axis],cellBounds.upper[axis]);

      int minLevel, maxLevel;
      boundsFindMinMaxLevel(V,minLevel,maxLevel);
      step = 1<<minLevel;

      if ((end-begin)/step > 128) {
        auto div_up = [](int a, int b) { return (a+b-1)/b; };
        // 16 bins
        step = div_up(end-begin,16);
      }

      std::cout << "Iteration range: " << V << ',' << axis << ','
                << '[' << begin << ',' << end << ':' << step << ']'
                << '\n';
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

    void boundsFindMinMaxLevel(box3i V, int &minLevel, int &maxLevel)
    {
      minLevel = 1000000000;
      maxLevel = 0;

      unsigned traversalStack[128];
      unsigned stackPtr = 0;
      unsigned addr = 0; // root
      traversalStack[stackPtr++] = addr;

      visionaray::aabb bounds{{(float)V.lower.x,(float)V.lower.y,(float)V.lower.z},
                              {(float)V.upper.x,(float)V.upper.y,(float)V.upper.z}};

      while (stackPtr) {
        auto node = sampler->abrBVH.node(addr);

        visionaray::aabb nodeBounds = node.get_bounds();

        auto I = intersect(nodeBounds,bounds);
        if (!I.empty()) {
          if (is_inner(node)) {
            addr = node.get_child(0);
            traversalStack[stackPtr++] = node.get_child(1);
          } else {
            for (unsigned i=node.get_indices().first; i<node.get_indices().last; ++i) {
              auto abr = sampler->abrBVH.primitive(i);
              const int *childList  = &sampler->model->abrs.leafList[abr.leafListBegin];
              const int  childCount = abr.leafListSize;
              for (int childID=0;childID<childCount;childID++) {
                const int brickID = childList[childID];
                const ExaBrick &brick = sampler->brickBuffer[brickID];
                minLevel = min(minLevel,brick.level);
                maxLevel = max(maxLevel,brick.level);
              }
            }
            addr = traversalStack[--stackPtr];
          }
        } else {
          addr = traversalStack[--stackPtr];
        }
      }
    }

    float boundsFindMajorant(box3i V, const std::vector<float> *rgbaCM)
    {
      float majorant = 0.f;
      int cmSize = rgbaCM ? rgbaCM->size()/4 : 0;

      unsigned traversalStack[128];
      unsigned stackPtr = 0;
      unsigned addr = 0; // root
      traversalStack[stackPtr++] = addr;

      visionaray::aabb bounds{{(float)V.lower.x,(float)V.lower.y,(float)V.lower.z},
                              {(float)V.upper.x,(float)V.upper.y,(float)V.upper.z}};

      while (stackPtr) {
        auto node = sampler->abrBVH.node(addr);

        visionaray::aabb nodeBounds = node.get_bounds();

        auto I = intersect(nodeBounds,bounds);
        if (!I.empty()) {
          if (is_inner(node)) {
            addr = node.get_child(0);
            traversalStack[stackPtr++] = node.get_child(1);
          } else {
            for (unsigned i=node.get_indices().first; i<node.get_indices().last; ++i) {
              auto abr = sampler->abrBVH.primitive(i);
              const int *childList  = &sampler->model->abrs.leafList[abr.leafListBegin];
              const int  childCount = abr.leafListSize;
              for (int childID=0;childID<childCount;childID++) {
                const int brickID = childList[childID];
                const ExaBrick &brick = sampler->brickBuffer[brickID];
                const float cellWidth = (float)(1<<brick.level);
                for (int z=0; z<+brick.size.z; ++z) {
                  for (int y=0; y<+brick.size.y; ++y) {
                    for (int x=0; x<+brick.size.x; ++x) {
                      visionaray::vec3i lower(brick.lower.x+x,brick.lower.y+y,brick.lower.z+z);
                      visionaray::vec3i upper(lower.x+1,lower.y+1,lower.z+1);
                      visionaray::aabb cellBounds(visionaray::vec3f(lower) - 0.5f*cellWidth,
                                                  visionaray::vec3f(upper) + 0.5f*cellWidth);
                      if (!intersect(cellBounds,bounds).empty()) {
                        int idx = brick.getIndexIndex({x,y,z});
                        float val = sampler->scalarBuffer[idx];
                        val -= valueRange.lower;
                        val /= valueRange.upper-valueRange.lower;
                        if (rgbaCM) {
                          int rgbaID = val*(cmSize-1);
                          float a = (*rgbaCM)[rgbaID];
                          majorant = fmaxf(majorant,a);
                        } else {
                          majorant = fmaxf(majorant,val);
                        }
                      }
                    }
                  }
                }
              }
            }
            addr = traversalStack[--stackPtr];
          }
        } else {
          addr = traversalStack[--stackPtr];
        }
      }

      return majorant;
    }

  };
} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0
