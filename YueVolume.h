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
#include "owl/common/parallel/parallel_for.h"
#include "model/ExaBrickModel.h"
#include "sampler/ExaBrickSamplerCPU.h"

namespace exa {

  // Wrapper for ExaBricks volume to compute Yue majorant kd-trees
  struct YueVolume
  {
    box3f cellBounds;
    range1f xfDomain;
    ExaBrickSamplerCPU::SP sampler = nullptr;

    YueVolume(ExaBrickModel::SP model, const std::vector<float> *rgbaCM = nullptr,
              range1f xfAbsDomain = {0.f,1.f}, range1f xfRelDomain = {0.f,100.f})
    {
      cellBounds = model->cellBounds;

      // valueRange = model->valueRange;

      xfDomain = {
       xfAbsDomain.lower + (xfRelDomain.lower/100.f) * (xfAbsDomain.upper-xfAbsDomain.lower),
       xfAbsDomain.lower + (xfRelDomain.upper/100.f) * (xfAbsDomain.upper-xfAbsDomain.lower)
      };

      sampler = std::make_shared<ExaBrickSamplerCPU>();
      sampler->build(model);
    }

    box3f getBounds() const
    {
      return cellBounds;
    }

    range1f min_max(box3f V, const std::vector<float> *rgbaCM) const
    {
      range1f valueRange(1e31f,-1e31f);

      unsigned traversalStack[128];
      unsigned stackPtr = 0;
      traversalStack[stackPtr++] = 0; // root

      visionaray::aabb bounds{{V.lower.x,V.lower.y,V.lower.z},
                              {V.upper.x,V.upper.y,V.upper.z}};

      while (stackPtr) {
        unsigned addr = traversalStack[--stackPtr];
        auto node = sampler->abrBVH.node(addr);

        visionaray::aabb nodeBounds = node.get_bounds();

        auto I = intersect(nodeBounds,bounds);

        if (I.empty())
          continue;

        if (is_inner(node)) {
          traversalStack[stackPtr++] = node.get_child(0);
          traversalStack[stackPtr++] = node.get_child(1);
        } else {
          for (unsigned i=node.get_indices().first; i<node.get_indices().last; ++i) {
            auto abr = sampler->abrBVH.primitive(i);
            if (V.contains(abr.domain.lower) && V.contains(abr.domain.upper)) {
              valueRange.extend(abr.valueRange);
            } else {
              const int *childList  = &sampler->model->abrs.leafList[abr.leafListBegin];
              const int  childCount = abr.leafListSize;
              for (int childID=0;childID<childCount;childID++) {
                const int brickID = childList[childID];
                const ExaBrick &brick = sampler->brickBuffer[brickID];

                box3f bd = brick.getDomain();
                visionaray::aabb brickBounds(visionaray::vec3f(bd.lower.x,bd.lower.y,bd.lower.z),
                                             visionaray::vec3f(bd.upper.x,bd.upper.y,bd.upper.z));

                auto II = intersect(nodeBounds,brickBounds);

                if (II.empty())
                  continue;

                const float cellWidth = (float)(1<<brick.level);

                for (int z=0; z<brick.size.z; ++z) {
                  for (int y=0; y<brick.size.y; ++y) {
                    for (int x=0; x<brick.size.x; ++x) {
                      visionaray::vec3i lower(brick.lower.x+x*cellWidth,
                                              brick.lower.y+y*cellWidth,
                                              brick.lower.z+z*cellWidth);
                      visionaray::vec3i upper(lower.x+cellWidth,
                                              lower.y+cellWidth,
                                              lower.z+cellWidth);
                      visionaray::aabb cellBounds(visionaray::vec3f(lower) - 0.5f*cellWidth,
                                                  visionaray::vec3f(upper) + 0.5f*cellWidth);
                      if (!intersect(cellBounds,bounds).empty()) {
                        int idx = brick.getIndexIndex({x,y,z});
                        float val = sampler->scalarBuffer[idx];
                        valueRange.extend(val);
                      }
                    }
                  }
                }

              }
            }
          }
        }
      }

      if (valueRange.lower > valueRange.upper)
        return range1f{0.f,0.f};

      assert(rgbaCM);

      range1f result{1e31f,-1e31f};

      valueRange.lower -= xfDomain.lower;
      valueRange.lower /= (xfDomain.upper-xfDomain.lower);

      valueRange.upper -= xfDomain.lower;
      valueRange.upper /= (xfDomain.upper-xfDomain.lower);

      int cmSize = rgbaCM->size()/4;

#ifdef EXASTITCH_CUDA_TEXTURE_TF
      const int idx_lo = clamp(int(valueRange.lower*cmSize),0,cmSize-1);
      const int idx_hi = clamp(int(valueRange.upper*cmSize)+1,0,cmSize-1);
#else
      const int idx_lo = clamp(int(valueRange.lower*cmSize-1),0,cmSize-1);
      const int idx_hi = clamp(int(valueRange.upper*cmSize-1)+1,0,cmSize-1);
#endif

      for (int i=idx_lo;i<=idx_hi;++i) {
        float alpha = (*rgbaCM)[i*4+3];
        result.lower = fminf(result.lower,alpha);
        result.upper = fmaxf(result.upper,alpha);
      }

      return result;
    }

  };
} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0
