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
#include "model/ExaBrickModel.h"
#include "sampler/ExaBrickSamplerCPU.h"

#include "owl/common/parallel/parallel_for.h"

namespace exa {

  // Wrapper for ExaBricks volume to compute Yue majorant kd-trees
  struct YueVolume
  {
    box3i cellBounds;
    range1f valueRange;
    ExaBrickSamplerCPU::SP sampler = nullptr;
    std::vector<range1f> valueRangesPerABR;

    YueVolume(ExaBrickModel::SP model, const std::vector<float> *rgbaCM = nullptr)
    {
      cellBounds = box3i(vec3i(model->cellBounds.lower),
                         vec3i(model->cellBounds.upper));

      valueRange = model->valueRange;

      sampler = std::make_shared<ExaBrickSamplerCPU>();
      sampler->build(model);

      valueRangesPerABR.resize(model->abrs.value.size());

      // this code block computes the accurate value range for each ABR
      if (!rgbaCM) {
        for (size_t i=0; i<model->abrs.value.size(); ++i) {
          valueRangesPerABR[i] = model->abrs.value[i].valueRange;
        }
      } else {
        std::cout << "Pre-computing color map value ranges for ABRs..." << std::flush;

        int cmSize = rgbaCM->size()/4;

        owl::parallel_for(sampler->abrBVH.num_primitives(), [&] (int i) {
          auto abr = sampler->abrBVH.primitive(i); // need to be in BVH (indirect) order!
          auto V = abr.domain;
          visionaray::aabb domain{{(float)V.lower.x,(float)V.lower.y,(float)V.lower.z},
                                  {(float)V.upper.x,(float)V.upper.y,(float)V.upper.z}};
          const int *childList  = &sampler->model->abrs.leafList[abr.leafListBegin];
          const int  childCount = abr.leafListSize;
          float minValue = 1e31f, maxValue = -1e31f;
          for (int childID=0;childID<childCount;childID++) {
            const int brickID = childList[childID];
            const ExaBrick &brick = sampler->brickBuffer[brickID];
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
                  if (!intersect(cellBounds,domain).empty()) {
                    int idx = brick.getIndexIndex({x,y,z});
                    float val = sampler->scalarBuffer[idx];
                    val -= valueRange.lower;
                    val /= valueRange.upper-valueRange.lower;
                    int rgbaID = val*(cmSize-1);
                    float a = (*rgbaCM)[rgbaID*4+3];
                    minValue = fminf(minValue,a);
                    maxValue = fmaxf(maxValue,a);
                  }
                }
              }
            }
          }
          valueRangesPerABR[i] = {minValue,maxValue};
        });

        std::cout << " done\n";
      }
    }

    box3f getBounds() const
    {
      return box3f(vec3f(cellBounds.lower),vec3f(cellBounds.upper));
    }

    void min_max(box3f V, float &minValue, float &maxValue,
                 const std::vector<float> *rgbaCM)
    {
      minValue =  1e31f;
      maxValue = -1e31f;

      int cmSize = rgbaCM ? rgbaCM->size()/4 : 0;

      unsigned traversalStack[128];
      unsigned stackPtr = 0;
      unsigned addr = 0; // root
      traversalStack[stackPtr++] = addr;

      visionaray::aabb bounds{{V.lower.x,V.lower.y,V.lower.z},
                              {V.upper.x,V.upper.y,V.upper.z}};

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
              static uint64_t lookupCount = 0;
              static uint64_t cachedCount = 0;
              lookupCount++;
              if (lookupCount && ((lookupCount%(1ull<<22))==0)) {
                std::cout << "ABR cache hits: " << prettyNumber(cachedCount)
                          << '/' << prettyNumber(lookupCount) << std::endl;
              }
              if (V.contains(abr.domain.lower) && V.contains(abr.domain.upper)) {
                // access with i b/c the ranges are stored in the same order
                // as are the ABRs in the BVH!
                minValue = fminf(minValue,valueRangesPerABR[i].lower);
                maxValue = fmaxf(maxValue,valueRangesPerABR[i].upper);
                cachedCount++;
              } else {
                const int *childList  = &sampler->model->abrs.leafList[abr.leafListBegin];
                const int  childCount = abr.leafListSize;
                for (int childID=0;childID<childCount;childID++) {
                  const int brickID = childList[childID];
                  const ExaBrick &brick = sampler->brickBuffer[brickID];
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
                          val -= valueRange.lower;
                          val /= valueRange.upper-valueRange.lower;
                          if (rgbaCM) {
                            int rgbaID = val*(cmSize-1);
                            float a = (*rgbaCM)[rgbaID*4+3];
                            minValue = fminf(minValue,a);
                            maxValue = fmaxf(maxValue,a);
                          } else {
                            minValue = fminf(minValue,val);
                            maxValue = fmaxf(maxValue,val);
                          }
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
    }

    float value(float x, float y, float z)
    {
      Sample s = sample(*sampler,{},{x,y,z});
      return s.value;
    }

    void boundsFindMinMaxLevel(box3f V, int &minLevel, int &maxLevel)
    {
      minLevel = 1000000000;
      maxLevel = 0;

      unsigned traversalStack[128];
      unsigned stackPtr = 0;
      unsigned addr = 0; // root
      traversalStack[stackPtr++] = addr;

      visionaray::aabb bounds{{V.lower.x,V.lower.y,V.lower.z},
                              {V.upper.x,V.upper.y,V.upper.z}};

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

  };
} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0
