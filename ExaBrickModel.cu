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

#include "ExaBrickModel.h"
#include "atomicOp.cuh"

namespace exa {

  inline int64_t __both__ iDivUp(int64_t a, int64_t b)
  {
    return (a + b - 1) / b;
  }

  __global__ void computeMaxOpacitiesGPU(float       *abrMaxOpacities,
                                         float       *exaBrickMaxOpacities,
                                         const ABR   *abrs,
                                         const int   *abrLeafList,
                                         const vec4f *colorMap,
                                         size_t       numABRs,
                                         size_t       numColors,
                                         range1f      xfRange)
  {
    size_t threadID = blockIdx.x * size_t(blockDim.x) + threadIdx.x;
    if (threadID >= numABRs) return;

    const ABR &abr = abrs[threadID];
    const int *childList = &abrLeafList[abr.leafListBegin];
    const int childCount = abr.leafListSize;
    range1f valueRange = abr.valueRange;

    const auto setOpacity = [&] (float opacity) {
      for (int childID=0;childID<childCount;childID++) {
        const int brickID = childList[childID];
        atomicMax(exaBrickMaxOpacities + brickID, opacity);
      }
      abrMaxOpacities[threadID] = opacity;
    };

    if (valueRange.upper < valueRange.lower) {
      setOpacity(0.f);
      return;
    }

    valueRange.lower -= xfRange.lower;
    valueRange.lower /= xfRange.upper-xfRange.lower;
    valueRange.upper -= xfRange.lower;
    valueRange.upper /= xfRange.upper-xfRange.lower;

    int lo = clamp(int(valueRange.lower*(numColors-1)),  0,(int)numColors-1);
    int hi = clamp(int(valueRange.upper*(numColors-1))+1,0,(int)numColors-1);

    float maxOpacity = 0.f;
    for (int i=lo; i<=hi; ++i) {
      maxOpacity = fmaxf(maxOpacity,colorMap[i].w);
    }

    setOpacity(maxOpacity);
  }

  void ExaBrickModel::computeMaxOpacities(OWLContext owl,
                                          OWLBuffer colorMap,
                                          range1f xfRange)
  {
    size_t numABRs = owlBufferSizeInBytes(abrBuffer)/sizeof(ABR);
    size_t numColors = owlBufferSizeInBytes(colorMap)/sizeof(vec4f);

    // maximum opacity buffers are pre-located in ExaBrickModel.cpp
    owlBufferClear(brickMaxOpacities);

    size_t numThreads = 1024;
    computeMaxOpacitiesGPU<<<iDivUp(numABRs, numThreads), numThreads>>>(
      (float *)owlBufferGetPointer(abrMaxOpacities,0),
      (float *)owlBufferGetPointer(brickMaxOpacities,0),
      (const ABR *)owlBufferGetPointer(abrBuffer,0),
      (const int *)owlBufferGetPointer(abrLeafListBuffer,0),
      (const vec4f *)owlBufferGetPointer(colorMap,0),
      numABRs,numColors,xfRange);
  }
} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0
