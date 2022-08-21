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

namespace exa {

  inline int __both__ iDivUp(int a, int b)
  {
    return (a + b - 1) / b;
  }

  __global__ void computeMaxOpacitiesGPU(float       *maxOpacities,
                                         const ABR   *abrs,
                                         const vec4f *colorMap,
                                         size_t       numABRs,
                                         size_t       numColors,
                                         range1f      xfRange)
  {
    size_t threadID = blockIdx.x * size_t(blockDim.x) + threadIdx.x;

    if (threadID >= numABRs)
      return;

    range1f valueRange = abrs[threadID].valueRange;

    if (valueRange.upper < valueRange.lower) {
      maxOpacities[threadID] = 0.f;
      return;
    }

    valueRange.lower -= xfRange.lower;
    valueRange.lower /= xfRange.upper-xfRange.lower;
    valueRange.upper -= xfRange.lower;
    valueRange.upper /= xfRange.upper-xfRange.lower;

    int lo = clamp(int(valueRange.lower*(numColors-1)),0,(int)numColors-1);
    int hi = clamp(int(valueRange.upper*(numColors-1)),0,(int)numColors-1);

    float maxOpacity = 0.f;
    for (int i=lo; i<=hi; ++i) {
      maxOpacity = fmaxf(maxOpacity,colorMap[i].w);
    }
    maxOpacities[threadID] = maxOpacity;
  }

  void ExaBrickModel::computeMaxOpacities(OWLContext owl,
                                          OWLBuffer colorMap,
                                          range1f xfRange)
  {
    if (maxOpacities) {
      owlBufferDestroy(maxOpacities);
    }

    size_t numABRs = owlBufferSizeInBytes(abrBuffer)/sizeof(ABR);
    size_t numColors = owlBufferSizeInBytes(colorMap)/sizeof(vec4f);

    maxOpacities = owlDeviceBufferCreate(owl, OWL_FLOAT,
                                         numABRs,
                                         nullptr);

    size_t numThreads = 1024;
    computeMaxOpacitiesGPU<<<iDivUp(numABRs, numThreads), numThreads>>>(
      (float *)owlBufferGetPointer(maxOpacities,0),
      (const ABR *)owlBufferGetPointer(abrBuffer,0),
      (const vec4f *)owlBufferGetPointer(colorMap,0),
      numABRs,numColors,xfRange);
  }
} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

