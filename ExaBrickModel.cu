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

  __global__ void computeMaxOpacitiesForBricks(float         *exaBrickMaxOpacities,
                                               const range1f *brickValueRanges,
                                               const vec4f   *colorMap,
                                               size_t         numBricks,
                                               size_t         numColors,
                                               range1f        xfRange)
  {
    size_t threadID = blockIdx.x * size_t(blockDim.x) + threadIdx.x;
    if (threadID >= numBricks)
      return;

    range1f valueRange = brickValueRanges[threadID];

    if (valueRange.upper < valueRange.lower) {
      exaBrickMaxOpacities[threadID] = 0.f;
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

    exaBrickMaxOpacities[threadID] = maxOpacity;
  }

  __global__ void computeMaxOpacitiesForABRs(float       *abrMaxOpacities,
                                             const ABR   *abrs,
                                             const vec4f *colorMap,
                                             size_t       numABRs,
                                             size_t       numColors,
                                             range1f      xfRange)
  {
    size_t threadID = blockIdx.x * size_t(blockDim.x) + threadIdx.x;
    if (threadID >= numABRs) return;

    const ABR &abr = abrs[threadID];
    range1f valueRange = abr.valueRange;

    if (valueRange.upper < valueRange.lower) {
      abrMaxOpacities[threadID] = 0.f;
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

    abrMaxOpacities[threadID] = maxOpacity;
  }

  void ExaBrickModel::computeMaxOpacities(OWLContext owl,
                                          OWLBuffer colorMap,
                                          range1f xfRange)
  {
#if EXA_STITCH_EXA_BRICK_TRAVERSAL_MODE == MC_DDA_TRAVERSAL || EXA_STITCH_EXA_BRICK_TRAVERSAL_MODE == MC_BVH_TRAVERSAL
    if (grid && grid->dims != vec3i(0)) {
      grid->computeMaxOpacities(owl,colorMap,xfRange);
    }
#endif

#if  EXA_STITCH_EXA_BRICK_SAMPLER_MODE == EXA_BRICK_SAMPLER_ABR_BVH || \
     EXA_STITCH_EXA_BRICK_TRAVERSAL_MODE == EXABRICK_ABR_TRAVERSAL
    {
      size_t numABRs = owlBufferSizeInBytes(abrBuffer)/sizeof(ABR);
      size_t numColors = owlBufferSizeInBytes(colorMap)/sizeof(vec4f);

      size_t numThreads = 1024;
      computeMaxOpacitiesForABRs<<<iDivUp(numABRs, numThreads), numThreads>>>(
        (float *)owlBufferGetPointer(abrMaxOpacities,0),
        (const ABR *)owlBufferGetPointer(abrBuffer,0),
        (const vec4f *)owlBufferGetPointer(colorMap,0),
        numABRs,numColors,xfRange);
    }

    owlGroupBuildAccel(abrBlas);
    owlGroupBuildAccel(abrTlas);
#endif

#if EXA_STITCH_EXA_BRICK_SAMPLER_MODE == EXA_BRICK_SAMPLER_EXT_BVH || \
    EXA_STITCH_EXA_BRICK_TRAVERSAL_MODE == EXABRICK_BVH_TRAVERSAL || \
    EXA_STITCH_EXA_BRICK_TRAVERSAL_MODE == EXABRICK_KDTREE_TRAVERSAL
    {
      size_t numColors = owlBufferSizeInBytes(colorMap)/sizeof(vec4f);

      size_t numThreads = 1024;
      computeMaxOpacitiesForBricks<<<iDivUp(bricks.size(), numThreads), numThreads>>>(
        (float *)owlBufferGetPointer(brickMaxOpacities,0),
        (const range1f *)owlBufferGetPointer(brickValueRanges,0),
        (const vec4f *)owlBufferGetPointer(colorMap,0),
        bricks.size(),numColors,xfRange);
    }

#if EXA_STITCH_EXA_BRICK_SAMPLER_MODE == EXA_BRICK_SAMPLER_EXT_BVH
    owlGroupBuildAccel(extBlas);
    owlGroupBuildAccel(extTlas);
#endif

#if EXA_STITCH_EXA_BRICK_TRAVERSAL_MODE == EXABRICK_BVH_TRAVERSAL
    owlGroupBuildAccel(brickBlas);
    owlGroupBuildAccel(brickTlas);
#endif

#endif
  }
} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0
