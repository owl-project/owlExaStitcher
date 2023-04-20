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
#include <owl/common/math/AffineSpace.h>
#include <owl/common/math/box.h>
#include <owl/common/math/random.h>

#include <cuda_runtime.h>

using namespace owl;
using namespace owl::common;

typedef owl::interval<float> range1f;

#define RADIANCE_RAY_TYPE 0
#define SAMPLING_RAY_TYPE 1

#define CLIP_PLANES_MAX 1
#define LIGHTS_MAX      4
#define ROIS_MAX        8

#define EXA_STITCH_SAMPLER 0
#define AMR_CELL_SAMPLER   1
#define EXA_BRICK_SAMPLER  2

#define EXA_BRICK_SAMPLER_ABR_BVH 0
#define EXA_BRICK_SAMPLER_EXT_BVH 1

#define PATH_TRACING_INTEGRATOR 0
#define DIRECT_LIGHT_INTEGRATOR 1
#define RAY_MARCHING_INTEGRATOR 2

#define SHADE_MODE_DEFAULT  0
#define SHADE_MODE_GRIDLETS 1
#define SHADE_MODE_TEASER   2

#define EXABRICK_ABR_TRAVERSAL     0
#define MC_DDA_TRAVERSAL           1
#define MC_BVH_TRAVERSAL           2
#define EXABRICK_KDTREE_TRAVERSAL  3
#define EXABRICK_BVH_TRAVERSAL     4
#define EXABRICK_EXT_BVH_TRAVERSAL 5

namespace exa {
  typedef int SamplingMode;
  typedef int TraversalMode;

  inline __both__
  float lerp(const float val1, const float val2, const float x)
  {
    return (1.f-x)*val1+x*val2;
  }

  inline void printGPUMemory(std::string str)
  {
    size_t free, total;
    cudaMemGetInfo(&free, &total);
    size_t used = total-free;
    std::cout << str << ": " << prettyNumber(used) << '\n';
  }

#ifdef __CUDA_ARCH__

#define DEBUGGING 1
#define DBG_X (owl::getLaunchDims().x/2)
#define DBG_Y (owl::getLaunchDims().y/2)

  __device__ inline bool debug()
  {
#if DEBUGGING
    return (owl::getLaunchIndex().x == DBG_X && owl::getLaunchIndex().y == DBG_Y);
#else
     return false;
#endif
  }

#endif
} // ::exa

