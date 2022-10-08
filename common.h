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
#include "owl/common/math/AffineSpace.h"
#include "owl/common/math/box.h"
#include "owl/common/math/random.h"

using namespace owl;
using namespace owl::common;

typedef owl::interval<float> range1f;

#define RADIANCE_RAY_TYPE 0
#define SAMPLING_RAY_TYPE 1

#define CLIP_PLANES_MAX 1
#define LIGHTS_MAX      4

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

#define EXABRICK_ABR_TRAVERSAL    0
#define MC_DDA_TRAVERSAL          1
#define MC_BVH_TRAVERSAL          2
#define EXABRICK_KDTREE_TRAVERSAL 3
#define EXABRICK_BVH_TRAVERSAL    4

namespace exa {
  typedef int SamplingMode;
  typedef int TraversalMode;
} // ::exa

