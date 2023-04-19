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

#include <stdint.h>

#include <cuda_runtime.h>

#include <owl/owl.h>
#include "owl/common/math/AffineSpace.h"
#include "owl/common/math/box.h"
#include "owl/common/math/random.h"

#include "sampler/AMRCellSampler.h"
#include "sampler/BigMeshSampler.h"
#include "sampler/ExaBrickSampler.h"
#include "sampler/ExaStitchSampler.h"
#include "sampler/QuickClustersSampler.h"
#include "common.h"
#include "Grid.cuh"
#include "KDTree.cuh"

namespace exa {

  struct MeshGeom {
    vec3i *indexBuffer;
    vec3f *vertexBuffer;
  };

  struct MajorantsGeom {
    box3f *domains;
    float *maxOpacities;
  };

  struct MacroCellGeom {
    vec3i  dims;
    vec3f  spacing;
    vec3f  origin;
    float *maxOpacities;
  };

  struct LaunchParams {
    union {
      AMRCellSampler::LP   acs;
      BigMeshSampler::LP   bms;
      ExaBrickSampler::LP  ebs;
      ExaStitchSampler::LP ess;
      QuickClustersSampler::LP qcs;
    } sampler;
    uint32_t *fbPointer;
    float    *fbDepth;
    float4   *accumBuffer;
    int       accumID;
    int       integrator;
    int       shadeMode;
    float    *maxOpacities;

    OptixTraversableHandle  meshBVH;
    OptixTraversableHandle  majorantBVH;
    KDTreeTraversableHandle majorantKDTree;
    GridTraversableHandle   majorantGrid;

    box3f     worldSpaceBounds;
    affine3f  voxelSpaceTransform;
    affine3f  lightSpaceTransform;

    struct {
#ifdef EXASTITCH_CUDA_TEXTURE_TF
      cudaTextureObject_t texture;
#else
      vec4f *values;
      int numValues;
#endif
      range1f             domain;
      float               opacityScale;
    } transferFunc;
    struct {
      vec3f org;
      vec3f dir_00;
      vec3f dir_du;
      vec3f dir_dv;
    } camera;
    struct {
      box2f value;
      box2f selection;
      int   active;
      int   selecting;
    } subImage;
    struct {
      float dt;
      int   heatMapEnabled;
      float heatMapScale;
      int   spp;
    } render;
    struct {
      int   enabled;
      vec3f N;
      float d;
    } clipPlanes[CLIP_PLANES_MAX];
    struct {
      vec3f pos;
      float intensity;
      int   on;
    } lights[LIGHTS_MAX];
    struct {
      int   enabled;
      float outsideOpacityScale;
      float outsideSaturationScale;
      box3f rois[ROIS_MAX];
    } roi;
  };

} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

