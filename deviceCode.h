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
#include "owl/common/math/box.h"
#include "owl/common/math/random.h"

using namespace owl;
using namespace owl::common;

typedef owl::interval<float> range1f;

#define RADIANCE_RAY_TYPE 0
#define SAMPLING_RAY_TYPE 1

#define CLIP_PLANES_MAX 1

#define EXA_STITCH_SAMPLER 0
#define AMR_CELL_SAMPLER   1
#define EXA_BRICK_SAMPLER  2

namespace exa {
  struct RayGen {
  };

  struct Gridlet {
    vec3i  lower;
    int    level;
    vec3i  dims;
    /*! offset into the scalar data index buffer (in which all gridlets
        are stored sequentially) */
    uint32_t begin;

    inline __both__
    box3f getBounds() const
    {
      const float cellWidth = 1<<level;
      return box3f((vec3f(lower) + 0.5f) * cellWidth,
                   (vec3f(lower) + vec3f(dims) + 0.5f) * cellWidth);
    }
  };

  struct AMRCell {
    vec3i pos;
    int   level;
  };

  struct GridletGeom {
    Gridlet *gridletBuffer;
    float   *gridletScalarBuffer;
  };

  struct StitchGeom {
    int   *indexBuffer;
    vec4f *vertexBuffer;
  };

  struct MeshGeom {
    vec3i *indexBuffer;
    vec3f *vertexBuffer;
  };

  struct AMRCellGeom {
    AMRCell *amrCellBuffer;
    float   *scalarBuffer;
  };

  struct ExaBrick { // TODO: move this closer to ExaBrickModel
    inline __both__
    size_t numCells() const
    {
      return size.x*size_t(size.y)*size.z;
    }
    
    inline __both__
    box3f getBounds() const
    {
      return box3f(vec3f(lower),
                   vec3f(lower + size*(1<<level)));
    }

    inline __both__
    box3f getDomain() const
    {
      const float cellWidth = 1<<level;
      return box3f(vec3f(lower) - 0.5f*cellWidth,
                   vec3f(lower) + (vec3f(size)+0.5f)*cellWidth);
    }

    inline __both__
    int getIndexIndex(const vec3i &idx) const
    {
      return begin + idx.x + size.x*(idx.y+size.y*idx.z);
    }

    vec3i    lower;
    vec3i    size;
    int      level;
    /*! offset into the scalar data index buffer (in which all bricks
        are stored sequentially) */
    uint32_t begin;
  };

  /*! denotes a region in which a given number of bricks overlap */
  struct ABR {
    /* space covered by this region - should not overlap any other
       brickregion */
    box3f   domain;
    /*! range of values of all cells overlapping this region */
    range1f valueRange;
    /*! offset in parent's leaflist class where our leaf list starts */
    int     leafListBegin;
    int     leafListSize;
    float   finestLevelCellWidth;
  };

  struct ExaBrickGeom {
    ABR *abrBuffer;
  };

  struct LaunchParams {
    uint32_t *fbPointer;
    float    *fbDepth;
    float4   *accumBuffer;
    int       accumID;
    int       shadeMode;
    int       sampler;
    OptixTraversableHandle sampleBVH;
    OptixTraversableHandle meshBVH;
    Gridlet  *gridletBuffer;
    box3f      modelBounds;

    // For ExaBrick benchmark
    ExaBrick *exaBrickBuffer;
    ABR      *abrBuffer;
    float    *scalarBuffer;
    int      *abrLeafListBuffer;
    float    *abrMaxOpacities;

    int       useDDA;

    struct {
      cudaTextureObject_t texture;
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
      vec3i    dims;
      range1f *valueRanges;
      float   *maxOpacities;
    } grid;
    struct {
      int   enabled;
      vec3f N;
      float d;
    } clipPlanes[CLIP_PLANES_MAX];
  };

} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

