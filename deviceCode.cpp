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

#include <float.h>
#include "deviceCode.h"
#include "Grid.cuh"
#include "Plane.h"
#include "UElems.h"

using owl::vec2f;
using owl::vec2i;
using owl::vec3f;
using owl::vec3i;
using owl::vec4f;
using owl::vec4i;

#define ELEM_TAG 0x1
#define GRID_TAG 0x2

namespace exa {

  extern "C" __constant__ LaunchParams optixLaunchParams;

  // ------------------------------------------------------------------
  // helpers
  // ------------------------------------------------------------------

  typedef owl::common::LCG<4> Random;

#define DEBUGGING 1
#define DBG_X (getLaunchDims().x/2)
#define DBG_Y (getLaunchDims().y/2)

  __device__ inline bool debug()
  {
#if DEBUGGING
    return (getLaunchIndex().x == DBG_X && getLaunchIndex().y == DBG_Y);
#else
     return false;
#endif
  }

  inline  __device__
  vec3f backGroundColor()
  {
    const vec2i pixelID = owl::getLaunchIndex();
    const float t = pixelID.y / (float)optixGetLaunchDimensions().y;
    const vec3f c = (1.0f - t)*vec3f(1.0f, 1.0f, 1.0f) + t * vec3f(0.5f, 0.7f, 1.0f);
    return c;
  }

  inline __device__
  float lerp(const float val1, const float val2, const float x)
  {
    return (1.f-x)*val1+x*val2;
  };

  inline __device__
  bool intersect(const Ray &ray,
                 const box3f &box,
                 float &t0,
                 float &t1)
  {
    vec3f lo = (box.lower - ray.origin) / ray.direction;
    vec3f hi = (box.upper - ray.origin) / ray.direction;
    
    vec3f nr = min(lo,hi);
    vec3f fr = max(lo,hi);

    t0 = max(ray.tmin,reduce_max(nr));
    t1 = min(ray.tmax,reduce_min(fr));

    return t0 < t1;
  }

  inline __device__
  float intersect(const Ray &ray, const Plane &plane, bool &backFace)
  {
    const float s = dot(plane.N,ray.direction);
    if (s == 0.f)
      return FLT_MAX;
    backFace = s > 0.f;
    return (plane.d - dot(plane.N,ray.origin)) / s;
  }

  inline __device__ vec3f randomColor(unsigned idx)
  {
    unsigned int r = (unsigned int)(idx*13*17 + 0x234235);
    unsigned int g = (unsigned int)(idx*7*3*5 + 0x773477);
    unsigned int b = (unsigned int)(idx*11*19 + 0x223766);
    return vec3f((r&255)/255.f,
                 (g&255)/255.f,
                 (b&255)/255.f);
  }

  inline  __device__ Ray generateRay(const vec2f screen)
  {
    auto &lp = optixLaunchParams;
    vec3f org = lp.camera.org;
    vec3f dir
      = lp.camera.dir_00
      + screen.u * lp.camera.dir_du
      + screen.v * lp.camera.dir_dv;
    dir = normalize(dir);
    if (fabs(dir.x) < 1e-5f) dir.x = 1e-5f;
    if (fabs(dir.y) < 1e-5f) dir.y = 1e-5f;
    if (fabs(dir.z) < 1e-5f) dir.z = 1e-5f;
    return Ray(org,dir,0.f,1e10f);
  }

  inline __device__ box2f subImageUV()
  {
    return optixLaunchParams.subImage.value;
  }
 
  inline __device__ box2i subImageWin()
  {
    const vec2i lo = clamp(vec2i(optixLaunchParams.subImage.value.lower*vec2f(owl::getLaunchDims())),
                           vec2i(0),vec2i(owl::getLaunchDims()-1));

    const vec2i hi = clamp(vec2i(optixLaunchParams.subImage.value.upper*vec2f(owl::getLaunchDims())),
                           vec2i(0),vec2i(owl::getLaunchDims()-1));
    return {lo,hi};
  }

  inline __device__ box2f subImageSelectionUV()
  {
    return optixLaunchParams.subImage.selection;
  }
 
  inline __device__ box2i subImageSelectionWin()
  {
    const vec2i lo = clamp(vec2i(optixLaunchParams.subImage.selection.lower*vec2f(owl::getLaunchDims())),
                           vec2i(0),vec2i(owl::getLaunchDims()-1));

    const vec2i hi = clamp(vec2i(optixLaunchParams.subImage.selection.upper*vec2f(owl::getLaunchDims())),
                           vec2i(0),vec2i(owl::getLaunchDims()-1));
    return {lo,hi};
  }
 
  inline __device__ void make_orthonormal_basis(vec3f &u, vec3f &v, const vec3f &w)
  {
    v = fabsf(w.x) > fabsf(w.y) ? normalize(vec3f(-w.z,0.f,w.x))
                                : normalize(vec3f(0.f,w.z,-w.y));
    u = cross(v, w);
  }

  inline  __device__ vec4f over(const vec4f &A, const vec4f &B)
  {
    return A + (1.f-A.w)*B;
  }

  inline __device__ vec3f hue_to_rgb(float hue)
  {
    float s = saturate( hue ) * 6.0f;
    float r = saturate( fabsf(s - 3.f) - 1.0f );
    float g = saturate( 2.0f - fabsf(s - 2.0f) );
    float b = saturate( 2.0f - fabsf(s - 4.0f) );
    return vec3f(r, g, b); 
  }
    
  inline __device__ vec3f temperature_to_rgb(float t)
  {
    float K = 4.0f / 6.0f;
    float h = K - K * t;
    float v = .5f + 0.5f * t;    return v * hue_to_rgb(h);
  }
    
                                    
  inline __device__
  vec3f heatMap(float t)
  {
#if 1
    return temperature_to_rgb(t);
#else
    if (t < .25f) return lerp(vec3f(0.f,1.f,0.f),vec3f(0.f,1.f,1.f),(t-0.f)/.25f);
    if (t < .5f)  return lerp(vec3f(0.f,1.f,1.f),vec3f(0.f,0.f,1.f),(t-.25f)/.25f);
    if (t < .75f) return lerp(vec3f(0.f,0.f,1.f),vec3f(1.f,1.f,1.f),(t-.5f)/.25f);
    if (t < 1.f)  return lerp(vec3f(1.f,1.f,1.f),vec3f(1.f,0.f,0.f),(t-.75f)/.25f);
    return vec3f(1.f,0.f,0.f);
#endif
  }


  struct VolumePRD {
    int primID;
    float value;
  };


  // ------------------------------------------------------------------
  // Gridlet user geometry
  // ------------------------------------------------------------------

  OPTIX_BOUNDS_PROGRAM(GridletGeomBounds)(const void* geomData,
                                          box3f& result,
                                          int leafID)
  {
    const GridletGeom &self = *(const GridletGeom *)geomData;

    const Gridlet &gridlet = self.gridletBuffer[leafID];
    vec3i lower = gridlet.lower * (1<<gridlet.level);
    vec3i upper = lower + gridlet.dims * (1<<gridlet.level);

    vec3f halfCell = vec3f(1<<gridlet.level)*.5f;

    result = box3f(vec3f(lower)+halfCell,vec3f(upper)+halfCell);
  }

  OPTIX_INTERSECT_PROGRAM(GridletGeomIsect)()
  {
    const GridletGeom &self = owl::getProgramData<GridletGeom>();
    int primID = optixGetPrimitiveIndex();
    vec3f pos = optixGetObjectRayOrigin();

    const Gridlet &gridlet = self.gridletBuffer[primID];
    vec3f lower(gridlet.lower * (1<<gridlet.level));
    vec3f upper = lower + vec3f(gridlet.dims * (1<<gridlet.level));

    vec3f halfCell = vec3f(1<<gridlet.level)*.5f;
    lower += halfCell;
    upper += halfCell;

    //box3f bounds((vec3f)lower,(vec3f)upper);
    pos -= lower;
    box3f bounds(vec3f(0.f),(vec3f)(upper-lower));

    if (bounds.contains(pos)) {
      vec3i numScalars = gridlet.dims+1;
      vec3f posInLevelCoords = pos / (1<<gridlet.level);
      vec3i imin(posInLevelCoords);
      vec3i imax = min(imin+1,numScalars-1);

      auto linearIndex = [numScalars](const int x, const int y, const int z) {
        return z*numScalars.y*numScalars.x + y*numScalars.x + x;
      };

      float f1 = gridlet.scalars[linearIndex(imin.x,imin.y,imin.z)];
      float f2 = gridlet.scalars[linearIndex(imax.x,imin.y,imin.z)];
      float f3 = gridlet.scalars[linearIndex(imin.x,imax.y,imin.z)];
      float f4 = gridlet.scalars[linearIndex(imax.x,imax.y,imin.z)];

      float f5 = gridlet.scalars[linearIndex(imin.x,imin.y,imax.z)];
      float f6 = gridlet.scalars[linearIndex(imax.x,imin.y,imax.z)];
      float f7 = gridlet.scalars[linearIndex(imin.x,imax.y,imax.z)];
      float f8 = gridlet.scalars[linearIndex(imax.x,imax.y,imax.z)];

      if (!isnan(f1) && !isnan(f2) && !isnan(f3) && !isnan(f4) &&
          !isnan(f5) && !isnan(f6) && !isnan(f7) && !isnan(f8)) {

        vec3f frac = posInLevelCoords-vec3f(vec3i(posInLevelCoords));

        // if (debug()) printf("%f,%f,%f -- %f,%f,%f -- %f,%f,%f\n",
        //                     pos.x,pos.y,pos.z,
        //                     posInLevelCoords.x,
        //                     posInLevelCoords.y,
        //                     posInLevelCoords.z,
        //                     frac.x,frac.y,frac.z);

        float f12 = lerp(f1,f2,frac.x);
        float f56 = lerp(f5,f6,frac.x);
        float f34 = lerp(f3,f4,frac.x);
        float f78 = lerp(f7,f8,frac.x);

        float f1234 = lerp(f12,f34,frac.y);
        float f5678 = lerp(f56,f78,frac.y);

        float value = lerp(f1234,f5678,frac.z);

        if (optixReportIntersection(0.f,0)) {
          VolumePRD& prd = owl::getPRD<VolumePRD>();
          prd.value = value;
          prd.primID = primID;
        }
      }
    }
  }

  OPTIX_CLOSEST_HIT_PROGRAM(GridletGeomCH)()
  {
  }

  // ------------------------------------------------------------------
  // Stitching user geometry
  // ------------------------------------------------------------------

  OPTIX_BOUNDS_PROGRAM(StitchGeomBounds)(const void* geomData,
                                         box3f& result,
                                         int leafID)
  {
    const StitchGeom &self = *(const StitchGeom *)geomData;

    result = box3f();
    for (int i=0; i<8; ++i) {
      int idx = self.indexBuffer[leafID*8+i];
      if (idx < 0) break;
      vec3f v(self.vertexBuffer[idx]);
      result.extend(v);
      // printf("%i: %f,%f,%f\n",idx,v.x,v.y,v.z);
    }
  }

  OPTIX_INTERSECT_PROGRAM(StitchGeomIsect)()
  {
    const StitchGeom &self = owl::getProgramData<StitchGeom>();
    int primID = optixGetPrimitiveIndex();
    vec3f pos = optixGetObjectRayOrigin();
    float value = 0.f;

    vec4f v[8];
    int numVerts = 0;
    for (int i=0; i<8; ++i) {
      int idx = self.indexBuffer[primID*8+i];
      if (idx >= 0) {
        numVerts++;
        v[i] = self.vertexBuffer[idx];
      }
    }

    bool hit=numVerts==4 && intersectTet(value,pos,v[0],v[1],v[2],v[3])
          || numVerts==5 && intersectPyrEXT(value,pos,v[0],v[1],v[2],v[3],v[4])
          || numVerts==6 && intersectWedgeEXT(value,pos,v[0],v[1],v[2],v[3],v[4],v[5])
          || numVerts==8 && intersectHexEXT(value,pos,v[0],v[1],v[2],v[3],v[4],v[5],v[6],v[7]);

    if (hit && optixReportIntersection(0.f,0)) {
      VolumePRD& prd = owl::getPRD<VolumePRD>();
      prd.value = value;
      prd.primID = primID;
    }
  }

  OPTIX_CLOSEST_HIT_PROGRAM(StitchGeomCH)()
  {
  }


  // ------------------------------------------------------------------
  // AMR cells user geometry (for eval only!)
  // ------------------------------------------------------------------

  struct BasisPRD {
    float sumWeights;
    float sumWeightedValues;
  };

  OPTIX_BOUNDS_PROGRAM(AMRCellGeomBounds)(const void* geomData,
                                          box3f& result,
                                          int leafID)
  {
    const AMRCellGeom &self = *(const AMRCellGeom *)geomData;

    result = box3f();
    const AMRCell &cell = self.amrCellBuffer[leafID];

    vec3f halfCell = vec3f(1<<cell.level)*.5f;

    box3f bounds(vec3f(cell.pos)-halfCell,
                 vec3f(cell.pos+vec3i(1<<cell.level))+halfCell);
    result.extend(bounds);
  }

  OPTIX_INTERSECT_PROGRAM(AMRCellGeomIsect)()
  {
    const AMRCellGeom &self = owl::getProgramData<AMRCellGeom>();
    int primID = optixGetPrimitiveIndex();
    vec3f pos = optixGetObjectRayOrigin();

    const AMRCell &cell = self.amrCellBuffer[primID];

    vec3f halfCell = vec3f(1<<cell.level)*.5f;

    box3f bounds(vec3f(cell.pos)-halfCell,
                 vec3f(cell.pos+vec3i(1<<cell.level))+halfCell);

    if (bounds.contains(pos)) {
      BasisPRD& prd = owl::getPRD<BasisPRD>();
      const vec3f center = bounds.center();

      const float weight =  (1.f-fabsf(pos.x-center.x)/(bounds.size().x*.5f))
                          * (1.f-fabsf(pos.y-center.y)/(bounds.size().y*.5f))
                          * (1.f-fabsf(pos.z-center.z)/(bounds.size().z*.5f));
      const float scalar = self.scalarBuffer[primID];

      // if (debug()) {
      //   printf("pos: (%f,%f,%f), center: (%f,%f,%f), scalar: %f, weight: %f\n",
      //          pos.x, pos.y, pos.z,
      //          center.x, center.y, center.z,
      //          scalar, weight);
      // }

      prd.sumWeights += weight;
      prd.sumWeightedValues += scalar*weight;
    }
  }

  OPTIX_CLOSEST_HIT_PROGRAM(AMRCellGeomCH)()
  {
  }

  // ------------------------------------------------------------------
  // Triangle meshes
  // ------------------------------------------------------------------

  struct MeshPRD {
    int primID;
    float t_hit;
    vec3f Ng;
    vec3f kd;
  };

  OPTIX_CLOSEST_HIT_PROGRAM(MeshGeomCH)()
  {
    const MeshGeom &self = owl::getProgramData<MeshGeom>();
    MeshPRD &prd = owl::getPRD<MeshPRD>();
    prd.t_hit    = optixGetRayTmax();
    prd.primID   = optixGetPrimitiveIndex();
    vec3i index  = self.indexBuffer[prd.primID];
    vec3f v1     = self.vertexBuffer[index.x];
    vec3f v2     = self.vertexBuffer[index.y];
    vec3f v3     = self.vertexBuffer[index.z];
    prd.Ng       = normalize(vec3f(optixTransformNormalFromObjectToWorldSpace(cross(v2-v1,v3-v1))));
    prd.kd       = vec3f(.8f);
  }


  
  struct Sample {
    int      primID;
    float    value;
    vec3f    gradient;
    unsigned cellTag;
  };

  inline __device__ Sample sampleVolume(const vec3f pos)
  {
    auto& lp = optixLaunchParams;

#if 0
    BasisPRD prd{0.f,0.f};
    owl::Ray ray(pos,vec3f(1.f),0.f,0.f);

    owl::traceRay(lp.amrCellBVH,ray,prd,
                  OPTIX_RAY_FLAG_DISABLE_ANYHIT);

    int primID = -1;
    float value = 0.f;
    if (prd.sumWeights > 0.f) {
      primID = 0; // non-negative dummy value
      value = prd.sumWeightedValues/prd.sumWeights;
    }
    return {primID,value,{0.f,0.f,0.f}};
#else
    VolumePRD prd{-1,0.f};
    owl::Ray ray(pos,vec3f(1.f),0.f,0.f);
    unsigned cellTag = 0;

    owl::traceRay(lp.gridletBVH,ray,prd,
                  OPTIX_RAY_FLAG_DISABLE_ANYHIT);

    if (prd.primID != -1)
      cellTag = GRID_TAG;
    else {
      owl::traceRay(lp.boundaryCellBVH,ray,prd,
                    OPTIX_RAY_FLAG_DISABLE_ANYHIT);
      if (prd.primID != -1)
        cellTag = ELEM_TAG;
    }

    return {prd.primID,prd.value,{0.f,0.f,0.f},cellTag};
#endif
  }

  inline __device__ Sample sampleVolume(float x, float y, float z)
  {
    return sampleVolume({x,y,z});
  }

  inline __device__ Sample sampleVolumeWithGradient(const vec3f pos)
  {
    Sample s = sampleVolume(pos);

    const vec3f delta = 1.f;

    s.gradient = vec3f(+sampleVolume(pos.x+delta.x,pos.y,pos.z).value
                       -sampleVolume(pos.x-delta.x,pos.y,pos.z).value,
                       +sampleVolume(pos.x,pos.y+delta.y,pos.z).value
                       -sampleVolume(pos.x,pos.y-delta.y,pos.z).value,
                       +sampleVolume(pos.x,pos.y,pos.z+delta.z).value
                       -sampleVolume(pos.x,pos.y,pos.z-delta.z).value);

    return s;
  }

  inline __device__ vec3f getLe(const Sample s)
  {
    auto& lp = optixLaunchParams;
    vec4f xf = tex1D<float4>(lp.transferFunc.texture,s.value);
    if (s.value > .99f)
    //if (s.value < .2f)
      return {120.f,60.f,45.f};
    return 0.f;
  }

  // ------------------------------------------------------------------
  // Woodcock path tracer
  // ------------------------------------------------------------------

  enum CollisionType {
    Boundary, Scattering, Emission,
  };

  enum ShadeMode {
    Default,  /* ordinary mode assigning colors from the TF */
    Gridlets, /* show gridlets as checkerboard */
    Teaser,   /* paper teaser */
  };

  template <ShadeMode SM>
  inline __device__
  void sampleInteraction(const Ray     &ray,
                         CollisionType &type,     /* scattering,emission,... */
                         vec3f         &pos,      /* position of interaction */
                         float         &Tr,       /* transmission samples [0,1] */
                         vec3f         &Le,       /* emitted radiance */
                         vec4f         &xf,
                         Random        &random)
  {
    auto& lp = optixLaunchParams;

    Le = 0.f;

    const vec3f rcp_dir = rcp(ray.direction);

    const vec3f lo = (lp.modelBounds.lower - ray.origin) * rcp_dir;
    const vec3f hi = (lp.modelBounds.upper - ray.origin) * rcp_dir;

    const vec3f tnear = min(lo,hi);
    const vec3f tfar  = max(lo,hi);

    vec3i cellID = projectOnGrid(ray.origin,lp.grid.dims,lp.modelBounds);

    // Distance in world space to get from cell to cell
    const vec3f dist((tfar-tnear)/vec3f(lp.grid.dims));

    // Cell increment
    const vec3i step = {
      ray.direction.x > 0.f ? 1 : -1,
      ray.direction.y > 0.f ? 1 : -1,
      ray.direction.z > 0.f ? 1 : -1
    };

    // Stop when we reach grid borders
    const vec3i stop = {
      ray.direction.x > 0.f ? lp.grid.dims.x : -1,
      ray.direction.y > 0.f ? lp.grid.dims.y : -1,
      ray.direction.z > 0.f ? lp.grid.dims.z : -1
    };

    // Increment in world space
    vec3f tnext = {
      ray.direction.x > 0.f ? tnear.x + float(cellID.x+1) * dist.x
                            : tnear.x + float(lp.grid.dims.x-cellID.x) * dist.x,
      ray.direction.y > 0.f ? tnear.y + float(cellID.y+1) * dist.y
                            : tnear.y + float(lp.grid.dims.y-cellID.y) * dist.y,
      ray.direction.z > 0.f ? tnear.z + float(cellID.z+1) * dist.z
                            : tnear.z + float(lp.grid.dims.z-cellID.z) * dist.z
    };


    float t0 = max(ray.tmin,0.f);

    while (1) { // DDA loop
    
      const float t1 = min(reduce_min(tnext),ray.tmax);
      const float majorant = lp.grid.maxOpacities[linearIndex(cellID,lp.grid.dims)];

      float t = t0;

      while (1) { // Delta tracking loop

        if (majorant <= 0.f)
          break;
        
        t -= logf(1.f-random())/majorant;

        if (t >= t1) {
          break;
        }

        pos = ray.origin+ray.direction*t;
        Sample s = sampleVolume(pos);
        if (s.primID < 0)
          continue;

        if constexpr (SM==Default) {
          const range1f xfDomain = lp.transferFunc.domain;
          s.value -= xfDomain.lower;
          s.value /= xfDomain.upper-xfDomain.lower;
          xf = tex2D<float4>(lp.transferFunc.texture,s.value,.5f);
        } else if constexpr (SM==Gridlets) {
          const range1f xfDomain = lp.transferFunc.domain;
          s.value -= xfDomain.lower;
          s.value /= xfDomain.upper-xfDomain.lower;
          xf.w = tex2D<float4>(lp.transferFunc.texture,s.value,.5f).w;
          if (s.cellTag == ELEM_TAG) {
            xf.x = 1.f; xf.y = 0.f; xf.z = 0.f;
          } else if (s.cellTag == GRID_TAG) {
            const Gridlet &gridlet = lp.gridletBuffer[s.primID];
            vec3f posInLevelCoords = pos / (1<<gridlet.level);
            vec3i imin(posInLevelCoords);
            int col_index = imin.x % 2 == imin.y  % 2;
            col_index = imin.z % 2 == 0 ? col_index : !col_index;
            if (col_index==0) {
              xf.x = 1.f; xf.y = 1.f; xf.z = 1.f;
            } else {
              xf.x = 0.f; xf.y = 0.f; xf.z = 0.7f;
            }
          }
        } else if constexpr (SM==Teaser) {
          const range1f xfDomain = lp.transferFunc.domain;
          s.value -= xfDomain.lower;
          s.value /= xfDomain.upper-xfDomain.lower;
          xf.w = tex2D<float4>(lp.transferFunc.texture,s.value,.5f).w;
          const vec3f rgb1(tex2D<float4>(lp.transferFunc.texture,s.value,.5f));
          vec3f rgb2;
          if (s.cellTag == ELEM_TAG) {
            rgb2 = vec3f(1,0,0);
          } else if (s.cellTag == GRID_TAG) {
            const Gridlet &gridlet = lp.gridletBuffer[s.primID];
            vec3f posInLevelCoords = pos / (1<<gridlet.level);
            vec3i imin(posInLevelCoords);
            int col_index = imin.x % 2 == imin.y  % 2;
            col_index = imin.z % 2 == 0 ? col_index : !col_index;
            if (col_index==0) {
              rgb2.x = 1.f; rgb2.y = 1.f; rgb2.z = 1.f;
            } else {
              rgb2.x = 0.f; rgb2.y = 0.f; rgb2.z = 0.7f;
            }
          }

          const vec2f cpOne(.4f,.6f);
          const vec2f cpTwo(.6f,.4f);

          const vec2i viewportSize = owl::getLaunchDims();
          const vec2i pixelIndex = owl::getLaunchIndex();
          const vec2f pixelUV(pixelIndex.x/float(viewportSize.x),
                              pixelIndex.y/float(viewportSize.y));
       
          const float A  = cpTwo.x-cpOne.x;
          const float B  = cpTwo.y-cpOne.y;
          const float C1 = A*cpOne.x+B*cpOne.y;
          const float C2 = A*cpTwo.x+B*cpTwo.y;
          const float C  = A*pixelUV.x+B*pixelUV.y;
          if (C <= C1) {
            xf.x = rgb1.x; xf.y = rgb1.y; xf.z = rgb1.z;
          } else if (C >= C2) {
            xf.x = rgb2.x; xf.y = rgb2.y; xf.z = rgb2.z;
          } else {
            xf.x = (rgb1.x * (C2-C) + rgb2.x * (C-C1))/(C2-C1);
            xf.y = (rgb1.y * (C2-C) + rgb2.y * (C-C1))/(C2-C1);
            xf.z = (rgb1.z * (C2-C) + rgb2.z * (C-C1))/(C2-C1);
          }
        }

        float u = random();

        // if (s.cellTag == ELEM_TAG) {
        //   vec3f color = randomColor((unsigned)s.primID);
        //   xf.x = color.x; xf.y = color.y; xf.z = color.z;
        // } else if (s.cellTag == GRID_TAG) {
        //   xf.x = xf.y = xf.z = .8f;
        // }
        // xf.w *= lp.transferFunc.opacityScale;
        // vec3f N = normalize(s.gradient);
        // N += 1.f;
        // N /= 2.f;
        // xf.x = N.x; xf.y = N.y; xf.z = N.z;
        float sigmaT = xf.w;
        float sigmaA = sigmaT/2.f;
        /*if (u < sigmaA/sigmaT) {
          Le = getLe(s);
          Tr = 0.f;
          type = Emission;
          return;
        } else */if (sigmaT >= u * majorant) {
          Tr = 0.f;
          type = Scattering;
          return;
        }
      }

#if 1
      int axis = arg_min(tnext);
      tnext[axis] += dist[axis];
      cellID[axis] += step[axis];
      if (cellID[axis]==stop[axis]) {
        type = Boundary;
        Tr = 1.f;
        break;
      }
#else
      const float t_closest = reduce_min(tnext);
      if (tnext.x == t_closest) {
        tnext.x += dist.x;
        cellID.x += step.x;
        if (cellID.x==stop.x) {
          type = Boundary;
          Tr = 1.f;
          break;
        }
      }
      if (tnext.y == t_closest) {
        tnext.y += dist.y;
        cellID.y += step.y;
        if (cellID.y==stop.y) {
          type = Boundary;
          Tr = 1.f;
          break;
        }
      }
      if (tnext.z == t_closest) {
        tnext.z += dist.z;
        cellID.z += step.z;
        if (cellID.z==stop.z) {
          type = Boundary;
          Tr = 1.f;
          break;
        }
      }
#endif
      t0 = t1;
    }
  }

  // ------------------------------------------------------------------
  // Lambertian BRDF
  // ------------------------------------------------------------------

  __device__ inline
  vec3f cosine_sample_hemisphere(float u1, float u2)
  {
    float r     = sqrtf(u1);
    float theta = 2.f*M_PI * u2;
    float x     = r * cosf(theta);
    float y     = r * sinf(theta);
    float z     = sqrtf(1.f-u1);
    return {x,y,z};
  }

  __device__ inline
  void lambertianSample(const vec3f &N, const vec3f &wo, vec3f &wi, float &pdf, Random &random)
  {
    vec3f u, v, w = N;
    make_orthonormal_basis(u, v, w);
    vec3f sp = cosine_sample_hemisphere(random(), random());
    wi = normalize(vec3f(sp.x*u+sp.y*v+sp.z*w));
    pdf = dot(N,wi) * (1.f/M_PI);
  }

  // ------------------------------------------------------------------
  // HG phase function
  // ------------------------------------------------------------------

  __device__ inline
  float henyeyGreensteinTr(const vec3f &wo, const vec3f &wi, float g)
  {
    const float cost = dot(wo, wi);
    const float denom = 1.f + g*g + 2.f*g*cost;
    return (1.f/(4.f*M_PI)) * (1.f-g*g) / (denom*sqrtf(denom));
  }

  __device__ inline
  float henyeyGreensteinSample(const vec3f &wo, vec3f &wi, float &pdf, float g, Random &random)
  {
    const bool gIsNotZero = fabsf(g) >= FLT_EPSILON;

    const float u1 = random();
    const float u2 = random();

    const float a = gIsNotZero ? (1.f-g*g) / (1.f-g + 2.f*g*u1) : 0.f;
    const float cost = gIsNotZero ? (1.f + g*g - a*a) / (2.f*g) : 1.f - 2.f*u1;
    const float sint = sqrtf(fmaxf(0.f, 1.f-cost*cost));
    const float phi = 2.f*M_PI*u2;

    vec3f u, v, w = wo;
    make_orthonormal_basis(u, v, w);

    wi = sint * cosf(phi) * u + sint * sinf(phi) * v + cost * -w;
    pdf = 1.f;

    return henyeyGreensteinTr(-w, wi, g);
  }

  template <ShadeMode SM>
  __device__ inline void renderFrame_Impl()
  {
    auto& lp = optixLaunchParams;

    vec4f bgColor = vec4f(backGroundColor(),1.f);
    const int spp = lp.render.spp;
    const vec2i pixelIndex = owl::getLaunchIndex();
    int pixelID = pixelIndex.x + owl::getLaunchDims().x*pixelIndex.y;
    Random random(pixelID,lp.accumID);
    uint64_t clock_begin = clock64();
    vec4f accumColor = 0.f;


    for (int sampleID=0;sampleID<spp;sampleID++) {
      vec4f color = 0.f;

      float rx = random();
      float ry = random();
      vec2f screen = vec2f(pixelIndex)+vec2f(rx,ry);
      if (lp.subImage.active) {
        const vec2f size(owl::getLaunchDims());
        screen /= size;
        screen = (vec2f(1.f)-screen)*subImageUV().lower
                            + screen*subImageUV().upper;
        screen *= size;
      }
      Ray ray = generateRay(screen);
      vec3f throughput = 1.f;
      unsigned bounce = 0;

      float t0 = 1e30f, t1 = -1e30f;

      if (intersect(ray,lp.modelBounds,t0,t1)) {
        for (int i=0; i<CLIP_PLANES_MAX; ++i) {
          const bool clipPlaneEnabled = lp.clipPlanes[i].enabled;
          Plane plane{lp.clipPlanes[i].N,lp.clipPlanes[i].d};
          bool backFace=false;
          float plane_t = FLT_MAX;

          if (clipPlaneEnabled) {
            plane_t = intersect(ray,plane,backFace);
            if (plane_t > t0 && !backFace) t0 = max(t0,plane_t);
            if (plane_t < t1 &&  backFace) t1 = min(plane_t,t1);
          }
        }

        ray.origin += ray.direction * t0;
        t1 -= t0;

        while (1) { // pathtracing loop
          vec3f Le; // emission
          float Tr; // transmittance
          CollisionType ctype;
          ray.tmax = t1;

          MeshPRD meshPRD{-1,-1.f,{0.f},{0.f}};
          if (lp.meshBVH) {
            owl::traceRay(lp.meshBVH,ray,meshPRD,
                          OPTIX_RAY_FLAG_DISABLE_ANYHIT);
            if (meshPRD.primID >= 0) {
              ray.tmax = min(ray.tmax,meshPRD.t_hit);
            }
          }

          vec3f pos;
          vec4f xf = 0.f; // albedo and extinction coefficient
          sampleInteraction<SM>(ray,ctype,pos,Tr,Le,xf,random);

          // left the volume?
          if (ctype==Boundary && meshPRD.primID < 0)
            break;

          // max path lenght exceeded?
          if (bounce++ >= 1024/*lp.maxBounces*/) {
            throughput = 0.f;
            break;
          }

          vec3f albedo;
          if (ctype==Boundary && meshPRD.primID >= 0) {
            constexpr float eps = 1.f;// finest cell size
            pos = ray.origin+ray.direction*meshPRD.t_hit+normalize(ray.direction)*eps;
            albedo = meshPRD.kd;
          } else {
            albedo = vec3f(xf);
          }
          throughput *= albedo;

          // russian roulette absorption
          float P = reduce_max(throughput);
          if (P < .2f/*lp.rouletteProb*/) {
            if (random() > P) {
              throughput = 0.f;
              break;
            }
            throughput /= P;
          }

          throughput += Le;

          // Sample BRDF or phase function
          vec3f scatterDir;
          float pdf;
          if (ctype==Boundary && meshPRD.primID >= 0) {
            lambertianSample(meshPRD.Ng,-ray.direction,scatterDir,pdf,random);
            throughput *= fmaxf(0.f,dot(scatterDir,meshPRD.Ng));
            pos += 16.f*normalize(scatterDir);
          } else {
            float g = 0.f; // isotropic
            henyeyGreensteinSample(-ray.direction,scatterDir,pdf,g,random);
          }
          ray.origin    = pos;
          ray.direction = scatterDir;

          intersect(ray,lp.modelBounds,t0,t1);

          for (int i=0; i<CLIP_PLANES_MAX; ++i) {
            const bool clipPlaneEnabled = lp.clipPlanes[i].enabled;
            Plane plane{lp.clipPlanes[i].N,lp.clipPlanes[i].d};
            if (clipPlaneEnabled) {
              bool backFace=false;
              float plane_t = FLT_MAX;
              plane_t = intersect(ray,plane,backFace);
              if (plane_t > t0 && !backFace) t0 = max(t0,plane_t);
              if (plane_t < t1 &&  backFace) t1 = min(plane_t,t1);
            }
          }
        }
      }

      color = 1.f;//over(color,bgColor);
      color *= vec4f(throughput.x,throughput.y,throughput.z,1.f);
      color = bounce ? color : bgColor;
      accumColor += color;
    }


    uint64_t clock_end = clock64();
    if (lp.render.heatMapEnabled > 0.f) {
      float t = (clock_end-clock_begin)*(lp.render.heatMapScale/spp);
      accumColor = over(vec4f(heatMap(t),.5f),accumColor);
    }

    if (lp.accumID > 0)
      accumColor += vec4f(lp.accumBuffer[pixelID]);
    lp.accumBuffer[pixelID] = accumColor;
    accumColor *= (1.f/(lp.accumID+1));

    bool crossHairs = (debug() && (owl::getLaunchIndex().x == owl::getLaunchDims().x/2
                                || owl::getLaunchIndex().y == owl::getLaunchDims().y/2));

    const box2i si = subImageSelectionWin();
    bool subImageSel = lp.subImage.selecting
          && (((pixelIndex.x==si.lower.x || pixelIndex.x==si.upper.x) && (pixelIndex.y >= si.lower.y && pixelIndex.y <= si.upper.y))
           || ((pixelIndex.y==si.lower.y || pixelIndex.y==si.upper.y) && (pixelIndex.x >= si.lower.x && pixelIndex.x <= si.upper.x)));

    if (crossHairs || subImageSel) accumColor = vec4f(1.f) - accumColor;

    lp.fbPointer[pixelID] = make_rgba(accumColor*(1.f/spp));
  }

  OPTIX_RAYGEN_PROGRAM(renderFrame)()
  {
    auto& lp = optixLaunchParams;

    if (lp.shadeMode==1)
      renderFrame_Impl<Gridlets>();
    else if (lp.shadeMode==2)
      renderFrame_Impl<Teaser>();
    else
      renderFrame_Impl<Default>();
  }
} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

