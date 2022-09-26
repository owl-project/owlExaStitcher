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
#include "KDTree.cuh"
#include "Plane.h"
#include "UElems.h"
#include "DDA.h"

using owl::vec2f;
using owl::vec2i;
using owl::vec3f;
using owl::vec3i;
using owl::vec4f;
using owl::vec4i;

namespace exa {

  extern "C" __constant__ LaunchParams optixLaunchParams;

  typedef owl::RayT<RADIANCE_RAY_TYPE,NUM_RAY_TYPES> Ray;
  typedef owl::RayT<SAMPLING_RAY_TYPE,NUM_RAY_TYPES> SamplingRay;

  // ------------------------------------------------------------------
  // helpers
  // ------------------------------------------------------------------

  typedef owl::common::LCG<4> Random;

#define DEBUGGING 0
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

  template <int RT=0, int NRT=1>
  inline __device__
  bool intersect(const RayT<RT,NRT> &ray,
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

  inline __device__ int getNumLights()
  {
    auto &lp = optixLaunchParams;
    int numLights = 0;
    for (int i=0; i<LIGHTS_MAX; ++i) {
      if (lp.lights[i].on) numLights++;
    }
    return numLights;
  }

  inline __device__ int uniformSampleOneLight(Random &rnd, int numLights)
  {
    int which = int(rnd() * numLights); if (which == numLights) which = 0;
    return which;
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

  inline __device__
  float firstSampleT(const range1f &rayInterval,
                     const float dt,
                     const float ils_t0)
  {
    float numSegsf = floor((rayInterval.lower - dt*ils_t0)/dt);
    float t = dt * (ils_t0 + numSegsf);
    if (t < rayInterval.lower) t += dt;
    return t;
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

  struct Sample {
    int primID;
    int cellID;
    float value;
  };

  struct SpatialPartitionPRD {
    float t0, t1;
    int   leafID;
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
    result = gridlet.getBounds();
  }

  OPTIX_INTERSECT_PROGRAM(GridletGeomIsect)()
  {
    const GridletGeom &self = owl::getProgramData<GridletGeom>();
    int primID = optixGetPrimitiveIndex();
    vec3f pos = optixGetObjectRayOrigin();

    const Gridlet &gridlet = self.gridletBuffer[primID];
    const box3f &bounds = gridlet.getBounds();

    if (bounds.contains(pos)) {
      vec3i numScalars = gridlet.dims+1;
      vec3f posInLevelCoords = (pos-bounds.lower) / (1<<gridlet.level);
      vec3i imin(posInLevelCoords);
      vec3i imax = min(imin+1,numScalars-1);

      auto linearIndex = [numScalars](const int x, const int y, const int z) {
        return z*numScalars.y*numScalars.x + y*numScalars.x + x;
      };

      const float *scalars = self.gridletScalarBuffer;
      float f1 = scalars[gridlet.begin + linearIndex(imin.x,imin.y,imin.z)];
      float f2 = scalars[gridlet.begin + linearIndex(imax.x,imin.y,imin.z)];
      float f3 = scalars[gridlet.begin + linearIndex(imin.x,imax.y,imin.z)];
      float f4 = scalars[gridlet.begin + linearIndex(imax.x,imax.y,imin.z)];

      float f5 = scalars[gridlet.begin + linearIndex(imin.x,imin.y,imax.z)];
      float f6 = scalars[gridlet.begin + linearIndex(imax.x,imin.y,imax.z)];
      float f7 = scalars[gridlet.begin + linearIndex(imin.x,imax.y,imax.z)];
      float f8 = scalars[gridlet.begin + linearIndex(imax.x,imax.y,imax.z)];

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
          Sample& sample = owl::getPRD<Sample>();
          sample.value = value;
          sample.primID = primID;
          sample.cellID = imin.z*gridlet.dims.y*gridlet.dims.x
                             + imin.y*gridlet.dims.x
                          + imin.x;
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
      Sample& sample = owl::getPRD<Sample>();
      sample.value = value;
      sample.primID = primID;
      sample.cellID = -1; // not a gridlet -> -1
    }
  }

  OPTIX_CLOSEST_HIT_PROGRAM(StitchGeomCH)()
  {
  }


  // ------------------------------------------------------------------
  // AMR cells user geometry (for eval only!)
  // ------------------------------------------------------------------

  struct BasisPRD {
    float sumWeightedValues = 0.f;
    float sumWeights = 0.f;
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
  // ExaBrick user geom (for eval!)
  // ------------------------------------------------------------------

  inline __device__ float ExaBrick_getScalar(const int brickID,
                                    const int ix, const int iy, const int iz)
  {
    auto& lp = optixLaunchParams;
    const ExaBrick &brick = lp.exaBrickBuffer[brickID];
    const int idx
      = brick.begin
      + ix
      + iy * brick.size.x
      + iz * brick.size.x*brick.size.y;
    return lp.scalarBuffer[idx];
  }
  
  inline __device__ void ExaBrick_addBasisFunctions(float &sumWeightedValues,
                                             float &sumWeights,
                                             const int brickID,
                                             const vec3f pos)
  {
    const ExaBrick &brick    = optixLaunchParams.exaBrickBuffer[brickID];
    const float cellWidth = (1<<brick.level);
    //const float invCellWidth = 1.f/cellWidth;
    const vec3f localPos = (pos - vec3f(brick.lower)) / vec3f(cellWidth) - vec3f(0.5f);
#if 0
    const vec3i idx_hi   = vec3i(localPos+vec3f(1.f)); // +1 to emulate 'floor()'
    const vec3i idx_lo   = idx_hi - vec3i(1);
#else
    vec3i idx_lo   = vec3i(floorf(localPos.x),floorf(localPos.y),floorf(localPos.z));
    idx_lo = max(vec3i(-1), idx_lo);
    const vec3i idx_hi   = idx_lo + vec3i(1);
#endif
    const vec3f frac     = localPos - vec3f(idx_lo);
    const vec3f neg_frac = vec3f(1.f) - frac;

    // #define INV_CELL_WIDTH invCellWidth
    #define INV_CELL_WIDTH 1.f
    if (idx_lo.z >= 0 && idx_lo.z < brick.size.z) {
      if (idx_lo.y >= 0 && idx_lo.y < brick.size.y) {
        if (idx_lo.x >= 0 && idx_lo.x < brick.size.x) {
          const float scalar = ExaBrick_getScalar(brickID,idx_lo.x,idx_lo.y,idx_lo.z);
          const float weight = (neg_frac.z)*(neg_frac.y)*(neg_frac.x);
          sumWeights += weight;
          sumWeightedValues += weight*scalar;
        }
        if (idx_hi.x < brick.size.x) {
          const float scalar = ExaBrick_getScalar(brickID,idx_hi.x,idx_lo.y,idx_lo.z);
          const float weight = (neg_frac.z)*(neg_frac.y)*(frac.x);
          sumWeights += weight;
          sumWeightedValues += weight*scalar;
        }
      }
      if (idx_hi.y < brick.size.y) {
        if (idx_lo.x >= 0 && idx_lo.x < brick.size.x) {
          const float scalar = ExaBrick_getScalar(brickID,idx_lo.x,idx_hi.y,idx_lo.z);
          const float weight = (neg_frac.z)*(frac.y)*(neg_frac.x);
          sumWeights += weight;
          sumWeightedValues += weight*scalar;
        }
        if (idx_hi.x < brick.size.x) {
          const float scalar = ExaBrick_getScalar(brickID,idx_hi.x,idx_hi.y,idx_lo.z);
          const float weight = (neg_frac.z)*(frac.y)*(frac.x);
          sumWeights += weight;
          sumWeightedValues += weight*scalar;
        }
      }
    }
      
    if (idx_hi.z < brick.size.z) {
      if (idx_lo.y >= 0 && idx_lo.y < brick.size.y) {
        if (idx_lo.x >= 0 && idx_lo.x < brick.size.x) {
          const float scalar = ExaBrick_getScalar(brickID,idx_lo.x,idx_lo.y,idx_hi.z);
          const float weight = (frac.z)*(neg_frac.y)*(neg_frac.x);
          sumWeights += weight;
          sumWeightedValues += weight*scalar;
        }
        if (idx_hi.x < brick.size.x) {
          const float scalar = ExaBrick_getScalar(brickID,idx_hi.x,idx_lo.y,idx_hi.z);
          const float weight = (frac.z)*(neg_frac.y)*(frac.x);
          sumWeights += weight;
          sumWeightedValues += weight*scalar;
        }
      }
      if (idx_hi.y < brick.size.y) {
        if (idx_lo.x >= 0 && idx_lo.x < brick.size.x) {
          const float scalar = ExaBrick_getScalar(brickID,idx_lo.x,idx_hi.y,idx_hi.z);
          const float weight = (frac.z)*(frac.y)*(neg_frac.x);
          sumWeights += weight;
          sumWeightedValues += weight*scalar;
        }
        if (idx_hi.x < brick.size.x) {
          const float scalar = ExaBrick_getScalar(brickID,idx_hi.x,idx_hi.y,idx_hi.z);
          const float weight = (frac.z)*(frac.y)*(frac.x);
          sumWeights += weight;
          sumWeightedValues += weight*scalar;
        }
      }
    }
  }

  OPTIX_CLOSEST_HIT_PROGRAM(ExaBrickGeomCH)()
  {
  }

  // ABR Geometry //

  OPTIX_BOUNDS_PROGRAM(ExaBrickABRGeomBounds)(const void* geomData,
                                           box3f& result,
                                           int leafID)
  {
    const ExaBrickGeom &self = *(const ExaBrickGeom *)geomData;

    if (self.maxOpacities[leafID] == 0.f) {
      result.lower = vec3f(+1e30f);
      result.upper = vec3f(-1e30f);
    } else {
      const ABR &abr = self.abrBuffer[leafID];
      result = abr.domain;
    }
  }

  // Isect program for non-zero length rays
  OPTIX_INTERSECT_PROGRAM(ExaBrickABRGeomIsect)()
  {
    const ExaBrickGeom &self = owl::getProgramData<ExaBrickGeom>();
    int leafID = optixGetPrimitiveIndex();
    owl::Ray ray(optixGetObjectRayOrigin(),
                 optixGetObjectRayDirection(),
                 optixGetRayTmin(),
                 optixGetRayTmax());
    const ABR &abr = self.abrBuffer[leafID];
    const box3f bounds = abr.domain;

    float t0 = ray.tmin, t1 = ray.tmax;
    if (!intersect(ray,bounds,t0,t1))
      return;

    if (optixReportIntersection(t0, 0)) {
      SpatialPartitionPRD& prd = owl::getPRD<SpatialPartitionPRD>();
      prd.t0 = t0;
      prd.t1 = t1;
      prd.leafID = leafID;
    }
  }

  // Isect program for sampling rays with zero length
  OPTIX_INTERSECT_PROGRAM(ExaBrickABRGeomSamplingIsect)()
  {
    const ExaBrickGeom &self = owl::getProgramData<ExaBrickGeom>();
    int leafID = optixGetPrimitiveIndex();
    owl::Ray ray(optixGetObjectRayOrigin(),
                 optixGetObjectRayDirection(),
                 optixGetRayTmin(),
                 optixGetRayTmax());
    const ABR &abr = self.abrBuffer[leafID];
    const box3f bounds = abr.domain;

    if (!bounds.contains(ray.origin))
      return;

    if (optixReportIntersection(0.f, 0)) {
      auto& sample = owl::getPRD<BasisPRD>();
      float sumWeightedValues = 0.f;
      float sumWeights = 0.f;
      const int *childList  = &optixLaunchParams.abrLeafListBuffer[abr.leafListBegin];
      const int  childCount = abr.leafListSize;
      for (int childID=0;childID<childCount;childID++) {
        const int brickID = childList[childID];
        ExaBrick_addBasisFunctions(sumWeightedValues, sumWeights, brickID, ray.origin);
      }
      sample.sumWeightedValues = sumWeightedValues;
      sample.sumWeights = sumWeights;
    }
  }

  // Extended ExaBricks //

  OPTIX_BOUNDS_PROGRAM(ExaBrickExtGeomBounds)(const void* geomData,
                                              box3f& result,
                                              int leafID)
  {
    const ExaBrickGeom &self = *(const ExaBrickGeom *)geomData;

    if (self.maxOpacities[leafID] == 0.f) {
      result.lower = vec3f(+1e30f);
      result.upper = vec3f(-1e30f);
    } else {
      const ExaBrick &brick = self.exaBrickBuffer[leafID];
      result = brick.getDomain();
    }
  }

  OPTIX_INTERSECT_PROGRAM(ExaBrickExtGeomSamplingIsect)() // sampling rays with zero length
  {
    const ExaBrickGeom &self = owl::getProgramData<ExaBrickGeom>();
    int leafID = optixGetPrimitiveIndex();
    owl::Ray ray(optixGetObjectRayOrigin(),
                 optixGetObjectRayDirection(),
                 optixGetRayTmin(),
                 optixGetRayTmax());
    const ExaBrick &brick = self.exaBrickBuffer[leafID];
    const box3f bounds = brick.getDomain();

    if (!bounds.contains(ray.origin))
      return;

    auto& sample = owl::getPRD<BasisPRD>();
    ExaBrick_addBasisFunctions(sample.sumWeightedValues, sample.sumWeights, leafID, ray.origin);
  }

  // ExaBricks //

  OPTIX_BOUNDS_PROGRAM(ExaBrickBrickGeomBounds)(const void* geomData,
                                              box3f& result,
                                              int leafID)
  {
    const ExaBrickGeom &self = *(const ExaBrickGeom *)geomData;

    if (self.maxOpacities[leafID] == 0.f) {
      result.lower = vec3f(+1e30f);
      result.upper = vec3f(-1e30f);
    } else {
      const ExaBrick &brick = self.exaBrickBuffer[leafID];
      result = brick.getBounds();
    }
  }

  OPTIX_INTERSECT_PROGRAM(ExaBrickBrickGeomIsect)() // for non-zero length rays
  {
    const ExaBrickGeom &self = owl::getProgramData<ExaBrickGeom>();
    int leafID = optixGetPrimitiveIndex();
    owl::Ray ray(optixGetObjectRayOrigin(),
                 optixGetObjectRayDirection(),
                 optixGetRayTmin(),
                 optixGetRayTmax());
    const ExaBrick &brick = self.exaBrickBuffer[leafID];
    const box3f bounds = brick.getBounds(); // use strict domain here

    float t0 = ray.tmin, t1 = ray.tmax;
    if (!intersect(ray,bounds,t0,t1))
      return;

    if (optixReportIntersection(t0, 0)) {
      SpatialPartitionPRD& prd = owl::getPRD<SpatialPartitionPRD>();
      prd.t0 = t0;
      prd.t1 = t1;
      prd.leafID = leafID;
    }
  }

  // ------------------------------------------------------------------
  // Macro Cell Grids
  // ------------------------------------------------------------------

  OPTIX_CLOSEST_HIT_PROGRAM(MacroCellGeomCH)()
  {
  }

  OPTIX_BOUNDS_PROGRAM(MacroCellGeomBounds)(const void* geomData,
                                            box3f& result,
                                            int leafID)
  {
    const MacroCellGeom &self = *(const MacroCellGeom *)geomData;

    if (self.maxOpacities[leafID] == 0.f) {
      result.lower = vec3f(+1e30f);
      result.upper = vec3f(-1e30f);
    } else {
      const vec3i index = gridIndex(leafID, self.dims);
      result.lower = self.spacing * vec3f(index) + self.origin;
      result.upper = self.spacing + result.lower;
    }

    // printf("bounds (%f,%f,%f) (%f,%f,%f)\n", 
    //        result.lower.x,result.lower.y,result.lower.z,
    //        result.upper.x,result.upper.y,result.upper.z);
  }

  OPTIX_INTERSECT_PROGRAM(MacroCellGeomIsect)() // for non-zero length rays
  {
    const MacroCellGeom &self = owl::getProgramData<MacroCellGeom>();
    int leafID = optixGetPrimitiveIndex();
    owl::Ray ray(optixGetObjectRayOrigin(),
                 optixGetObjectRayDirection(),
                 optixGetRayTmin(),
                 optixGetRayTmax());

    const auto index = gridIndex(leafID, self.dims);
    box3f bounds;
    bounds.lower = self.spacing * vec3f(index) + self.origin;
    bounds.upper = self.spacing + bounds.lower;

    float t0 = ray.tmin, t1 = ray.tmax;
    if (!intersect(ray,bounds,t0,t1))
      return;

    if (optixReportIntersection(t0, 0)) {
      SpatialPartitionPRD& prd = owl::getPRD<SpatialPartitionPRD>();
      prd.t0 = t0;
      prd.t1 = t1;
      prd.leafID = leafID;
    }
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


  // ------------------------------------------------------------------
  // Generic integration sampler for DVR and ISOs
  // ------------------------------------------------------------------

  template <typename VolumeSampler>
  struct DefaultMarcher {

    template <bool Shading=true>
    inline __device__
    vec4f integrateDVR(Ray ray, float t0, float t1, float ils_t0 = 0.f, const int numLights =0)
    {
      auto& lp = optixLaunchParams;

      // first, since we now traverse bricks and sample cells: convert ray to voxel space...
      ray.origin = xfmPoint(lp.voxelSpaceTransform,ray.origin);
      ray.direction = xfmVector(lp.voxelSpaceTransform,ray.direction);

      const float dt_scale = length(vec3f(ray.direction));
      ray.direction = normalize(vec3f(ray.direction));
      ray.tmin = ray.tmin * dt_scale;
      ray.tmax = ray.tmax * dt_scale;

      VolumeSampler sampler;
      float dt = lp.render.dt;
      vec4f color = 0.f;

      for (float t=firstSampleT({t0,t1},dt,ils_t0); t<=t1; t+=dt) {
        const vec3f pos = ray.origin+ray.direction*t;
        Sample s = sampler.sampleVolume(pos);

        if (s.primID < 0)
          continue;

        const range1f xfDomain = lp.transferFunc.domain;
        s.value -= xfDomain.lower;
        s.value /= xfDomain.upper-xfDomain.lower;
        const vec4f xf = tex2D<float4>(lp.transferFunc.texture,s.value,.5f);

        if constexpr (Shading) {
          const vec3f g = gradient(pos);
          const vec3f N = g / (length(g + 1e-4f));
          const vec3f cd = vec3f(xf);
          vec3f shaded = .1f*cd; // bit of ambient
          if (numLights==0) shaded = cd; // in that case, fully ambient
          for (int lightID=0; lightID<numLights; ++lightID) {
            const vec3f L = normalize(lp.lights[lightID].pos-pos);
            const float ld = length(lp.lights[lightID].pos-pos);
            shaded += cd * (.9f*dot(N,L) / (ld*ld) * lp.lights[lightID].intensity);
          }
          color += (1.f-color.w)*xf.w*vec4f(shaded, 1.f);
        }
        else {
          color += (1.f-color.w)*xf.w*vec4f(vec3f(xf), 1.f);
        }

        if (color.w >= 0.99f) {
          break;
        }
      }

      return color;
    }

    inline __device__
    vec4f integrateISO(const SamplingRay ray, float t0, float t1)
    {
      //
    }

    inline __device__
    vec3f gradient(const vec3f pos, VolumeSampler sampler = {})
    {
      const vec3f delta = 0.5f;

      Sample s[6] = {
        sampler.sampleVolume({pos.x+delta.x,pos.y,pos.z}),
        sampler.sampleVolume({pos.x-delta.x,pos.y,pos.z}),
        sampler.sampleVolume({pos.x,pos.y+delta.y,pos.z}),
        sampler.sampleVolume({pos.x,pos.y-delta.y,pos.z}),
        sampler.sampleVolume({pos.x,pos.y,pos.z+delta.z}),
        sampler.sampleVolume({pos.x,pos.y,pos.z-delta.z})
      };

      for (int i=0; i<6; i+=2) {
        if (s[i].primID < 0 || s[i+1].primID < 0) {
          s[i].value = {0.f};
          s[i+1].value = {0.f};
        }
      }

      return {
        s[0].value-s[1].value,
        s[2].value-s[3].value,
        s[4].value-s[5].value
      };
    }
  };

  // ------------------------------------------------------------------
  // Samplers
  // ------------------------------------------------------------------

  struct ExaStitchSampler : DefaultMarcher<ExaStitchSampler> {
    inline __device__ Sample sampleVolume(const vec3f pos)
    {
      auto& lp = optixLaunchParams;

      Sample prd{-1,-1,0.f};
      SamplingRay ray(pos,vec3f(1.f),0.f,0.f);

      owl::traceRay(lp.sampleBVH,ray,prd,
                    OPTIX_RAY_FLAG_DISABLE_ANYHIT);

      return prd;
    }
  };

  struct AMRCellSampler : DefaultMarcher<AMRCellSampler> {
    inline __device__ Sample sampleVolume(const vec3f pos)
    {
      auto& lp = optixLaunchParams;

      BasisPRD prd{0.f,0.f};
      SamplingRay ray(pos,vec3f(1.f),0.f,0.f);

      owl::traceRay(lp.sampleBVH,ray,prd,
                    OPTIX_RAY_FLAG_DISABLE_ANYHIT);

      int primID = -1;
      float value = 0.f;
      if (prd.sumWeights > 0.f) {
        primID = 0; // non-negative dummy value
        value = prd.sumWeightedValues/prd.sumWeights;
      }
      return {primID,-1/*TODO:cellID*/,value};
    }
  };

  struct ExaBrickSampler {
    // the sampler where we don't know which ABR we're in
    inline __device__ Sample sampleVolume(const vec3f pos)
    {
      SamplingRay ray;
      ray.origin = pos;
      ray.direction = vec3f(1.f);
      ray.tmin = 0.f;
      ray.tmax = 0.f;

      BasisPRD sample;
      owl::traceRay(optixLaunchParams.sampleBVH, ray, sample,
                  OPTIX_RAY_FLAG_DISABLE_ANYHIT);

      if (sample.sumWeights <= 0) return {0,0,0.f};
      return {0,-1,sample.sumWeightedValues/sample.sumWeights};
    }

    // the sampler where we know the ABR
    inline __device__ Sample sampleVolume(vec3f pos, const int leafID)
    {
      auto &lp = optixLaunchParams;
      const ABR &abr = lp.abrBuffer[leafID];

#ifdef EXA_STITCH_MIRROR_EXAJET
      if (!abr.domain.contains(pos)) {
        pos = xfmPoint(lp.mirrorInvTransform,pos);
      }
#endif
      const int *childList  = &lp.abrLeafListBuffer[abr.leafListBegin];
      const int  childCount = abr.leafListSize;
      float sumWeightedValues = 0.f;
      float sumWeights = 0.f;
      for (int childID=0;childID<childCount;childID++) {
        const int brickID = childList[childID];
        ExaBrick_addBasisFunctions(sumWeightedValues, sumWeights, brickID, pos);
      }

      return {0,-1,sumWeightedValues/sumWeights};
    }

    template <bool Shading=true>
    inline __device__
    vec4f integrateDVR(Ray ray, float t0, float t1, float ils_t0 = 0.f, const int numLights = 0);
  };

  template <typename Traversable, typename Func>
  inline __device__
  void traverse(Traversable traversable, Ray ray, const Func &func);

  template <bool Shading>
  inline __device__
  vec4f ExaBrickSampler::integrateDVR(Ray ray,
                                      float t0,
                                      float t1,
                                      float ils_t0,
                                      const int numLights)
  {
    const auto& lp = optixLaunchParams;

    vec4f pixelColor(0.f);

    // first, since we now traverse bricks and sample cells: convert ray to voxel space...
    ray.origin = xfmPoint(lp.voxelSpaceTransform,ray.origin);
    ray.direction = xfmVector(lp.voxelSpaceTransform,ray.direction);

    const float dt_scale = length(vec3f(ray.direction));
    ray.direction = normalize(vec3f(ray.direction));
    ray.tmin = ray.tmin * dt_scale;
    ray.tmax = ray.tmax * dt_scale;

    auto integrate = [=,&pixelColor](const int leafID, float t0, float t1) {
      const float global_dt = lp.render.dt;

      const ABR &abr = lp.abrBuffer[leafID];
      const float dt = global_dt * abr.finestLevelCellWidth;

      int i0 = int(ceilf((t0-dt*ils_t0) / dt));
      float t_i = (ils_t0 + i0) * dt;
      while ((t_i-dt) >= t0) t_i = t_i-dt;
      while (t_i < t0) t_i += dt;

      float t_last = t0;
      for (;true;t_i += dt)
      {
        const float t_next = min(t_i,t1);
        const float t_sample = 0.5f*(min(t1,t_next)+t_last);
        const float actual_dt = t_next-t_last;
        t_last = t_next;

        const vec3f pos = ray.origin + t_sample * ray.direction;

        const int *childList  = &lp.abrLeafListBuffer[abr.leafListBegin];
        const int  childCount = abr.leafListSize;
        float sumWeightedValues = 0.f;
        float sumWeights = 0.f;
        for (int childID=0;childID<childCount;childID++) {
          const int brickID = childList[childID];
          ExaBrick_addBasisFunctions(sumWeightedValues, sumWeights, brickID, pos);
        }

        float value = sumWeightedValues/sumWeights;
        const range1f xfDomain = lp.transferFunc.domain;
        value -= xfDomain.lower;
        value /= xfDomain.upper-xfDomain.lower;
        vec4f sample = tex2D<float4>(lp.transferFunc.texture,value,.5f);

        sample.w    = 1.f - powf(1.f-sample.w, actual_dt);
        pixelColor += (1.f-pixelColor.w)*sample.w*vec4f(vec3f(sample), 1.f);

        if (pixelColor.w >= 0.99f) break;
        if (t_next >= t1) break;
      }

      if (pixelColor.w >= 0.99f) {
        pixelColor = vec4f(vec3f(pixelColor)*pixelColor.w,1.f);
        return false; // stop iteration
      }

      return true; // go on
    };

    // Only implemented for ABR-BVH!
    traverse(lp.majorantBVH,ray,integrate);
    return pixelColor;
  }


  // ------------------------------------------------------------------
  // Spatial domain iterators
  // ------------------------------------------------------------------

  template <typename Traversable, typename Func>
  inline __device__
  void traverse(Traversable traversable, Ray ray, const Func &func)
  {
    float alreadyIntegratedDistance = ray.tmin;

    while (1) {
      SpatialPartitionPRD prd;
      prd.leafID = -1;
      prd.t0 = prd.t1 = 0.f; // doesn't matter as long as leafID==-1
      ray.tmin = alreadyIntegratedDistance;

      owl::traceRay(traversable, ray, prd, OPTIX_RAY_FLAG_DISABLE_ANYHIT);

      if (prd.leafID < 0)
        return;

      if (!func(prd.leafID,prd.t0,prd.t1))
        return;

      alreadyIntegratedDistance = prd.t1 * (1.0000001f);
    }
  }

  template <typename Func>
  inline __device__
  void traverse(const KDTreeTraversableHandle &traversable, Ray ray, const Func &func)
  {
    SpatialPartitionPRD prd;

    kd::traceRay(traversable, ray, prd, [=](const Ray &ray,
                                            SpatialPartitionPRD &prd,
                                            int primID,
                                            float tmin, float tmax,
                                            KDTreeHitRec &hitRec) {
      const ExaBrick &brick = optixLaunchParams.exaBrickBuffer[primID];
      const box3f bounds = brick.getBounds(); // use strict domain

      float t0 = 1e30f, t1 = -1e30f;
      if (intersect(ray,bounds,t0,t1)) {
        t0 = max(t0, tmin);
        t1 = min(t1, tmax);
        hitRec.hit = !func(primID,t0,t1);
      }
    });
  }

  template <typename Func>
  inline __device__
  void traverse(const GridTraversableHandle &grid, Ray ray, const Func &func)
  {
#ifdef EXA_STITCH_MIRROR_EXAJET
    if (grid.mirrorPlane.axis < 0) {
      dda3(ray,grid.traversable.dims,grid.traversable.bounds,func);
    } else {
      float d;
      if (grid.mirrorPlane.axis == 0) {
        d = (grid.mirrorPlane.offset - ray.origin.x) * rcp(ray.direction.x);
      }
      else if (grid.mirrorPlane.axis == 1) {
        d = (grid.mirrorPlane.offset - ray.origin.y) * rcp(ray.direction.y);
      }
      else {
        d = (grid.mirrorPlane.offset - ray.origin.z) * rcp(ray.direction.z);
      }

      const vec3f ori = ray.origin;
      const vec3f dir = ray.direction;
      const float tmin = ray.tmin;
      const float tmax = ray.tmax;

      unsigned i_near = signbit(ray.direction[grid.mirrorPlane.axis]);
      unsigned i_far  = 1^i_near;

      if (d <= ray.tmin) {
        if (i_far == 1) {
          ray.origin = xfmPoint(grid.mirrorInvTransform,ray.origin);
          ray.direction = xfmVector(grid.mirrorInvTransform,ray.direction);
        }
        dda3(ray,grid.traversable.dims,grid.traversable.bounds,func);
      } else if (d >= ray.tmax) {
        if (i_near == 1) {
          ray.origin = xfmPoint(grid.mirrorInvTransform,ray.origin);
          ray.direction = xfmVector(grid.mirrorInvTransform,ray.direction);
        }
        dda3(ray,grid.traversable.dims,grid.traversable.bounds,func);
      } else {
        bool goOn = false;
        auto wrapper = [&goOn,func](int leafID, float t0, float t1) {
          goOn = func(leafID,t0,t1);
          return goOn;
        };

        if (i_near == 1) {
          ray.origin = xfmPoint(grid.mirrorInvTransform,ori);
          ray.direction = xfmVector(grid.mirrorInvTransform,dir);
        }
        ray.tmin = tmin;
        ray.tmax = min(tmax,d);
        dda3(ray,grid.traversable.dims,grid.traversable.bounds,wrapper);

        if (goOn) {
          ray.origin = ori;
          ray.direction = dir;
          if (i_far == 1) {
            ray.origin = xfmPoint(grid.mirrorInvTransform,ori);
            ray.direction = xfmVector(grid.mirrorInvTransform,dir);
          }
          ray.tmin = max(tmin,d);
          ray.tmax = tmax;
          dda3(ray,grid.traversable.dims,grid.traversable.bounds,func);
        }
      }
    }
#else
    dda3(ray,grid.dims,grid.bounds,func);
#endif
  }


  // ------------------------------------------------------------------
  // Woodcock path tracer
  // ------------------------------------------------------------------

  inline __device__ vec3f getLe(const Sample s)
  {
    auto& lp = optixLaunchParams;
    vec4f xf = tex1D<float4>(lp.transferFunc.texture,s.value);
    if (s.value > .99f)
    //if (s.value < .2f)
      return {120.f,60.f,45.f};
    return 0.f;
  }

  enum CollisionType {
    Boundary, Scattering, Emission,
  };

  enum ShadeMode {
    Default,  /* ordinary mode assigning colors from the TF */
    Gridlets, /* show gridlets as checkerboard */
    Teaser,   /* paper teaser */
  };

  template <ShadeMode SM, typename Traversable, typename Sampler>
  inline __device__
  void sampleInteraction(const Traversable &traversable,
                         Ray                ray,
                         Sampler            sampler,
                         CollisionType     &type,     /* scattering,emission,... */
                         vec3f             &pos,      /* position of interaction */
                         float             &Tr,       /* transmission samples [0,1] */
                         vec3f             &Le,       /* emitted radiance */
                         vec4f             &xf,
                         Random            &random)
  {
    auto& lp = optixLaunchParams;

    // first, since we now traverse bricks and sample cells: convert ray to voxel space...
    ray.origin = xfmPoint(lp.voxelSpaceTransform,ray.origin);
    ray.direction = xfmVector(lp.voxelSpaceTransform,ray.direction);

    const float dt_scale = length(vec3f(ray.direction));
    ray.direction = normalize(vec3f(ray.direction));
    ray.tmin = ray.tmin * dt_scale;
    ray.tmax = ray.tmax * dt_scale;

    Le = 0.f;
    type = Boundary;
    Tr = 1.f;

    auto woodcockFunc = [&](const int leafID, float t0, float t1) {
      const float majorant = optixLaunchParams.maxOpacities[leafID];

      float t = t0;

      while (1) { // Delta tracking loop

        if (majorant <= 0.f)
          break;
        
        t -= logf(1.f-random())/majorant;

        if (t >= t1) {
          break;
        }

        pos = ray.origin+ray.direction*t;
#if EXA_STITCH_EXA_BRICK_TRAVERSAL_MODE == EXABRICK_ABR_TRAVERSAL && EXA_STITCH_EXA_BRICK_SAMPLER_MODE == EXA_BRICK_SAMPLER_ABR_BVH
        Sample s = sampler.sampleVolume(pos,leafID);
#else
        Sample s = sampler.sampleVolume(pos);
#endif
        if (s.primID < 0)
          continue;

        if constexpr (SM==Default) {
          const range1f xfDomain = lp.transferFunc.domain;
          s.value -= xfDomain.lower;
          s.value /= xfDomain.upper-xfDomain.lower;
          xf = tex2D<float4>(lp.transferFunc.texture,s.value,.5f);
          //xf = vec4f(randomColor(leafID),0.1f);
        } else if constexpr (SM==Gridlets) {
          const range1f xfDomain = lp.transferFunc.domain;
          s.value -= xfDomain.lower;
          s.value /= xfDomain.upper-xfDomain.lower;
          xf.w = tex2D<float4>(lp.transferFunc.texture,s.value,.5f).w;
          if (s.cellID == -1) { // uelem
            xf.x = 1.f; xf.y = 0.f; xf.z = 0.f;
          } else {
            const Gridlet &gridlet = lp.gridletBuffer[s.primID];
            vec3i imin = {
              s.cellID%gridlet.dims.x,
              s.cellID/gridlet.dims.x%gridlet.dims.y,
              s.cellID/(gridlet.dims.x*gridlet.dims.y)
            };
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
          if (s.cellID == -1) { // uelem
            rgb2 = vec3f(1,0,0);
          } else {
            const Gridlet &gridlet = lp.gridletBuffer[s.primID];
            vec3i imin = {
              s.cellID%gridlet.dims.x,
              s.cellID/gridlet.dims.x%gridlet.dims.y,
              s.cellID/(gridlet.dims.x*gridlet.dims.y)
            };
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
          return false;
        } else */if (sigmaT >= u * majorant) {
          Tr = 0.f;
          type = Scattering;
          return false;
        }
      }

      return true;
    };

    traverse(traversable,ray,woodcockFunc);

    // TODO: instead of transforming this back, keep both the world space
    // and voxel space rays around, and cmpute pos = ray.ori+t*ray.dir in
    // world space; I opted not to for the moment b/c it's a it confusing
    // with the above almbda, and which params it has access to and what
    // they mean, but eventually, we should do it this way because it's
    // way cleaner/probably faster
    pos = xfmPoint(rcp(lp.voxelSpaceTransform),pos);
  }

  template <ShadeMode SM, typename Traversable, typename Sampler>
  inline __device__
  vec3f sampleLight(const Traversable &traversable,
                    const vec3f        pos,
                    Sampler            sampler,
                    Random            &random,
                    int                numLights)
  {
    auto &lp = optixLaunchParams;

    int lightID = uniformSampleOneLight(random,numLights);

    vec3f lightDir = lp.lights[lightID].pos-pos;
    const float ld = length(xfmVector(lp.lightSpaceTransform,lightDir));
    const float ldWorld = length(lightDir);
    // if (debug()) printf("lightDir: (%f,%f,%f), ld: %f\n",lightDir.x,lightDir.y,lightDir.z,ld);
    lightDir = normalize(lightDir);

    Ray ray;
    ray.origin = pos;
    ray.direction = lightDir;
    ray.tmax = ldWorld;

    float t00 = 1e30f, t11 = -1e30f;

    intersect(ray,lp.worldSpaceBounds,t00,t11);
    for (int i=0; i<CLIP_PLANES_MAX; ++i) {
      const bool clipPlaneEnabled = lp.clipPlanes[i].enabled;
      Plane plane{lp.clipPlanes[i].N,lp.clipPlanes[i].d};
      bool backFace=false;
      float plane_t = FLT_MAX;

      if (clipPlaneEnabled) {
        plane_t = intersect(ray,plane,backFace);
        if (plane_t > t00 && !backFace) t00 = max(t00,plane_t);
        if (plane_t < t11 &&  backFace) t11 = min(plane_t,t11);
      }
    }

    ray.tmax = t11;

    MeshPRD meshPRD{-1,-1.f,{0.f},{0.f}};
    if (lp.meshBVH) {
      owl::traceRay(lp.meshBVH,ray,meshPRD,
                    OPTIX_RAY_FLAG_DISABLE_ANYHIT);
      if (meshPRD.primID >= 0) {
        ray.tmax = min(ray.tmax,meshPRD.t_hit);
      }
    }

    vec3f Le;
    CollisionType ctype;
    vec3f newPos;
    float Tr;
    vec4f xf = 0.f; // albedo and extinction coefficient (ignored)
    sampleInteraction<SM>(traversable,
                          ray,
                          sampler,
                          ctype,
                          newPos,
                          Tr,
                          Le,
                          xf,
                          random);

    if (ctype==Boundary && meshPRD.primID >= 0) {
      return max(0.f,dot(meshPRD.Ng,lightDir)) * Tr / (ld*ld) * lp.lights[lightID].intensity;
    } else {
      return Tr / (ld*ld) * lp.lights[lightID].intensity;
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

  // ------------------------------------------------------------------
  // Multi-scattering path tracing integration function
  // ------------------------------------------------------------------

  template <ShadeMode SM, typename Traversable,  typename Sampler>
  inline __device__
  vec4f pathTracingIntegrate(const Traversable &traversable,
                             Ray                ray,
                             Sampler            sampler,
                             Random            &random,
                             const vec4f        bgColor,
                             int                numLights)
  {
    auto& lp = optixLaunchParams;

    vec4f color = 0.f;
    vec3f throughput = 1.f;
    vec3f Ld = 0.f;
    unsigned bounce = 0;

    float t0 = 1e30f, t1 = -1e30f;

    if (intersect(ray,lp.worldSpaceBounds,t0,t1)) {
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
        sampleInteraction<SM>(traversable,ray,sampler,ctype,pos,Tr,Le,xf,random);

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

        if (numLights > 0) {
          Ld += throughput * sampleLight<SM>(traversable,pos,sampler,random,numLights);
        }

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

        intersect(ray,lp.worldSpaceBounds,t0,t1);

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

    if (numLights==0) { // ambient light!
      Ld = 1.f;
    }

    vec3f L = Ld * throughput;
    color = vec4f(L,1.f);
    color = bounce ? color : bgColor;

    return color;
  }

  // ------------------------------------------------------------------
  // Single-scattering/direct lighting integration function
  // ------------------------------------------------------------------

  template <ShadeMode SM, typename Traversable,  typename Sampler>
  inline __device__
  vec4f directLightingIntegrate(const Traversable &traversable,
                                Ray                ray,
                                Sampler            sampler,
                                Random            &random,
                                const vec4f        bgColor,
                                int                numLights)
  {
    auto& lp = optixLaunchParams;

    vec4f color = 0.f;
    vec3f throughput = 1.f;

    float t0 = 1e30f, t1 = -1e30f;

    if (intersect(ray,lp.worldSpaceBounds,t0,t1)) {
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
      sampleInteraction<SM>(traversable,ray,sampler,ctype,pos,Tr,Le,xf,random);

      // left the volume?
      if (ctype==Boundary && meshPRD.primID < 0) {
        color = bgColor;
      } else {
        vec3f albedo;
        if (ctype==Boundary && meshPRD.primID >= 0) {
          constexpr float eps = 1.f;// finest cell size
          pos = ray.origin+ray.direction*meshPRD.t_hit+normalize(ray.direction)*eps;
          albedo = meshPRD.kd;
        } else {
          albedo = vec3f(xf);
        }

        if (numLights > 0) {
          throughput *= albedo*sampleLight<SM>(traversable,pos,sampler,random,numLights);
        } else {
          // Just assume we have a (1,1,1) ambient light
          throughput *= albedo;
          // Surfaces, shade them with a directional headlight
          if (ctype==Boundary && meshPRD.primID >= 0) {
            throughput *= fmaxf(0.f,dot(-ray.direction,meshPRD.Ng));
          }
        }
        color = vec4f(throughput,1.f);
      }
    } else {
      color = bgColor;
    }

    return color;
  }

  // ------------------------------------------------------------------
  // Ray marching integration function
  // ------------------------------------------------------------------

  template <ShadeMode SM, typename Sampler>
  inline __device__
  vec4f rayMarchingIntegrate(Ray          ray,
                             Sampler      sampler,
                             Random      &random,
                             const vec4f  bgColor,
                             int          numLights)
  {
    auto& lp = optixLaunchParams;

    vec4f color = bgColor;
    vec3f throughput = 1.f;

    float t0 = 1e30f, t1 = -1e30f;

    if (intersect(ray,lp.worldSpaceBounds,t0,t1)) {
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


      MeshPRD meshPRD{-1,-1.f,{0.f},{0.f}};
      if (lp.meshBVH) {
        owl::traceRay(lp.meshBVH,ray,meshPRD,
                      OPTIX_RAY_FLAG_DISABLE_ANYHIT);
        if (meshPRD.primID >= 0) {
          t1 = min(t1,meshPRD.t_hit);
          color = fmaxf(0.f,dot(-ray.direction,meshPRD.Ng));
        }
      }

      ray.tmin = t0;
      ray.tmax = t1;

      color = over(sampler.integrateDVR(ray,t0,t1,random(),numLights),color);
    }

    return color;
  }

  // ------------------------------------------------------------------
  // RAYGEN
  // ------------------------------------------------------------------

  enum Integrator { PathTracer, DirectLighting, RayMarcher };

  template <Integrator I, ShadeMode SM, typename Traversable,  typename Sampler>
  __device__ inline void renderFrame_Impl(const Traversable &traversable, Sampler sampler)
  {
    auto& lp = optixLaunchParams;

    vec4f bgColor = vec4f(backGroundColor(),1.f);
    const int numLights = getNumLights();
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

      if constexpr (I==PathTracer) {
        accumColor += pathTracingIntegrate<SM>(traversable,
                                               ray,
                                               sampler,
                                               random,
                                               bgColor,
                                               numLights);
      } else if constexpr (I==DirectLighting) {
        accumColor += directLightingIntegrate<SM>(traversable,
                                                  ray,
                                                  sampler,
                                                  random,
                                                  bgColor,
                                                  numLights);
      } else if constexpr (I==RayMarcher) {
        accumColor += rayMarchingIntegrate<SM>(ray,
                                               sampler,
                                               random,
                                               bgColor,
                                               numLights);
      }

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

    bool crossHairs = DEBUGGING && (owl::getLaunchIndex().x == owl::getLaunchDims().x/2
                                 || owl::getLaunchIndex().y == owl::getLaunchDims().y/2);

    const box2i si = subImageSelectionWin();
    bool subImageSel = lp.subImage.selecting
          && (((pixelIndex.x==si.lower.x || pixelIndex.x==si.upper.x) && (pixelIndex.y >= si.lower.y && pixelIndex.y <= si.upper.y))
           || ((pixelIndex.y==si.lower.y || pixelIndex.y==si.upper.y) && (pixelIndex.x >= si.lower.x && pixelIndex.x <= si.upper.x)));

    if (crossHairs || subImageSel) accumColor = vec4f(1.f) - accumColor;

    lp.fbPointer[pixelID] = make_rgba(accumColor*(1.f/spp));
  }

  template<Integrator I, typename S, ShadeMode Shade>
  inline void __device__ renderFrame_SelectAccelType()
  {
    auto& lp = optixLaunchParams;
#if EXA_STITCH_EXA_BRICK_TRAVERSAL_MODE == MC_DDA_TRAVERSAL
    renderFrame_Impl<I,Shade>(lp.majorantGrid,S{});
#elif EXA_STITCH_EXA_BRICK_TRAVERSAL_MODE ==  EXABRICK_KDTREE_TRAVERSAL
    renderFrame_Impl<I,Shade>(lp.majorantKDTree,S{});
#else
    renderFrame_Impl<I,Shade>(lp.majorantBVH,S{});
#endif
  }

  template<Integrator I>
  inline __device__ void renderFrame_SelectSampler()
  {
    auto& lp = optixLaunchParams;

#ifdef EXA_STITCH_WITH_EXA_STITCH_SAMPLER
    if (lp.sampler == EXA_STITCH_SAMPLER) {
      if (lp.shadeMode == SHADE_MODE_DEFAULT)
        return renderFrame_SelectAccelType<I, ExaStitchSampler, Default>();
      else if (lp.shadeMode == SHADE_MODE_GRIDLETS)
        return renderFrame_SelectAccelType<I, ExaStitchSampler, Gridlets>();
      else if (lp.shadeMode == SHADE_MODE_TEASER)
        return renderFrame_SelectAccelType<I, ExaStitchSampler, Teaser>();
    }
#endif

#ifdef EXA_STITCH_WITH_EXA_BRICK_SAMPLER
    if (lp.sampler==EXA_BRICK_SAMPLER) {
      return renderFrame_SelectAccelType<I, ExaBrickSampler, Default>();
    }
#endif

#ifdef EXA_STITCH_WITH_AMR_CELL_SAMPLER
    if (lp.sampler==AMR_CELL_SAMPLER) {
      return renderFrame_SelectAccelType<I, AMRCellSampler, Default>();
    }
#endif
  }

  OPTIX_RAYGEN_PROGRAM(renderFrame)()
  {
    const vec2i pixelIndex = owl::getLaunchIndex();
    auto& lp = optixLaunchParams;
    
    if (lp.integrator==PATH_TRACING_INTEGRATOR)      renderFrame_SelectSampler<PathTracer>();
    else if (lp.integrator==DIRECT_LIGHT_INTEGRATOR) renderFrame_SelectSampler<DirectLighting>();
    else if (lp.integrator==RAY_MARCHING_INTEGRATOR) renderFrame_SelectSampler<RayMarcher>();
  }
} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

