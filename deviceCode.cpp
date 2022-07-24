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
#include "Plane.h"

using owl::vec2f;
using owl::vec2i;
using owl::vec3f;
using owl::vec3i;
using owl::vec4f;
using owl::vec4i;

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
        auto lerp = [](const float val1, const float val2, const float x) {
          return (1.f-x)*val1+x*val2;
        };

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
          || numVerts==5 && intersectPyr(value,pos,v[0],v[1],v[2],v[3],v[4])
          || numVerts==6 && intersectWedge(value,pos,v[0],v[1],v[2],v[3],v[4],v[5])
          || numVerts==8 && intersectHex(value,pos,v[0],v[1],v[2],v[3],v[4],v[5],v[6],v[7]);

    if (hit && optixReportIntersection(0.f,0)) {
      VolumePRD& prd = owl::getPRD<VolumePRD>();
      prd.value = value;
      prd.primID = primID;
    }
  }

  OPTIX_CLOSEST_HIT_PROGRAM(StitchGeomCH)()
  {
  }

  struct Sample {
    int   primID;
    float value;
    vec3f gradient;
  };

  inline __device__ Sample sampleVolume(const vec3f pos)
  {
    auto& lp = optixLaunchParams;

    VolumePRD prd{-1,0.f};
    owl::Ray ray(pos,vec3f(1.f),0.f,0.f);

    owl::traceRay(lp.gridlets,ray,prd,
                  OPTIX_RAY_FLAG_DISABLE_ANYHIT);

    if (prd.primID == -1) {
      owl::traceRay(lp.boundaryCells,ray,prd,
                    OPTIX_RAY_FLAG_DISABLE_ANYHIT);
    }

    return {prd.primID,prd.value,{0.f,0.f,0.f}};
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

  inline __device__
  void sampleInteraction(Ray           &ray,
                         const float    majorant,
                         CollisionType &type,     /* scattering,emission,... */
                         float         &Tr,       /* transmission samples [0,1] */
                         vec3f         &Le,       /* emitted radiance */
                         vec4f         &xf,
                         Random        &random)
  {
    auto& lp = optixLaunchParams;

    float t = 0.f;
    vec3f pos;
    Le = 0.f;

    // Delta tracking loop
    while (1) {
      t -= logf(1.f-random())/majorant*1.f;
      pos = ray.origin+ray.direction*t;

      if (t >= ray.tmax) {
        type = Boundary;
        break;
      }

      Sample s = sampleVolume(pos);
      if (s.primID < 0)
        continue;

      float u = random();
      const range1f xfDomain = lp.transferFunc.domain;
      s.value -= xfDomain.lower;
      s.value /= xfDomain.upper-xfDomain.lower;
      xf = tex2D<float4>(lp.transferFunc.texture,s.value,.5f);
      // xf.w *= lp.transferFunc.opacityScale;
      float sigmaT = xf.w;
      float sigmaA = sigmaT/2.f;
      /*if (u < sigmaA/sigmaT) {
        Le = getLe(s);
        type = Emission;
        break;
      } else */if (sigmaT >= u * majorant) {
        type = Scattering;
        break;
      }
    }

    Tr = type==Boundary?1.f:0.f;
    ray.origin = pos;
  }

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

  OPTIX_RAYGEN_PROGRAM(renderFrame)()
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
      Ray ray = generateRay(vec2f(pixelIndex)+vec2f(rx,ry));
      vec3f throughput = 1.f;
      unsigned bounce = 0;

      float box_t0 = 1e30f, box_t1 = -1e30f;

      if (intersect(ray,lp.modelBounds,box_t0,box_t1)) {
        ray.origin += ray.direction * box_t0;
        box_t1 -= box_t0;

        while (1) { // pathtracing loop
          vec3f Le; // emission
          float Tr; // transmittance
          CollisionType ctype;
          ray.tmax = box_t1;
          float majorant = 1.f; // TODO

          vec4f xf = 0.f; // albedo and extinction coefficient
          sampleInteraction(ray,majorant,ctype,Tr,Le,xf,random);

          // left the volume?
          if (ctype==Boundary)
            break;

          // max path lenght exceeded?
          if (bounce++ >= 1024/*lp.maxBounces*/) {
            throughput = 0.f;
            break;
          }

          vec3f albedo(xf);
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

          // Sample phase function
          vec3f scatterDir;
          float pdf;
          float g = 0.f; // isotropic
          henyeyGreensteinSample(-ray.direction,scatterDir,pdf,g,random);
          ray.direction = scatterDir;

          intersect(ray,lp.modelBounds,box_t0,box_t1);
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

    bool crossHairs = (owl::getLaunchIndex().x == owl::getLaunchDims().x/2
            ||
            owl::getLaunchIndex().y == owl::getLaunchDims().y/2
            );
    if (crossHairs) accumColor = vec4f(1.f) - accumColor;

    lp.fbPointer[pixelID] = make_rgba(accumColor*(1.f/spp));
  }

} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

