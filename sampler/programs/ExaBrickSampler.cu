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

#include <common.h>
#include <LaunchParams.h> // TODO: still depends on renderer's laucnh params

namespace exa {

  extern "C" __constant__ LaunchParams optixLaunchParams;

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
    if (!boxTest(ray,bounds,t0,t1))
      return;

    if (optixReportIntersection(t0, 0)) {
      DomainPRD& prd = owl::getPRD<DomainPRD>();
      prd.t0 = t0;
      prd.t1 = t1;
      prd.domainID = leafID;
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
      const int *childList  = &optixLaunchParams.sampler.ebs.abrLeafListBuffer[abr.leafListBegin];
      const int  childCount = abr.leafListSize;
      for (int childID=0;childID<childCount;childID++) {
        const int brickID = childList[childID];
        addBasisFunctions(optixLaunchParams.sampler.ebs,sumWeightedValues,sumWeights,brickID,ray.origin);
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
      const ExaBrick &brick = self.brickBuffer[leafID];
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
    const ExaBrick &brick = self.brickBuffer[leafID];
    const box3f bounds = brick.getDomain();

    if (!bounds.contains(ray.origin))
      return;

    auto& sample = owl::getPRD<BasisPRD>();
    addBasisFunctions(optixLaunchParams.sampler.ebs,sample.sumWeightedValues,sample.sumWeights,leafID,ray.origin);
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
      const ExaBrick &brick = self.brickBuffer[leafID];
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
    const ExaBrick &brick = self.brickBuffer[leafID];
    const box3f bounds = brick.getBounds(); // use strict domain here

    float t0 = ray.tmin, t1 = ray.tmax;
    if (!boxTest(ray,bounds,t0,t1))
      return;

    if (optixReportIntersection(t0, 0)) {
      DomainPRD& prd = owl::getPRD<DomainPRD>();
      prd.t0 = t0;
      prd.t1 = t1;
      prd.domainID = leafID;
    }
  }

} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

