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
#include <Gridlet.h>
#include <Plane.h>
#include <UElems.h>
#include "../ExaStitchSampler.h"

namespace exa {

  // ------------------------------------------------------------------
  // Gridlet user geometry
  // ------------------------------------------------------------------

  OPTIX_BOUNDS_PROGRAM(GridletGeomBounds)(const void* geomData,
                                          box3f& result,
                                          int leafID)
  {
    const GridletGeom &self = *(const GridletGeom *)geomData;

    if (self.gridletMaxOpacities[leafID] == 0.f) {
      result.lower = vec3f(+1e30f);
      result.upper = vec3f(-1e30f);
    } else {
      const Gridlet &gridlet = self.gridletBuffer[leafID];
      result = gridlet.getBounds();
    }
  }

  OPTIX_INTERSECT_PROGRAM(GridletGeomIsect)()
  {
    const GridletGeom &self = owl::getProgramData<GridletGeom>();
    int primID = optixGetPrimitiveIndex();
    vec3f pos = optixGetObjectRayOrigin();

    const Gridlet &gridlet = self.gridletBuffer[primID];
    const box3f &bounds = gridlet.getBounds();

    float value = 0.f;
    int cellID = -1;
    if (intersectGridlet(value,cellID,pos,gridlet,self.gridletScalarBuffer)) {
      if (optixReportIntersection(0.f,0)) {
        Sample& sample = owl::getPRD<Sample>();
        sample.value = value;
        sample.primID = primID;
        sample.cellID = cellID;
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

    if (self.maxOpacities[leafID] == 0.f) {
      result.lower = vec3f(+1e30f);
      result.upper = vec3f(-1e30f);
    } else {
    result = box3f();
      for (int i=0; i<8; ++i) {
        int idx = self.indexBuffer[leafID*8+i];
        if (idx < 0) break;
        vec3f v(self.vertexBuffer[idx]);
        result.extend(v);
        // printf("%i: %f,%f,%f\n",idx,v.x,v.y,v.z);
      }
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

} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0
