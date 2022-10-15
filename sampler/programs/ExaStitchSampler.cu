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

} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0
