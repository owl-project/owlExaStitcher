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
#include "../QuickClustersSampler.h"

namespace exa {

  // ------------------------------------------------------------------
  // Quick Clusters user geometry
  // ------------------------------------------------------------------

  OPTIX_BOUNDS_PROGRAM(QCLeafGeomBounds)(
    const void* geomData, box3f& result, int primID
  ) {
    const QCLeafGeom &self = *(const QCLeafGeom *)geomData;
    result = box3f();

    // #pragma unroll
    for (int k = 0; k < QCLeafGeom::ELEMENTS_PER_BOX; ++k) {
      const uint32_t leafID = (uint32_t(primID) + /*offset=*/0) * QCLeafGeom::ELEMENTS_PER_BOX + k;
      if (leafID >= self.numElements) break;

      if (self.maxOpacities[leafID] > 0.f) {
        for (int i=0; i<8; ++i) {
          int idx = self.indexBuffer[leafID*8+i];
          if (idx >= 0) {
            vec3f v(self.vertexBuffer[idx]);
            result.extend(v);
          }
        }
      }
    }

    // printf("Bounds (%f %f %f) %f %f %f - %f %f %f\n", 
    //        result.size().x,result.size().y,result.size().z,
    //        result.lower.x,result.lower.y,result.lower.z,
    //        result.upper.x,result.upper.y,result.upper.z);

    // if (empty) {
    //   result.lower = vec3f(+1e30f);
    //   result.upper = vec3f(-1e30f);
    // }
  }

  OPTIX_INTERSECT_PROGRAM(QCLeafGeomIsect)() 
  {
    const QCLeafGeom &self = owl::getProgramData<QCLeafGeom>();
    int primID = optixGetPrimitiveIndex();
    vec3f pos = optixGetObjectRayOrigin();
    float value = 0.f;

    vec4f v[8];

    // #pragma unroll
    for (int k = 0; k < QCLeafGeom::ELEMENTS_PER_BOX; ++k) {
      const uint32_t leafID = (uint32_t(primID) + /*offset=*/0) * QCLeafGeom::ELEMENTS_PER_BOX + k;
      if (leafID >= self.numElements) break;

      int numVerts = 0;
      for (int i=0; i<8; ++i) {
        int idx = self.indexBuffer[leafID*8+i];
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
        sample.primID = leafID;
        sample.cellID = -1; // not a gridlet -> -1
        return;
      }

    }
  }
} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0
