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
#include <deviceCode.h>

namespace exa {

  // ------------------------------------------------------------------
  // AMR cells user geometry (for eval only!)
  // ------------------------------------------------------------------

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

} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

