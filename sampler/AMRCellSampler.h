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

#include "model/AMRCellModel.h"
#include "Sampler.h"

namespace exa {

  struct AMRCellGeom {
    AMRCell *amrCellBuffer;
    float   *scalarBuffer;
  };

  struct AMRCellSampler : Sampler
  {
    typedef std::shared_ptr<AMRCellSampler> SP;

    // Launch params associated with sampler
    struct LP {
      OptixTraversableHandle sampleBVH;
#ifdef EXA_STITCH_MIRROR_EXAJET
      affine3f  mirrorInvTransform;
#endif
    };

    bool build(OWLContext owl, Model::SP model);

    void computeMaxOpacities(OWLContext owl, OWLBuffer colorMap, range1f xfRange);

    std::vector<OWLVarDecl> getLPVariables();

    void setLPs(OWLParams lp);

    std::string className() { return "AMRCellSampler"; }

    AMRCellModel::SP model = nullptr;

  private:
    // owl
    OWLModule   module;
    OWLGeomType geomType;
    OWLGroup    blas;
    OWLGroup    tlas;
  public: // for grid
    OWLBuffer   cellBuffer;
    OWLBuffer   scalarBuffer;
  private:

  };

#ifdef __CUDA_ARCH__
  inline __device__
  Sample sample(const AMRCellSampler::LP &lp,
                const SpatialDomain &domain,
                const vec3f pos)
  {
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
#endif

} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

