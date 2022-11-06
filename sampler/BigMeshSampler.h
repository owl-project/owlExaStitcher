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

#if HAVE_BIGMESH
#include <sampler/BigMeshSampler.h>
#endif
#include "../model/BigMeshModel.h"
#include "Sampler.h"

namespace exa {

  struct BigMeshSampler : Sampler
  {
    typedef std::shared_ptr<BigMeshSampler> SP;

#if HAVE_BIGMESH
    typedef fun::BigMeshSampler::LP LP;
#else
    struct LP {};
#endif

    bool build(OWLContext owl, Model::SP model);

    void computeMaxOpacities(OWLContext owl, OWLBuffer colorMap, range1f xfRange);

    std::vector<OWLVarDecl> getLPVariables();

    void setLPs(OWLParams lp);

    std::string className() { return "BigMeshSampler"; }

    BigMeshModel::SP bmModel = nullptr;

#if HAVE_BIGMESH
    fun::BigMeshSampler::SP bmSampler = nullptr;
#endif
  };

#ifdef __CUDA_ARCH__
  inline __device__
  Sample sample(const BigMeshSampler::LP &lp,
                const SpatialDomain &domain,
                const vec3f pos)
  {
#ifdef HAVE_BIGMESH
    // can't just do that b/c umesh only has one raytype:
    //fun::BigMeshSampler::MarchState marchState{};
    //float ignore=0.f;
    //float value = fun::sample(lp,marchState,ignore,pos);
    
    const owl::RayT<0,2> sampleRay(pos,vec3f(1e-5f,1e-5f,1e-5f),0.f,1e-5f);
    fun::SampleResult samplePRD;
    // trace an epsilon-ray that will find all bigmesh-multileaves
    owl::traceRay(lp.sampleBVH,sampleRay,samplePRD,
                  OPTIX_RAY_FLAG_DISABLE_ANYHIT/*|
                  OPTIX_RAY_FLAG_DISABLE_CLOSESTHIT
                  |OPTIX_RAY_FLAG_TERMINATE_ON_FIRST_HIT*/
                  );
    float value = samplePRD.value;

    if (value<-1e19f)
      return {-1,-1,value};
    else
      return {0,-1,value};
#endif
  }
#endif
} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

