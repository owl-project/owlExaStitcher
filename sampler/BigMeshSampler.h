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

#include <sampler/BigMeshSampler.h>
#include "../model/BigMeshModel.h"
#include "Sampler.h"

namespace exa {

  struct BigMeshSampler : Sampler
  {
    typedef std::shared_ptr<BigMeshSampler> SP;

    typedef fun::BigMeshSampler::LP LP;

    bool build(OWLContext owl, Model::SP model);

    void computeMaxOpacities(OWLContext owl, OWLBuffer colorMap, range1f xfRange);

    std::vector<OWLVarDecl> getLPVariables();

    void setLPs(OWLParams lp);

    std::string className() { return "BigMeshSampler"; }

    BigMeshModel::SP bmModel = nullptr;

    fun::BigMeshSampler::SP bmSampler = nullptr;
  };

#ifdef __CUDA_ARCH__
  inline __device__
  Sample sample(const BigMeshSampler::LP &lp,
                const SpatialDomain &domain,
                const vec3f pos)
  {
    fun::BigMeshSampler::MarchState marchState;
    float ignore=0.f;
    vec3f P(6569.999023,1624.000244,81564.000000);
    float value = fun::sample(lp,marchState,ignore,P);
    if (value<-1e19f)
      return {-1,-1,value};
    else
      return {0,-1,value};
  }
#endif
} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

