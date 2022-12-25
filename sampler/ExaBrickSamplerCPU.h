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

#include <visionaray/bvh.h>
#include <visionaray/traverse.h>
#include "ExaBrickSampler.h"

namespace exa {

  struct ABRPrimitive : ABR, visionaray::primitive<unsigned>
  {
  };

  inline visionaray::hit_record<visionaray::basic_ray<float>,
                                visionaray::primitive<unsigned>>
    intersect(visionaray::basic_ray<float> r, const ABRPrimitive &abr)
  {
    using namespace visionaray;

    hit_record<basic_ray<float>, primitive<unsigned>> result;
    result.hit = false;

    owl::vec3f P(r.ori.x,r.ori.y,r.ori.z);
    if (abr.domain.contains(P)) {
      result.hit = true;
      result.t = 0.f;
      result.prim_id = abr.prim_id;
      result.isect_pos = r.ori;
    }

    return result;
  }

  inline visionaray::aabb get_bounds(const ABRPrimitive &abr)
  {
    return {{abr.domain.lower.x,abr.domain.lower.y,abr.domain.lower.z},
            {abr.domain.upper.x,abr.domain.upper.y,abr.domain.upper.z}};
  }

  inline void split_primitive(visionaray::aabb& L, visionaray::aabb& R,
                              float plane, int axis, const ABRPrimitive &abr)
  {
    VSNRAY_UNUSED(L);
    VSNRAY_UNUSED(R);
    VSNRAY_UNUSED(plane);
    VSNRAY_UNUSED(axis);
    VSNRAY_UNUSED(abr);

    // TODO: implement this to support SBVHs
  }


  // ================================================================
  // 
  // ================================================================

  class ExaBrickSamplerCPU {
  public:
    typedef std::shared_ptr<ExaBrickSamplerCPU> SP;

    bool build(ExaBrickModel::SP model);

    visionaray::index_bvh<ABRPrimitive> abrBVH;

    ExaBrickModel::SP model = nullptr;

    ExaBrick *brickBuffer = nullptr;
    float *scalarBuffer = nullptr;
  };

  inline __host__
  Sample sample(const ExaBrickSamplerCPU &sampler,
                const SpatialDomain &domain,
                vec3f pos)
  {
    visionaray::basic_ray<float> r;
    r.ori = visionaray::vec3(pos.x,pos.y,pos.z);
    r.dir = visionaray::vec3(1.f,1.f,1.f);
    r.tmin = 0.f;
    r.tmax = 0.f;
    auto abrBVH = sampler.abrBVH.ref();
    std::vector<decltype(abrBVH)> refs;
    refs.push_back(abrBVH);
    auto hr = visionaray::closest_hit(r,refs.begin(),refs.end());

    if (hr.hit) {
      const ABR &abr = sampler.model->abrs.value[hr.prim_id];
      const int *childList  = &sampler.model->abrs.leafList[abr.leafListBegin];
      const int  childCount = abr.leafListSize;
      float sumWeightedValues = 0.f;
      float sumWeights = 0.f;
      for (int childID=0;childID<childCount;childID++) {
        const int brickID = childList[childID];
        addBasisFunctions(sampler, sumWeightedValues, sumWeights, brickID, pos);
      }

      return {0,-1,sumWeights!=0.f?sumWeightedValues/sumWeights:0.f};
    } else {
      return {-1,-1,0.f};
    }
  }
} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

