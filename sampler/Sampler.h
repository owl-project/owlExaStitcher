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

#include <memory>
#include <string>
#include <vector>
#include <owl/owl.h>
#include "model/Model.h"
#include "common.h"

namespace exa {

  struct Sampler : std::enable_shared_from_this<Sampler>
  {
    typedef std::shared_ptr<Sampler> SP;

    virtual ~Sampler();

    template <typename T>
    inline std::shared_ptr<T> as()
    {
      if (!this) return {};
      return std::dynamic_pointer_cast<T>(shared_from_this());
    }

    virtual bool build(OWLContext owl, Model::SP model);

    virtual void computeMaxOpacities(OWLContext owl, OWLBuffer colorMap, range1f xfRange);
    
    /*! return vector of the variables that this sampler has to set on
      the LPs */
    virtual std::vector<OWLVarDecl> getLPVariables() = 0;

    /*! apply the current launch parameters */
    virtual void setLPs(OWLParams lp) = 0;

    /*! simple reflection, used to identify program objects;
        set to the exact (case sensitive) name of the subclass */
    virtual std::string className() = 0;

    /*! the accel used for traversal/space skipping;
      upon successful initGPU, *either one* of these should
      not be NULL! */
    struct {
      OWLGroup bvh{ 0 };
      Grid::SP grid{ 0 };
      KDTree::SP kdtree{ 0 };
    } majorantAccel;

    /*! majorants/max opacities; these are set by the model classes,
      e.g., when initGPU is called */
    OWLBuffer maxOpacities{ 0 };

  };

#ifdef __CUDA_ARCH__
  typedef owl::RayT<SAMPLING_RAY_TYPE,2> SamplingRay;

  struct Sample {
    int primID;
    int cellID;
    float value;
  };

  struct SpatialDomain
  {
    /* range in ray space */
    float t0, t1;
    /* ID of the domain; not always set to sth. useful */
    int domainID;
  };

  template <typename DeviceSampler>
  inline __device__
  Sample testSample(DeviceSampler /*for tag dispatch*/, const vec3f pos, int primID)
  {
    return {-1,-1,0.f};
  }

  /* call it PRD when used as taht */
  typedef SpatialDomain DomainPRD;

  /* shared PRD for samplers that use basis interpolation */
  struct BasisPRD {
    float sumWeightedValues = 0.f;
    float sumWeights = 0.f;
  };

  template <int RT=0, int NRT=1>
  inline __device__
  bool boxTest(const RayT<RT,NRT> &ray,
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

#endif
} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

