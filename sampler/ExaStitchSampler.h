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

#include <model/ExaStitchModel.h>
#include <Gridlet.h>
#include "Sampler.h"

namespace exa {

  struct GridletGeom {
    Gridlet *gridletBuffer;
    float   *gridletScalarBuffer;
    float   *gridletMaxOpacities;
  };

  struct StitchGeom {
    int   *indexBuffer;
    vec4f *vertexBuffer;
    float *maxOpacities;
  };

  class ExaStitchSampler : public Sampler {
  public:
    typedef std::shared_ptr<ExaStitchSampler> SP;

    // Launch params associated with sampler
    struct LP {
      OptixTraversableHandle sampleBVH;
      Gridlet *gridletBuffer;
      float *gridletScalarBuffer;
#ifdef EXA_STITCH_MIRROR_EXAJET
      affine3f  mirrorInvTransform;
#endif
    };

    bool build(OWLContext owl, Model::SP model);

    void computeMaxOpacities(OWLContext owl, OWLBuffer colorMap, range1f xfRange);

    std::vector<OWLVarDecl> getLPVariables();

    void setLPs(OWLParams lp);

    std::string className() { return "ExaStitchSampler"; }

    ExaStitchModel::SP model = nullptr;

  private:
    // owl
    OWLModule module;

    struct {
      OWLGeomType geomType;
      OWLGroup blas;
    } gridletGeom;

#ifdef EXA_STITCH_SEPARATE_INDEX_BUFFERS_PER_UELEM
    struct {
      OWLGeomType geomType;
      OWLGroup blas;
    } stitchGeom[4]; // one per type
#else
    struct {
      OWLGeomType geomType;
      OWLGroup blas;
    } stitchGeom;
#endif

    OWLGroup tlas;

  public: // for grid
#ifdef EXA_STITCH_SEPARATE_INDEX_BUFFERS_PER_UELEM
    OWLBuffer indexBuffers[4];
#else
    OWLBuffer indexBuffer{ 0 };
#endif
    OWLBuffer vertexBuffer{ 0 };
    OWLBuffer gridletBuffer{ 0 };
    OWLBuffer gridletScalarBuffer{ 0 };
  private:

    OWLBuffer gridletValueRanges{ 0 };
    OWLBuffer gridletMaxOpacities{ 0 };
#ifdef EXA_STITCH_SEPARATE_INDEX_BUFFERS_PER_UELEM
    OWLBuffer umeshMaxOpacities[4];
#else
    OWLBuffer umeshMaxOpacities{ 0 };
#endif

    void computeGridletValueRanges(OWLContext owl);
  };

#ifdef __CUDA_ARCH__
  inline __device__
  Sample testSample(const ExaStitchSampler::LP &lp,
                    const vec3f pos, int primID)
  {
    Sample sample{-1,-1,0.f};

    if (primID < 0)
      return sample;

    if (intersectGridlet(sample.value,sample.cellID,pos,
                         lp.gridletBuffer[primID],
                         lp.gridletScalarBuffer))
    {
      sample.primID = primID;
    }

    return sample;
  }

  inline __device__
  Sample sample(const ExaStitchSampler::LP &lp,
                const SpatialDomain &domain,
                const vec3f pos)
  {
    Sample prd{-1,-1,0.f};
    SamplingRay ray(pos,vec3f(1.f),0.f,0.f);

    owl::traceRay(lp.sampleBVH,ray,prd,
                  OPTIX_RAY_FLAG_DISABLE_ANYHIT);

    return prd;
  }
#endif

} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

