#pragma once

#include <model/QuickClustersModel.h>
#include <Gridlet.h>
#include "Sampler.h"

namespace exa {

  struct QCLeafGeom {
    int   *indexBuffer;
    vec4f *vertexBuffer;
    float *maxOpacities;
  };

  class QuickClustersSampler : public Sampler {
  public:
    typedef std::shared_ptr<QuickClustersSampler> SP;

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

    std::string className() { return "QuickClustersSampler"; }

    // still using the stitcher model ...
    QuickClustersModel::SP model = nullptr;

    OWLBuffer indexBuffer;
    OWLBuffer vertexBuffer;

  private:
    void sortLeafPrimitives(uint64_t* &codesSorted, uint32_t* &elementIdsSorted);

    OWLModule module;

    struct {
      OWLGeomType geomType;
      OWLGroup blas;
    } leafGeom;

    OWLGroup tlas;

    OWLBuffer umeshMaxOpacities{ 0 };
  };

#ifdef __CUDA_ARCH__
  inline __device__
  Sample sample(const QuickClustersSampler::LP &lp,
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
}
