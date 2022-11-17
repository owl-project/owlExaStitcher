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

  // struct Meshlet {
  //   uint32_t numElements;
  //   uint32_t numPyramids;
  //   uint32_t numWedges;
  //   uint32_t numHexahedra;
  //   std::vector<uint8_t> tetrahedra;
  //   std::vector<uint8_t> pyramids;
  //   std::vector<uint8_t> wedges;
  //   std::vector<uint8_t> hexahedra;
  //   std::vector<vec4f> vertices;
  // };

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
