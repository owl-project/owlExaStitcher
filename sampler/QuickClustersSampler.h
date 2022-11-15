#pragma once

#include <model/ExaStitchModel.h>
#include <Gridlet.h>
#include "Sampler.h"

namespace exa {

  struct QCLeafGeom {
    int   *indexBuffer;
    vec4f *vertexBuffer;
    float *maxOpacities;
  };

  struct QuickClustersSampler : Sampler
  {
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

    std::string className() { return "ExaStitchSampler"; }

    // still using the stitcher model ...
    ExaStitchModel::SP model = nullptr;

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

}
