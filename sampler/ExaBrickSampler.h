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

#include "model/ExaBrickModel.h"
#include "Sampler.h"

namespace exa {
  
  struct ExaBrickGeom {
    ABR *abrBuffer;
    ExaBrick *brickBuffer;
    float *maxOpacities;
  };

  class ExaBrickSampler : public Sampler {
  public:
    typedef std::shared_ptr<ExaBrickSampler> SP;

    static int traversalMode;
    static int samplerMode;

    // Launch params associated with sampler
    struct LP {
      OptixTraversableHandle sampleBVH;
      ExaBrick *brickBuffer;
      ABR      *abrBuffer;
      float    *scalarBuffer;
      unsigned *numFields;
      unsigned *numScalarsPerField;
      int      *abrLeafListBuffer;
#ifdef EXA_STITCH_MIRROR_EXAJET
      affine3f  mirrorInvTransform;
#endif
    };

    bool build(OWLContext owl, Model::SP model);

    /* build OptiX BVH (currently implemented in another module) */
    bool buildOptixBVH(OWLContext owl, OWLModule module);

    void computeMaxOpacities(OWLContext owl, OWLBuffer colorMap, range1f xfRange);

    std::vector<OWLVarDecl> getLPVariables();

    void setLPs(OWLParams lp);

    std::string className() { return "ExaBrickSampler"; }

    ExaBrickModel::SP model = nullptr;

  private:
    // owl
    OWLModule   module;

    OWLGeomType abrGeomType;
    OWLGroup    abrBlas;
    OWLGroup    abrTlas;

    OWLGeomType extGeomType;
    OWLGroup    extBlas;
    OWLGroup    extTlas;

    OWLGeomType brickGeomType;
    OWLGroup    brickBlas;
    OWLGroup    brickTlas;

    OWLBuffer   abrMaxOpacities{ 0 };
    OWLBuffer   brickValueRanges{ 0 };
    OWLBuffer   brickMaxOpacities{ 0 };

  public: // for grid
    OWLBuffer   abrBuffer{ 0 };
  private:
    OWLBuffer   brickBuffer{ 0 };
    OWLBuffer   scalarBuffer{ 0 };
    OWLBuffer   abrLeafListBuffer{ 0 };

    /*! BVH used to sample the volume grid */
    OWLGroup sampleBVH{ 0 };

    // sets traversal accel on base sampler
    void initTraversal();
  };

  template <typename Sampler>
  inline __both__ float getScalar(const Sampler &self,
                                  const int brickID,
                                  const int ix, const int iy, const int iz)
  {
    const ExaBrick &brick = self.brickBuffer[brickID];
    const int idx
      = brick.begin
      + ix
      + iy * brick.size.x
      + iz * brick.size.x*brick.size.y;
    return self.scalarBuffer[idx];
  }

  template <typename Sampler>
  inline __both__ void addBasisFunctions(const Sampler &self,
                                         float &sumWeightedValues,
                                         float &sumWeights,
                                         const int brickID,
                                         const vec3f pos)
  {
    const ExaBrick &brick    = self.brickBuffer[brickID];
    const float cellWidth = (1<<brick.level);
    //const float invCellWidth = 1.f/cellWidth;
    const vec3f localPos = (pos - vec3f(brick.lower)) / vec3f(cellWidth) - vec3f(0.5f);
#if 0
    const vec3i idx_hi   = vec3i(localPos+vec3f(1.f)); // +1 to emulate 'floor()'
    const vec3i idx_lo   = idx_hi - vec3i(1);
#else
    vec3i idx_lo   = vec3i(floorf(localPos.x),floorf(localPos.y),floorf(localPos.z));
    idx_lo = max(vec3i(-1), idx_lo);
    const vec3i idx_hi   = idx_lo + vec3i(1);
#endif
    const vec3f frac     = localPos - vec3f(idx_lo);
    const vec3f neg_frac = vec3f(1.f) - frac;

    // #define INV_CELL_WIDTH invCellWidth
    #define INV_CELL_WIDTH 1.f
    if (idx_lo.z >= 0 && idx_lo.z < brick.size.z) {
      if (idx_lo.y >= 0 && idx_lo.y < brick.size.y) {
        if (idx_lo.x >= 0 && idx_lo.x < brick.size.x) {
          const float scalar = getScalar(self,brickID,idx_lo.x,idx_lo.y,idx_lo.z);
          const float weight = (neg_frac.z)*(neg_frac.y)*(neg_frac.x);
          sumWeights += weight;
          sumWeightedValues += weight*scalar;
        }
        if (idx_hi.x < brick.size.x) {
          const float scalar = getScalar(self,brickID,idx_hi.x,idx_lo.y,idx_lo.z);
          const float weight = (neg_frac.z)*(neg_frac.y)*(frac.x);
          sumWeights += weight;
          sumWeightedValues += weight*scalar;
        }
      }
      if (idx_hi.y < brick.size.y) {
        if (idx_lo.x >= 0 && idx_lo.x < brick.size.x) {
          const float scalar = getScalar(self,brickID,idx_lo.x,idx_hi.y,idx_lo.z);
          const float weight = (neg_frac.z)*(frac.y)*(neg_frac.x);
          sumWeights += weight;
          sumWeightedValues += weight*scalar;
        }
        if (idx_hi.x < brick.size.x) {
          const float scalar = getScalar(self,brickID,idx_hi.x,idx_hi.y,idx_lo.z);
          const float weight = (neg_frac.z)*(frac.y)*(frac.x);
          sumWeights += weight;
          sumWeightedValues += weight*scalar;
        }
      }
    }
      
    if (idx_hi.z < brick.size.z) {
      if (idx_lo.y >= 0 && idx_lo.y < brick.size.y) {
        if (idx_lo.x >= 0 && idx_lo.x < brick.size.x) {
          const float scalar = getScalar(self,brickID,idx_lo.x,idx_lo.y,idx_hi.z);
          const float weight = (frac.z)*(neg_frac.y)*(neg_frac.x);
          sumWeights += weight;
          sumWeightedValues += weight*scalar;
        }
        if (idx_hi.x < brick.size.x) {
          const float scalar = getScalar(self,brickID,idx_hi.x,idx_lo.y,idx_hi.z);
          const float weight = (frac.z)*(neg_frac.y)*(frac.x);
          sumWeights += weight;
          sumWeightedValues += weight*scalar;
        }
      }
      if (idx_hi.y < brick.size.y) {
        if (idx_lo.x >= 0 && idx_lo.x < brick.size.x) {
          const float scalar = getScalar(self,brickID,idx_lo.x,idx_hi.y,idx_hi.z);
          const float weight = (frac.z)*(frac.y)*(neg_frac.x);
          sumWeights += weight;
          sumWeightedValues += weight*scalar;
        }
        if (idx_hi.x < brick.size.x) {
          const float scalar = getScalar(self,brickID,idx_hi.x,idx_hi.y,idx_hi.z);
          const float weight = (frac.z)*(frac.y)*(frac.x);
          sumWeights += weight;
          sumWeightedValues += weight*scalar;
        }
      }
    }
  }

#ifdef __CUDA_ARCH__
  inline __device__
  Sample sample(const ExaBrickSampler::LP &lp,
                const SpatialDomain &domain,
                vec3f pos)
  {
#if EXA_STITCH_EXA_BRICK_TRAVERSAL_MODE == EXABRICK_ABR_TRAVERSAL && \
    EXA_STITCH_EXA_BRICK_SAMPLER_MODE == EXA_BRICK_SAMPLER_ABR_BVH
    const ABR &abr = lp.abrBuffer[domain.domainID];

#ifdef EXA_STITCH_MIRROR_EXAJET
    if (!abr.domain.contains(pos)) {
      pos = xfmPoint(lp.mirrorInvTransform,pos);
    }
#endif
    const int *childList  = &lp.abrLeafListBuffer[abr.leafListBegin];
    const int  childCount = abr.leafListSize;
    float sumWeightedValues = 0.f;
    float sumWeights = 0.f;
    for (int childID=0;childID<childCount;childID++) {
      const int brickID = childList[childID];
      addBasisFunctions(lp, sumWeightedValues, sumWeights, brickID, pos);
    }

    return {0,-1,sumWeightedValues/sumWeights};
#else
    SamplingRay ray;
    ray.origin = pos;
    ray.direction = vec3f(1.f);
    ray.tmin = 0.f;
    ray.tmax = 0.f;

    BasisPRD sample;
    owl::traceRay(lp.sampleBVH, ray, sample,
        OPTIX_RAY_FLAG_DISABLE_ANYHIT);

    if (sample.sumWeights <= 0) return {0,0,0.f};
    return {0,-1,sample.sumWeightedValues/sample.sumWeights};
#endif
  }
#endif

} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

