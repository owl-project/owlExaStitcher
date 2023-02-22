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

#include "ExaBrickSampler.h"

extern "C" char embedded_ExaBrickSampler[];

namespace exa {

  int ExaBrickSampler::traversalMode = EXA_STITCH_EXA_BRICK_TRAVERSAL_MODE;
  int ExaBrickSampler::samplerMode   = EXA_STITCH_EXA_BRICK_SAMPLER_MODE;

  bool ExaBrickSampler::build(OWLContext context, Model::SP mod)
  {
    module = owlModuleCreate(context, embedded_ExaBrickSampler);

    OWLVarDecl geomVars[]
    = {
       { "abrBuffer",  OWL_BUFPTR, OWL_OFFSETOF(ExaBrickGeom,abrBuffer)},
       { "brickBuffer",  OWL_BUFPTR, OWL_OFFSETOF(ExaBrickGeom,brickBuffer)},
       { "maxOpacities",  OWL_BUFPTR, OWL_OFFSETOF(ExaBrickGeom,maxOpacities)},
       { nullptr /* sentinel to mark end of list */ }
    };
  
    if (!mod)
      return false;

    model = std::dynamic_pointer_cast<ExaBrickModel>(mod);
    if (!model)
      return false;

    std::vector<ExaBrick> &bricks  = model->bricks;
    std::vector<float>    &scalars = model->scalars;
    ABRs                  &abrs    = model->abrs;
    KDTree::SP            &kdtree  = model->kdtree;
    Grid::SP              &grid    = model->grid;
    std::vector<std::vector<int>> &adjacentBricks = model->adjacentBricks;
    box3f &cellBounds = model->cellBounds;
#ifdef EXA_STITCH_MIRROR_EXAJET
    owl4x3f &mirrorTransform = model->mirrorTransform;
#endif

    if (bricks.empty())
      return false;

    // ==================================================================
    // exa brick geom
    // ==================================================================

    brickBuffer       = owlDeviceBufferCreate(context, OWL_USER_TYPE(ExaBrick), bricks.size(), bricks.data());
    scalarBuffer      = owlDeviceBufferCreate(context, OWL_FLOAT, scalars.size(), scalars.data());

    if (traversalMode == EXABRICK_ABR_TRAVERSAL || samplerMode == EXA_BRICK_SAMPLER_ABR_BVH) {
      if (abrs.value.empty())
        return false;

      abrBuffer         = owlDeviceBufferCreate(context, OWL_USER_TYPE(ABR), abrs.value.size(), abrs.value.data());
      abrLeafListBuffer = owlDeviceBufferCreate(context, OWL_INT, abrs.leafList.size(), abrs.leafList.data());
      abrMaxOpacities   = owlDeviceBufferCreate(context, OWL_FLOAT, abrs.value.size(), nullptr);

      abrGeomType = owlGeomTypeCreate(context, OWL_GEOM_USER, sizeof(ExaBrickGeom), geomVars, -1);
      owlGeomTypeSetBoundsProg   (abrGeomType, module, "ExaBrickABRGeomBounds");
      owlGeomTypeSetIntersectProg(abrGeomType, RADIANCE_RAY_TYPE, module, "ExaBrickABRGeomIsect");
      owlGeomTypeSetClosestHit   (abrGeomType, RADIANCE_RAY_TYPE, module, "ExaBrickGeomCH");
      owlGeomTypeSetIntersectProg(abrGeomType, SAMPLING_RAY_TYPE, module, "ExaBrickABRGeomSamplingIsect");
      owlGeomTypeSetClosestHit   (abrGeomType, SAMPLING_RAY_TYPE, module, "ExaBrickGeomCH");
      OWLGeom abrGeom = owlGeomCreate(context, abrGeomType);
      owlGeomSetPrimCount(abrGeom, abrs.value.size());
      owlGeomSetBuffer(abrGeom,"abrBuffer", abrBuffer);
      owlGeomSetBuffer(abrGeom,"brickBuffer", brickBuffer);
      owlGeomSetBuffer(abrGeom,"maxOpacities", abrMaxOpacities);

      owlBuildPrograms(context);

      abrBlas = owlUserGeomGroupCreate(context, 1, &abrGeom);
      owlGroupBuildAccel(abrBlas);
#ifdef EXA_STITCH_MIRROR_EXAJET
      abrTlas = owlInstanceGroupCreate(context, 2);
      owlInstanceGroupSetChild(abrTlas, 0, abrBlas);
      owlInstanceGroupSetChild(abrTlas, 1, abrBlas);
      owlInstanceGroupSetTransform(abrTlas, 1, &mirrorTransform);
#else
      abrTlas = owlInstanceGroupCreate(context, 1);
      owlInstanceGroupSetChild(abrTlas, 0, abrBlas);
#endif
      owlGroupBuildAccel(abrTlas);
    }

    if (traversalMode == EXABRICK_EXT_BVH_TRAVERSAL || samplerMode == EXA_BRICK_SAMPLER_EXT_BVH ) {
      // extended brick geometry //
      extGeomType = owlGeomTypeCreate(context, OWL_GEOM_USER, sizeof(ExaBrickGeom), geomVars, -1);
      owlGeomTypeSetBoundsProg   (extGeomType, module, "ExaBrickExtGeomBounds");
      owlGeomTypeSetIntersectProg(extGeomType, RADIANCE_RAY_TYPE, module, "ExaBrickExtGeomIsect");
      owlGeomTypeSetClosestHit   (extGeomType, RADIANCE_RAY_TYPE, module, "ExaBrickGeomCH");
      owlGeomTypeSetIntersectProg(extGeomType, SAMPLING_RAY_TYPE, module, "ExaBrickExtGeomSamplingIsect");
      owlGeomTypeSetClosestHit   (extGeomType, SAMPLING_RAY_TYPE, module, "ExaBrickGeomCH");
      OWLGeom extGeom = owlGeomCreate(context, extGeomType);
      owlGeomSetPrimCount(extGeom, bricks.size());
      owlGeomSetBuffer(extGeom,"brickBuffer", brickBuffer);
      if (!brickMaxOpacities)
        brickMaxOpacities = owlDeviceBufferCreate(context, OWL_FLOAT, bricks.size(), nullptr);
      owlGeomSetBuffer(extGeom,"maxOpacities", brickMaxOpacities);

      owlBuildPrograms(context);

      extBlas = owlUserGeomGroupCreate(context, 1, &extGeom);
      owlGroupBuildAccel(extBlas);
#ifdef EXA_STITCH_MIRROR_EXAJET
      extTlas = owlInstanceGroupCreate(context, 2);
      owlInstanceGroupSetChild(extTlas, 0, extBlas);
      owlInstanceGroupSetChild(extTlas, 1, extBlas);
      owlInstanceGroupSetTransform(extTlas, 1, &mirrorTransform);
#else
      extTlas = owlInstanceGroupCreate(context, 1);
      owlInstanceGroupSetChild(extTlas, 0, extBlas);
#endif
      owlGroupBuildAccel(extTlas);
    }

    if (traversalMode == EXABRICK_BVH_TRAVERSAL) {
      // brick geometry //
      brickGeomType = owlGeomTypeCreate(context, OWL_GEOM_USER, sizeof(ExaBrickGeom), geomVars, -1);
      owlGeomTypeSetBoundsProg   (brickGeomType, module, "ExaBrickBrickGeomBounds");
      owlGeomTypeSetIntersectProg(brickGeomType, RADIANCE_RAY_TYPE, module, "ExaBrickBrickGeomIsect");
      owlGeomTypeSetClosestHit   (brickGeomType, RADIANCE_RAY_TYPE, module, "ExaBrickGeomCH");
      OWLGeom brickGeom = owlGeomCreate(context, brickGeomType);
      owlGeomSetPrimCount(brickGeom, bricks.size());
      owlGeomSetBuffer(brickGeom,"brickBuffer", brickBuffer);
      if (!brickMaxOpacities)
        brickMaxOpacities = owlDeviceBufferCreate(context, OWL_FLOAT, bricks.size(), nullptr);
      owlGeomSetBuffer(brickGeom,"maxOpacities", brickMaxOpacities);

      owlBuildPrograms(context);

      brickBlas = owlUserGeomGroupCreate(context, 1, &brickGeom);
      owlGroupBuildAccel(brickBlas);
#ifdef EXA_STITCH_MIRROR_EXAJET
      brickTlas = owlInstanceGroupCreate(context, 2);
      owlInstanceGroupSetChild(brickTlas, 0, brickBlas);
      owlInstanceGroupSetChild(brickTlas, 1, brickBlas);
      owlInstanceGroupSetTransform(brickTlas, 1, &mirrorTransform);
#else
      brickTlas = owlInstanceGroupCreate(context, 1);
      owlInstanceGroupSetChild(brickTlas, 0, brickBlas);
#endif
      owlGroupBuildAccel(brickTlas);
    } else if (traversalMode == EXABRICK_KDTREE_TRAVERSAL) {
      // build KD tree over exabricks
      if (!kdtree)
        return false;

      if (!brickMaxOpacities)
        brickMaxOpacities = owlDeviceBufferCreate(context, OWL_FLOAT, bricks.size(), nullptr);

      kdtree->initGPU();
#ifdef EXA_STITCH_MIRROR_EXAJET
      kdtree->deviceTraversable.mirrorInvTransform = rcp((const affine3f &)mirrorTransform);
      kdtree->deviceTraversable.mirrorPlane.axis = 1;
      kdtree->deviceTraversable.mirrorPlane.offset = cellBounds.upper.y;
#endif
    }

    if (traversalMode == MC_DDA_TRAVERSAL || traversalMode == MC_BVH_TRAVERSAL) {
      // build the grid
      if (!grid || grid->dims==vec3i(0))
        return false;

      grid->build(context,shared_from_this()->as<ExaBrickSampler>(),grid->dims,cellBounds);
#ifdef EXA_STITCH_MIRROR_EXAJET
      if (traversalMode == MC_DDA_TRAVERSAL) {
        grid->deviceTraversable.mirrorInvTransform = rcp((const affine3f &)mirrorTransform);
        grid->deviceTraversable.mirrorPlane.axis = 1;
        grid->deviceTraversable.mirrorPlane.offset = cellBounds.upper.y;
      }
#endif
    }


    if (samplerMode == EXA_BRICK_SAMPLER_EXT_BVH ||
        traversalMode == EXABRICK_BVH_TRAVERSAL ||
        traversalMode == EXABRICK_EXT_BVH_TRAVERSAL ||
        traversalMode == EXABRICK_KDTREE_TRAVERSAL) {

      std::vector<range1f> hValueRanges(bricks.size());
      std::fill(hValueRanges.begin(),
                hValueRanges.end(),
                range1f{1e30f,-1e30f});

      for (size_t i=0; i<bricks.size(); ++i) {
        const ExaBrick &brick = bricks[i];
        for (int z=0; z<brick.size.z; ++z) {
          for (int y=0; y<brick.size.y; ++y) {
            for (int x=0; x<brick.size.x; ++x) {
              vec3i index3(x,y,z);
              int idx = brick.getIndexIndex(index3);
              const float value = scalars[idx];

              hValueRanges[i].lower = std::min(hValueRanges[i].lower,value);
              hValueRanges[i].upper = std::max(hValueRanges[i].upper,value);

              vec3i lower = brick.lower + index3*(1<<brick.level);
              vec3i upper = lower + (1<<brick.level);

              const vec3f halfCell = vec3f((float)(1<<brick.level))*.5f;

              box3f domain(vec3f(lower)-halfCell,vec3f(upper)+halfCell);

              for (int j=0; j<adjacentBricks[i].size(); ++j) {
                const ExaBrick &adjacentBrick = bricks[j];
                if (domain.overlaps(adjacentBrick.getDomain())) {
                  hValueRanges[j].lower = std::min(hValueRanges[j].lower,value);
                  hValueRanges[j].upper = std::max(hValueRanges[j].upper,value);
                }
              }
            }
          }
        }

        std::cout << '(' << (i+1) << '/' << bricks.size() << ")\r";
      }

      owlBufferRelease(brickValueRanges);
      brickValueRanges = owlDeviceBufferCreate(context, OWL_USER_TYPE(range1f),
                                               hValueRanges.size(),
                                               hValueRanges.data());
    }

    // Set the sampling accel
    if (samplerMode == EXA_BRICK_SAMPLER_ABR_BVH) {
      sampleBVH = abrTlas;
    } else if (samplerMode == EXA_BRICK_SAMPLER_EXT_BVH) {
      sampleBVH = extTlas;
    }

    initTraversal();

    return true;
  }

  bool ExaBrickSampler::buildOptixBVH(OWLContext owl, OWLModule module)
  {
    if (traversalMode != MC_BVH_TRAVERSAL)
      return false;

    if (!model->grid || model->grid->dims==vec3i(0))
      return false;

    if (!model->grid->buildOptixBVH(owl,module))
      return false;

#ifdef EXA_STITCH_MIRROR_EXAJET
    owlInstanceGroupSetTransform(model->grid->tlas, 1, &model->mirrorTransform);
    owlGroupBuildAccel(model->grid->tlas);
#endif

    initTraversal();

    return true;
  }

  std::vector<OWLVarDecl> ExaBrickSampler::getLPVariables()
  {
    std::vector<OWLVarDecl> vars
      = {
         { "ebs.sampleBVH", OWL_GROUP, OWL_OFFSETOF(LP,sampleBVH) },
         { "ebs.brickBuffer", OWL_BUFPTR, OWL_OFFSETOF(LP,brickBuffer) },
         { "ebs.abrBuffer", OWL_BUFPTR, OWL_OFFSETOF(LP,abrBuffer) },
         { "ebs.scalarBuffer", OWL_BUFPTR, OWL_OFFSETOF(LP,scalarBuffer) },
         { "ebs.abrLeafListBuffer", OWL_BUFPTR, OWL_OFFSETOF(LP,abrLeafListBuffer) },
#ifdef EXA_STITCH_MIRROR_EXAJET
         { "ebs.mirrorInvTransform", OWL_USER_TYPE(affine3f), OWL_OFFSETOF(LP,mirrorInvTransform)}
#endif
    };
    return vars;
  }

  void ExaBrickSampler::setLPs(OWLParams lp)
  {
    owlParamsSetGroup(lp,"ebs.sampleBVH",sampleBVH);
    owlParamsSetBuffer(lp,"ebs.brickBuffer",brickBuffer);
    owlParamsSetBuffer(lp,"ebs.abrBuffer",abrBuffer);
    owlParamsSetBuffer(lp,"ebs.scalarBuffer",scalarBuffer);
    owlParamsSetBuffer(lp,"ebs.abrLeafListBuffer",abrLeafListBuffer);
#ifdef EXA_STITCH_MIRROR_EXAJET
    affine3f mirrorInvTransform = rcp((const affine3f &)model->mirrorTransform);
    owlParamsSetRaw(lp,"ebs.mirrorInvTransform",&mirrorInvTransform);
#endif
  }

  void ExaBrickSampler::initTraversal()
  {
    Sampler::majorantAccel.bvh = NULL;
    Sampler::majorantAccel.grid = NULL;
    Sampler::majorantAccel.kdtree = NULL;

    // Set the majorant traversal accel and majorants buffer
    switch (traversalMode) {
    
      case EXABRICK_ABR_TRAVERSAL: {
        Sampler::majorantAccel.bvh = abrTlas;
        Sampler::maxOpacities = abrMaxOpacities;
        break;
      }

      case MC_DDA_TRAVERSAL: {
        Sampler::majorantAccel.grid = model->grid;
        Sampler::maxOpacities = model->grid->maxOpacities;
        break;
      }

      case MC_BVH_TRAVERSAL: {
        Sampler::majorantAccel.bvh = model->grid->tlas;
        Sampler::maxOpacities = model->grid->maxOpacities;
        break;
      }

      case EXABRICK_KDTREE_TRAVERSAL: {
        Sampler::majorantAccel.kdtree = model->kdtree;
        Sampler::maxOpacities = brickMaxOpacities;
        break;
      }

      case EXABRICK_BVH_TRAVERSAL: {
        Sampler::majorantAccel.bvh = brickTlas;
        Sampler::maxOpacities = brickMaxOpacities;
        break;
      }

      case EXABRICK_EXT_BVH_TRAVERSAL: {
        Sampler::majorantAccel.bvh = extTlas;
        Sampler::maxOpacities = brickMaxOpacities;
        break;
      }

      default: throw std::runtime_error("wrong traversal mode?!");
        break;
    }
  }

} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

