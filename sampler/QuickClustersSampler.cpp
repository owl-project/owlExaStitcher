#include "QuickClustersSampler.h"

#include <owl/common/parallel/parallel_for.h>

extern "C" char embedded_ExaStitchSampler[];

namespace exa {

  bool QuickClustersSampler::build(OWLContext context, Model::SP mod)
  {
    // process host data
    if (!mod)
      return false;
    model = std::dynamic_pointer_cast<ExaStitchModel>(mod);
    if (!model)
      return false;
    const std::vector<int> &indices = model->indices;
    const std::vector<vec4f> &vertices = model->vertices;

    // ==================================================================
    // setup geometry data
    // ==================================================================

    OWLVarDecl leafGeomVars[]
    = {
       { "indexBuffer",  OWL_BUFPTR, OWL_OFFSETOF(QCLeafGeom,indexBuffer)},
       { "vertexBuffer",  OWL_BUFPTR, OWL_OFFSETOF(QCLeafGeom,vertexBuffer)},
       { "maxOpacities", OWL_BUFPTR, OWL_OFFSETOF(QCLeafGeom,maxOpacities)},
       { nullptr /* sentinel to mark end of list */ }
    };

    module = owlModuleCreate(context, embedded_ExaStitchSampler);

    if (!vertices.empty()) {
      umeshMaxOpacities = owlDeviceBufferCreate(context, OWL_FLOAT, indices.size()/8, nullptr);

      leafGeom.geomType = owlGeomTypeCreate(context, OWL_GEOM_USER, sizeof(QCLeafGeom), leafGeomVars, -1);
      owlGeomTypeSetBoundsProg   (leafGeom.geomType, module, "StitchGeomBounds");
      owlGeomTypeSetIntersectProg(leafGeom.geomType, SAMPLING_RAY_TYPE, module, "StitchGeomIsect");
      owlGeomTypeSetClosestHit   (leafGeom.geomType, SAMPLING_RAY_TYPE, module, "StitchGeomCH");

      OWLGeom geom = owlGeomCreate(context, leafGeom.geomType);
      owlGeomSetPrimCount(geom, indices.size()/8);

      vertexBuffer = owlDeviceBufferCreate(context, OWL_FLOAT4,
                                           vertices.size(),
                                           vertices.data());

      indexBuffer = owlDeviceBufferCreate(context, OWL_INT,
                                          indices.size(),
                                          indices.data());

      owlGeomSetBuffer(geom,"vertexBuffer",vertexBuffer);
      owlGeomSetBuffer(geom,"indexBuffer",indexBuffer);
      owlGeomSetBuffer(geom,"maxOpacities",umeshMaxOpacities);

      owlBuildPrograms(context);

      leafGeom.blas = owlUserGeomGroupCreate(context, 1, &geom);
      owlGroupBuildAccel(leafGeom.blas);
    }

    if (leafGeom.blas) {
#ifdef EXA_STITCH_MIRROR_EXAJET
      tlas = owlInstanceGroupCreate(context, 2);
#else
      tlas = owlInstanceGroupCreate(context, 1);
#endif
      owlInstanceGroupSetChild(tlas, 0, leafGeom.blas);
#ifdef EXA_STITCH_MIRROR_EXAJET
      owlInstanceGroupSetChild(tlas, 1, leafGeom.blas);
      owlInstanceGroupSetTransform(tlas, 1, &mirrorTransform);
#endif
    } else {
      return false;
    }

    owlGroupBuildAccel(tlas);

    // build the grid
    Grid::SP &grid = model->grid;
    if (!grid || grid->dims==vec3i(0))
      return false;

    box3f &cellBounds = model->cellBounds;
    grid->build(context,shared_from_this()->as<QuickClustersSampler>(),grid->dims,cellBounds);
#ifdef EXA_STITCH_MIRROR_EXAJET
    grid->deviceTraversable.mirrorInvTransform = rcp((const affine3f &)mirrorTransform);
    grid->deviceTraversable.mirrorPlane.axis = 1;
    grid->deviceTraversable.mirrorPlane.offset = cellBounds.upper.y;
#endif
    Sampler::majorantAccel.grid = model->grid;
    Sampler::maxOpacities = model->grid->maxOpacities;

    // All tests passed => success
    return true;
  }

  std::vector<OWLVarDecl> QuickClustersSampler::getLPVariables()
  {
    std::vector<OWLVarDecl> vars
      = {
         { "ess.sampleBVH", OWL_GROUP, OWL_OFFSETOF(LP,sampleBVH) },
#ifdef EXA_STITCH_MIRROR_EXAJET
         { "ess.mirrorInvTransform", OWL_USER_TYPE(affine3f), OWL_OFFSETOF(LP,mirrorInvTransform)}
#endif
    };
    return vars;
  }

  void QuickClustersSampler::setLPs(OWLParams lp)
  {
    owlParamsSetGroup(lp,"ess.sampleBVH",tlas);
#ifdef EXA_STITCH_MIRROR_EXAJET
    affine3f mirrorInvTransform = rcp((const affine3f &)model->mirrorTransform);
    owlParamsSetRaw(lp,"ess.mirrorInvTransform",&mirrorInvTransform);
#endif
  }
} // ::exa
