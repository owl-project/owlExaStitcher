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

#include "ExaStitchSampler.h"

extern "C" char embedded_ExaStitchSampler[];

namespace exa {

  bool ExaStitchSampler::build(OWLContext context, Model::SP mod)
  {
    module = owlModuleCreate(context, embedded_ExaStitchSampler);

    OWLVarDecl gridletGeomVars[]
    = {
       { "gridletBuffer",  OWL_BUFPTR, OWL_OFFSETOF(GridletGeom,gridletBuffer)},
       { "gridletScalarBuffer",  OWL_BUFPTR, OWL_OFFSETOF(GridletGeom,gridletScalarBuffer)},
       { nullptr /* sentinel to mark end of list */ }
    };

    OWLVarDecl stitchGeomVars[]
    = {
       { "indexBuffer",  OWL_BUFPTR, OWL_OFFSETOF(StitchGeom,indexBuffer)},
       { "vertexBuffer",  OWL_BUFPTR, OWL_OFFSETOF(StitchGeom,vertexBuffer)},
       { nullptr /* sentinel to mark end of list */ }
    };

    if (!mod)
      return false;

    model = std::dynamic_pointer_cast<ExaStitchModel>(mod);
    if (!model)
      return false;

    std::vector<int>     &indices        = model->indices;
    std::vector<vec4f>   &vertices       = model->vertices;
    std::vector<Gridlet> &gridlets       = model->gridlets;
    std::vector<float>   &gridletScalars = model->gridletScalars;
    Grid::SP             &grid    = model->grid;
    box3f &cellBounds = model->cellBounds;
#ifdef EXA_STITCH_MIRROR_EXAJET
    owl4x3f &mirrorTransform = model->mirrorTransform;
#endif

    // ==================================================================
    // gridlet geom
    // ==================================================================

    if (!gridlets.empty()) {
      gridletGeom.geomType = owlGeomTypeCreate(context,
                                               OWL_GEOM_USER,
                                               sizeof(GridletGeom),
                                               gridletGeomVars, -1);
      owlGeomTypeSetBoundsProg(gridletGeom.geomType, module, "GridletGeomBounds");
      owlGeomTypeSetIntersectProg(gridletGeom.geomType,
                                  SAMPLING_RAY_TYPE,
                                  module,
                                  "GridletGeomIsect");
      owlGeomTypeSetClosestHit(gridletGeom.geomType,
                               SAMPLING_RAY_TYPE,
                               module,
                               "GridletGeomCH");

      OWLGeom geom = owlGeomCreate(context, gridletGeom.geomType);
      owlGeomSetPrimCount(geom, gridlets.size());

      gridletBuffer = owlDeviceBufferCreate(context, OWL_USER_TYPE(Gridlet),
                                            gridlets.size(),
                                            gridlets.data());

      gridletScalarBuffer = owlDeviceBufferCreate(context, OWL_FLOAT,
                                                  gridletScalars.size(),
                                                  gridletScalars.data());

      owlGeomSetBuffer(geom,"gridletBuffer",gridletBuffer);
      owlGeomSetBuffer(geom,"gridletScalarBuffer",gridletScalarBuffer);

      owlBuildPrograms(context);

      gridletGeom.blas = owlUserGeomGroupCreate(context, 1, &geom);
      owlGroupBuildAccel(gridletGeom.blas);
    }

    // ==================================================================
    // stitching geom
    // ==================================================================

    if (!vertices.empty() && !indices.empty()) {
      stitchGeom.geomType = owlGeomTypeCreate(context,
                                              OWL_GEOM_USER,
                                              sizeof(StitchGeom),
                                              stitchGeomVars, -1);
      owlGeomTypeSetBoundsProg(stitchGeom.geomType, module, "StitchGeomBounds");
      owlGeomTypeSetIntersectProg(stitchGeom.geomType,
                                  SAMPLING_RAY_TYPE,
                                  module,
                                  "StitchGeomIsect");
      owlGeomTypeSetClosestHit(stitchGeom.geomType,
                               SAMPLING_RAY_TYPE,
                               module,
                               "StitchGeomCH");

      OWLGeom geom = owlGeomCreate(context, stitchGeom.geomType);
      owlGeomSetPrimCount(geom, indices.size()/8);

      vertexBuffer = owlDeviceBufferCreate(context, OWL_FLOAT4,
                                           vertices.size(),
                                           vertices.data());

      indexBuffer = owlDeviceBufferCreate(context, OWL_INT,
                                          indices.size(),
                                          indices.data());

      owlGeomSetBuffer(geom,"vertexBuffer",vertexBuffer);
      owlGeomSetBuffer(geom,"indexBuffer",indexBuffer);

      owlBuildPrograms(context);

      stitchGeom.blas = owlUserGeomGroupCreate(context, 1, &geom);
      owlGroupBuildAccel(stitchGeom.blas);
    }

    if (gridletGeom.blas && stitchGeom.blas) {
#ifdef EXA_STITCH_MIRROR_EXAJET
      tlas = owlInstanceGroupCreate(context, 4);
#else
      tlas = owlInstanceGroupCreate(context, 2);
#endif
      owlInstanceGroupSetChild(tlas, 0, gridletGeom.blas);
      owlInstanceGroupSetChild(tlas, 1, stitchGeom.blas);
#ifdef EXA_STITCH_MIRROR_EXAJET
      owlInstanceGroupSetChild(tlas, 2, gridletGeom.blas);
      owlInstanceGroupSetChild(tlas, 3, stitchGeom.blas);
      owlInstanceGroupSetTransform(tlas, 2, &mirrorTransform);
      owlInstanceGroupSetTransform(tlas, 3, &mirrorTransform);
#endif
    } else if (gridletGeom.blas) {
#ifdef EXA_STITCH_MIRROR_EXAJET
      tlas = owlInstanceGroupCreate(context, 2);
#else
      tlas = owlInstanceGroupCreate(context, 1);
#endif
      owlInstanceGroupSetChild(tlas, 0, gridletGeom.blas);
#ifdef EXA_STITCH_MIRROR_EXAJET
      owlInstanceGroupSetChild(tlas, 1, gridletGeom.blas);
      owlInstanceGroupSetTransform(tlas, 1, &mirrorTransform);
#endif
    } else if (stitchGeom.blas) {
#ifdef EXA_STITCH_MIRROR_EXAJET
      tlas = owlInstanceGroupCreate(context, 2);
#else
      tlas = owlInstanceGroupCreate(context, 1);
#endif
      owlInstanceGroupSetChild(tlas, 0, stitchGeom.blas);
#ifdef EXA_STITCH_MIRROR_EXAJET
      owlInstanceGroupSetChild(tlas, 1, stitchGeom.blas);
      owlInstanceGroupSetTransform(tlas, 1, &mirrorTransform);
#endif
    } else {
      return false;
    }

    owlGroupBuildAccel(tlas);

    // build the grid
    if (!grid || grid->dims==vec3i(0))
      return false;

    grid->build(context,shared_from_this()->as<ExaStitchSampler>(),grid->dims,cellBounds);
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

  void ExaStitchSampler::computeMaxOpacities(OWLContext owl, OWLBuffer colorMap, range1f xfRange)
  {
    if (!model->grid || model->grid->dims==vec3i(0))
      return;

    model->grid->computeMaxOpacities(owl,colorMap,xfRange);
  }

  std::vector<OWLVarDecl> ExaStitchSampler::getLPVariables()
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

  void ExaStitchSampler::setLPs(OWLParams lp)
  {
    owlParamsSetGroup(lp,"ess.sampleBVH",tlas);
#ifdef EXA_STITCH_MIRROR_EXAJET
    affine3f mirrorInvTransform = rcp((const affine3f &)model->mirrorTransform);
    owlParamsSetRaw(lp,"ess.mirrorInvTransform",&mirrorInvTransform);
#endif
  }
} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

