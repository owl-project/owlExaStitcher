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
       { "gridletMaxOpacities",  OWL_BUFPTR, OWL_OFFSETOF(GridletGeom,gridletMaxOpacities)},
       { nullptr /* sentinel to mark end of list */ }
    };

    OWLVarDecl stitchGeomVars[]
    = {
       { "indexBuffer",  OWL_BUFPTR, OWL_OFFSETOF(StitchGeom,indexBuffer)},
       { "vertexBuffer",  OWL_BUFPTR, OWL_OFFSETOF(StitchGeom,vertexBuffer)},
       { "maxOpacities", OWL_BUFPTR, OWL_OFFSETOF(StitchGeom,maxOpacities)},
       { nullptr /* sentinel to mark end of list */ }
    };

    if (!mod)
      return false;

    model = std::dynamic_pointer_cast<ExaStitchModel>(mod);
    if (!model)
      return false;

#ifdef EXA_STITCH_SEPARATE_INDEX_BUFFERS_PER_UELEM
    std::vector<int>     &tetIndices     = model->tetIndices;
    std::vector<int>     &pyrIndices     = model->pyrIndices;
    std::vector<int>     &wedgeIndices   = model->wedgeIndices;
    std::vector<int>     &hexIndices     = model->hexIndices;
#else
    std::vector<int>     &indices        = model->indices;
#endif
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
      gridletValueRanges = owlDeviceBufferCreate(context, OWL_USER_TYPE(range1f),
                                                 model->gridlets.size(),
                                                 nullptr);


      gridletMaxOpacities = owlDeviceBufferCreate(context, OWL_FLOAT,
                                                  model->gridlets.size(),
                                                  nullptr);

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
      owlGeomSetBuffer(geom,"gridletMaxOpacities",gridletMaxOpacities);

      owlBuildPrograms(context);

      computeGridletValueRanges(context);

      gridletGeom.blas = owlUserGeomGroupCreate(context, 1, &geom);
      owlGroupBuildAccel(gridletGeom.blas);
    }

    // ==================================================================
    // stitching geom
    // ==================================================================

#ifdef EXA_STITCH_SEPARATE_INDEX_BUFFERS_PER_UELEM
    enum Type { TET, PYR, WEDGE, HEX };
    int indexCount[] = {4,5,6,8};
    const char *boundsNames[] = {
      "StitchGeomBoundsTet",
      "StitchGeomBoundsPyr",
      "StitchGeomBoundsWedge",
      "StitchGeomBoundsHex",
    };
    const char *isectNames[] = {
      "StitchGeomIsectTet",
      "StitchGeomIsectPyr",
      "StitchGeomIsectWedge",
      "StitchGeomIsectHex",
    };

    std::vector<int> *indices[] = {
      &tetIndices,
      &pyrIndices,
      &wedgeIndices,
      &hexIndices
    };

    for (int type=0; type<4; ++type) {

      // Make sure we always _do_ build a BLAS!
      // if (indices.empty()) {
      //   indexBuffers[type] = {0};
      //   umeshMaxOpacities[type] = {0};
      //   continue;
      // }

      if (!vertexBuffer) {
        // lazily construct the first itme we find an element type
        vertexBuffer = owlDeviceBufferCreate(context, OWL_FLOAT4,
                                             vertices.size(),
                                             vertices.data());
      }

      umeshMaxOpacities[type] = owlDeviceBufferCreate(context, OWL_FLOAT,
                                                      indices[type]->size()/indexCount[type],
                                                      nullptr);

      stitchGeom[type].geomType = owlGeomTypeCreate(context,
                                                    OWL_GEOM_USER,
                                                    sizeof(StitchGeom),
                                                    stitchGeomVars, -1);
      owlGeomTypeSetBoundsProg(stitchGeom[type].geomType, module, boundsNames[type]);
      owlGeomTypeSetIntersectProg(stitchGeom[type].geomType,
                                  SAMPLING_RAY_TYPE,
                                  module,
                                  isectNames[type]);
      owlGeomTypeSetClosestHit(stitchGeom[type].geomType,
                               SAMPLING_RAY_TYPE,
                               module,
                               "StitchGeomCH");

      OWLGeom geom = owlGeomCreate(context, stitchGeom[type].geomType);
      owlGeomSetPrimCount(geom, indices[type]->size()/indexCount[type]);

      indexBuffers[type] = owlDeviceBufferCreate(context, OWL_INT,
                                                 indices[type]->size(),
                                                 indices[type]->data());

      owlGeomSetBuffer(geom,"vertexBuffer",vertexBuffer);
      owlGeomSetBuffer(geom,"indexBuffer",indexBuffers[type]);
      owlGeomSetBuffer(geom,"maxOpacities",umeshMaxOpacities[type]);

      owlBuildPrograms(context);

      stitchGeom[type].blas = owlUserGeomGroupCreate(context, 1, &geom);
      owlGroupBuildAccel(stitchGeom[type].blas);
    }
#else
    if (!vertices.empty() && !indices.empty()) {
      umeshMaxOpacities = owlDeviceBufferCreate(context, OWL_FLOAT,
                                                indices.size()/8,
                                                nullptr);

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
      owlGeomSetBuffer(geom,"maxOpacities",umeshMaxOpacities);

      owlBuildPrograms(context);

      stitchGeom.blas = owlUserGeomGroupCreate(context, 1, &geom);
      owlGroupBuildAccel(stitchGeom.blas);
    }
#endif

#ifdef EXA_STITCH_SEPARATE_INDEX_BUFFERS_PER_UELEM
    bool hasStitchGeom = stitchGeom[0].blas || stitchGeom[1].blas ||
                         stitchGeom[2].blas || stitchGeom[3].blas;
#else
    bool hasStitchGeom = stitchGeom.blas;
#endif

    if (gridletGeom.blas && hasStitchGeom) {
#ifdef EXA_STITCH_MIRROR_EXAJET
#ifdef EXA_STITCH_SEPARATE_INDEX_BUFFERS_PER_UELEM
      tlas = owlInstanceGroupCreate(context, 10);
#else
      tlas = owlInstanceGroupCreate(context, 4);
#endif
#else
#ifdef EXA_STITCH_SEPARATE_INDEX_BUFFERS_PER_UELEM
      tlas = owlInstanceGroupCreate(context, 5);
#else
      tlas = owlInstanceGroupCreate(context, 2);
#endif
#endif
      owlInstanceGroupSetChild(tlas, 0, gridletGeom.blas);
#ifdef EXA_STITCH_SEPARATE_INDEX_BUFFERS_PER_UELEM
      for (int i=0; i<4; ++i)
        owlInstanceGroupSetChild(tlas, i+1, stitchGeom[i].blas);
#else
      owlInstanceGroupSetChild(tlas, 1, stitchGeom.blas);
#endif
#ifdef EXA_STITCH_MIRROR_EXAJET
      owlInstanceGroupSetChild(tlas, 2, gridletGeom.blas);
      owlInstanceGroupSetTransform(tlas, 2, &mirrorTransform);
#ifdef EXA_STITCH_SEPARATE_INDEX_BUFFERS_PER_UELEM
      for (int i=0; i<4; ++i) {
        owlInstanceGroupSetChild(tlas, i+5, stitchGeom[i].blas);
        owlInstanceGroupSetTransform(tlas, i+5, &mirrorTransform);
      }
#else
      owlInstanceGroupSetChild(tlas, 3, stitchGeom.blas);
      owlInstanceGroupSetTransform(tlas, 3, &mirrorTransform);
#endif
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
    } else if (hasStitchGeom) {
#ifdef EXA_STITCH_MIRROR_EXAJET
#ifdef EXA_STITCH_SEPARATE_INDEX_BUFFERS_PER_UELEM
      tlas = owlInstanceGroupCreate(context, 8);
#else
      tlas = owlInstanceGroupCreate(context, 2);
#endif
#else
#ifdef EXA_STITCH_SEPARATE_INDEX_BUFFERS_PER_UELEM
      tlas = owlInstanceGroupCreate(context, 4);
#else
      tlas = owlInstanceGroupCreate(context, 1);
#endif
#endif

#ifdef EXA_STITCH_SEPARATE_INDEX_BUFFERS_PER_UELEM
      for (int i=0; i<4; ++i) {
        owlInstanceGroupSetChild(tlas, i, stitchGeom[i].blas);
#ifdef EXA_STITCH_MIRROR_EXAJET
        owlInstanceGroupSetChild(tlas, i+4, stitchGeom[i].blas);
        owlInstanceGroupSetTransform(tlas, i+4, &mirrorTransform);
#endif
      }
#else
      owlInstanceGroupSetChild(tlas, 0, stitchGeom.blas);
#ifdef EXA_STITCH_MIRROR_EXAJET
      owlInstanceGroupSetChild(tlas, 1, stitchGeom.blas);
      owlInstanceGroupSetTransform(tlas, 1, &mirrorTransform);
#endif
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

  std::vector<OWLVarDecl> ExaStitchSampler::getLPVariables()
  {
    std::vector<OWLVarDecl> vars
      = {
         { "ess.sampleBVH", OWL_GROUP, OWL_OFFSETOF(LP,sampleBVH) },
         { "ess.gridletBuffer", OWL_BUFPTR, OWL_OFFSETOF(LP,gridletBuffer) },
         { "ess.gridletScalarBuffer", OWL_BUFPTR, OWL_OFFSETOF(LP,gridletScalarBuffer) },
#ifdef EXA_STITCH_MIRROR_EXAJET
         { "ess.mirrorInvTransform", OWL_USER_TYPE(affine3f), OWL_OFFSETOF(LP,mirrorInvTransform)}
#endif
    };
    return vars;
  }

  void ExaStitchSampler::setLPs(OWLParams lp)
  {
    owlParamsSetGroup(lp,"ess.sampleBVH",tlas);
    owlParamsSetBuffer(lp,"ess.gridletBuffer",gridletBuffer);
    owlParamsSetBuffer(lp,"ess.gridletScalarBuffer",gridletScalarBuffer);
#ifdef EXA_STITCH_MIRROR_EXAJET
    affine3f mirrorInvTransform = rcp((const affine3f &)model->mirrorTransform);
    owlParamsSetRaw(lp,"ess.mirrorInvTransform",&mirrorInvTransform);
#endif
  }
} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

