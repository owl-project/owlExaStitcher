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

#include "AMRCellSampler.h"

extern "C" char embedded_AMRCellSampler[];

namespace exa {

  bool AMRCellSampler::build(OWLContext context, Model::SP mod)
  {
    module = owlModuleCreate(context, embedded_AMRCellSampler);

    OWLVarDecl geomVars[]
    = {
       { "amrCellBuffer",  OWL_BUFPTR, OWL_OFFSETOF(AMRCellGeom,amrCellBuffer)},
       { "scalarBuffer",  OWL_BUFPTR, OWL_OFFSETOF(AMRCellGeom,scalarBuffer)},
       { nullptr /* sentinel to mark end of list */ }
    };

    if (!mod)
      return false;

    model = std::dynamic_pointer_cast<AMRCellModel>(mod);
    if (!model)
      return false;

    std::vector<AMRCell> &cells   = model->cells;
    std::vector<float>   &scalars = model->scalars;
    Grid::SP             &grid    = model->grid;
    box3f &cellBounds = model->cellBounds;
#ifdef EXA_STITCH_MIRROR_EXAJET
    owl4x3f &mirrorTransform = model->mirrorTransform;
#endif

    if (!cells.empty()) {
      geomType = owlGeomTypeCreate(context,
                                   OWL_GEOM_USER,
                                   sizeof(AMRCellGeom),
                                   geomVars, -1);
      owlGeomTypeSetBoundsProg(geomType, module, "AMRCellGeomBounds");
      owlGeomTypeSetIntersectProg(geomType,
                                  SAMPLING_RAY_TYPE,
                                  module,
                                  "AMRCellGeomIsect");
      owlGeomTypeSetClosestHit(geomType,
                               SAMPLING_RAY_TYPE,
                               module,
                               "AMRCellGeomCH");

      OWLGeom geom = owlGeomCreate(context, geomType);
      owlGeomSetPrimCount(geom, cells.size());

      cellBuffer = owlDeviceBufferCreate(context, OWL_USER_TYPE(AMRCell),
                                         cells.size(),
                                         cells.data());

      scalarBuffer = owlDeviceBufferCreate(context, OWL_FLOAT,
                                           scalars.size(),
                                           scalars.data());

      owlGeomSetBuffer(geom,"amrCellBuffer",cellBuffer);
      owlGeomSetBuffer(geom,"scalarBuffer",scalarBuffer);

      owlBuildPrograms(context);

      blas = owlUserGeomGroupCreate(context, 1, &geom);
      owlGroupBuildAccel(blas);

#ifdef EXA_STITCH_MIRROR_EXAJET
      tlas = owlInstanceGroupCreate(context, 2);
#else
      tlas = owlInstanceGroupCreate(context, 1);
#endif
      owlInstanceGroupSetChild(tlas, 0, blas);

#ifdef EXA_STITCH_MIRROR_EXAJET
      owlInstanceGroupSetChild(tlas, 1, blas);
      owlInstanceGroupSetTransform(tlas, 1, &mirrorTransform);
#endif

      owlGroupBuildAccel(tlas);

      // build the grid
      if (!grid || grid->dims==vec3i(0))
        return false;

      grid->build(context,shared_from_this()->as<AMRCellSampler>(),grid->dims,cellBounds);
#ifdef EXA_STITCH_MIRROR_EXAJET
      grid->deviceTraversable.mirrorInvTransform = rcp((const affine3f &)mirrorTransform);
      grid->deviceTraversable.mirrorPlane.axis = 1;
      grid->deviceTraversable.mirrorPlane.offset = cellBounds.upper.y;
#endif
      Sampler::majorantAccel.grid = model->grid;
      Sampler::maxOpacities = model->grid->maxOpacities;

      return true;
    }

    return false;
  }

  void AMRCellSampler::computeMaxOpacities(OWLContext owl, OWLBuffer colorMap, range1f xfRange)
  {
    if (!model->grid || model->grid->dims==vec3i(0))
      return;

    model->grid->computeMaxOpacities(owl,colorMap,xfRange);
  }

  std::vector<OWLVarDecl> AMRCellSampler::getLPVariables()
  {
    std::vector<OWLVarDecl> vars
      = {
         { "acs.sampleBVH", OWL_GROUP, OWL_OFFSETOF(LP,sampleBVH) },
         { "acs.mirrorInvTransform", OWL_USER_TYPE(affine3f), OWL_OFFSETOF(LP,mirrorInvTransform)}
    };
    return vars;
  }

  void AMRCellSampler::setLPs(OWLParams lp)
  {
    owlParamsSetGroup(lp,"acs.sampleBVH",tlas);
#ifdef EXA_STITCH_MIRROR_EXAJET
    affine3f mirrorInvTransform = rcp((const affine3f &)model->mirrorTransform);
    owlParamsSetRaw(lp,"acs.mirrorInvTransform",&mirrorInvTransform);
#endif
  }
} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

