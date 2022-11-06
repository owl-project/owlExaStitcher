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

#include "BigMeshSampler.h"

namespace exa {

  bool BigMeshSampler::build(OWLContext owl, Model::SP model)
  {
#ifdef HAVE_BIGMESH
    bmModel = model->as<BigMeshModel>();

    if (!bmModel)
      return false;

    if (!bmSampler)
      bmSampler = std::make_shared<fun::BigMeshSampler>();

    bmSampler->build(owl,bmModel->getFunModel());

    bmModel->grid->worldBounds = model->cellBounds;
    //std::cout << model->cellBounds << '\n';

    const vec3i dims = bmModel->grid->dims;
    bmModel->grid->valueRanges = owlDeviceBufferCreate(owl, OWL_USER_TYPE(range1f),
                                                       dims.x*size_t(dims.y)*dims.z,
                                                       nullptr);
    bmModel->grid->maxOpacities = owlDeviceBufferCreate(owl, OWL_FLOAT,
                                                        dims.x*size_t(dims.y)*dims.z,
                                                        nullptr);
    bmSampler->buildMacroCells((range1f*)owlBufferGetPointer(bmModel->grid->valueRanges,0),
                               bmModel->grid->dims, bmModel->grid->worldBounds);
#ifdef EXA_STITCH_MIRROR_EXAJET
    bmModel->grid->deviceTraversable.traversable.dims = dims;
    bmModel->grid->deviceTraversable.traversable.bounds = model->cellBounds;
#else
    bmModel->grid->deviceTraversable.dims = dims;
    bmModel->grid->deviceTraversable.bounds = model->cellBounds;
#endif

    Sampler::majorantAccel.grid = bmModel->grid;
    Sampler::maxOpacities = bmModel->grid->maxOpacities;

    return true;
#else
    return false;
#endif
  }

  void BigMeshSampler::computeMaxOpacities(OWLContext owl,
                                           OWLBuffer colorMap,
                                           range1f xfRange)
  {
#ifdef HAVE_BIGMESH
    if (!bmModel->grid || bmModel->grid->dims==vec3i(0))
      return;

    bmModel->grid->computeMaxOpacities(owl,colorMap,xfRange);
#endif
  }

  std::vector<OWLVarDecl> BigMeshSampler::getLPVariables()
  {
#ifdef HAVE_BIGMESH
    if (!bmSampler)
      bmSampler = std::make_shared<fun::BigMeshSampler>();

    return bmSampler->getLPVariables();
#else
    return {};
#endif
  }

  void BigMeshSampler::setLPs(OWLParams lp)
  {
#ifdef HAVE_BIGMESH
    if (!bmSampler)
      bmSampler = std::make_shared<fun::BigMeshSampler>();

    bmSampler->setLPs(lp);
#endif
  }

} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

