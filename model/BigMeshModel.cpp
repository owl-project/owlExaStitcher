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

#ifdef HAVE_BIGMESH
#include <model/BigMeshModel.h>
#endif
#include "BigMeshModel.h"

namespace exa {

  struct BigMeshModel::Pimpl
  {
#ifdef HAVE_BIGMESH
    fun::BigMeshModel::SP bmModel;
#endif
  };

  BigMeshModel::SP BigMeshModel::load(const std::string bmFileName)
  {
#ifdef HAVE_BIGMESH
    BigMeshModel::SP model = std::make_shared<BigMeshModel>();

    fun::Model::SP funModel = fun::Model::load(bmFileName);
    if (!funModel->as<fun::BigMeshModel>())
      return nullptr;
      
    model->impl->bmModel = funModel->as<fun::BigMeshModel>();

    model->cellBounds = funModel->getBounds();
    model->valueRange = funModel->valueRange;
    std::cout << model->valueRange << '\n';

    return model;
#else
    return nullptr;
#endif
  }

  BigMeshModel::BigMeshModel()
    : impl(new Pimpl)
  {
  }

  BigMeshModel::~BigMeshModel()
  {
  }

  std::shared_ptr<fun::Model> BigMeshModel::getFunModel()
  {
#ifdef HAVE_BIGMESH
    return impl->bmModel;
#else
    return nullptr;
#endif
  }
} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

