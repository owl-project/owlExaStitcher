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

#include "Model.h"

namespace fun {
  struct Model;
} // ::fun

namespace exa {

  struct BigMeshModel : Model
  {
    typedef std::shared_ptr<BigMeshModel> SP;

    static BigMeshModel::SP load(const std::string bmFileName);

    BigMeshModel();
   ~BigMeshModel();

    std::shared_ptr<fun::Model> getFunModel();

    struct Pimpl;
    std::unique_ptr<Pimpl> impl;
  };
} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

