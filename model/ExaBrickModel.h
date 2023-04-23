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

#include <vector>
#include <common.h>
#include <Grid.h>
#include "Model.h"
#include "ABRs.h"

namespace exa {

  struct ExaBrickModel : Model
  {
    typedef std::shared_ptr<ExaBrickModel> SP;

    static ExaBrickModel::SP load(const std::string brickFileName,
                                  const std::string scalarFileName,
                                  const std::string kdTreeFileName);

    static ExaBrickModel::SP load(const std::string brickFileName,
                                  const std::string scalarFileName0,
                                  const std::string scalarFileName1,
                                  const std::string scalarFileName2,
                                  const std::string scalarFileName3,
                                  const std::string scalarFileName4,
                                  const std::string scalarFileName5,
                                  const std::string scalarFileName6,
                                  const std::string scalarFileName7,
                                  const std::string kdTreeFileName);

    std::vector<ExaBrick> bricks;
    std::vector<float>    scalars;
    unsigned              numFields;
    unsigned              numScalarsPerField;
    ABRs                  abrs;
    KDTree::SP            kdtree; // optional kd-tree over bricks
    std::vector<std::vector<int>> adjacentBricks; // adjacency list to splat majorants into neighboring bricks

    // Statistics
    void memStats(size_t &bricksBytes,
                  size_t &scalarsBytes,
                  size_t &abrsBytes,
                  size_t &abrLeafListBytes);
  
    static int traversalMode;
    static int samplerMode;
  };

} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

