// ======================================================================== //
// Copyright 2019 Ingo Wald                                                 //
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

#include <cstddef>
#include <mutex>
#include <set>
#include <vector>
#include "deviceCode.h"

namespace exa {

  /*! helper class that allows for keeping track which bricks overlap
      in which basis-function region */
  struct ABRs {
    
    void buildFrom(const ExaBrick *bricks,
                   const size_t numBricks,
                   const float *scalarFields);
    void addLeaf(std::vector<std::pair<box3f,int>> &buildPrims,
                 const box3f &domain);
    void buildRec(std::vector<std::pair<box3f,int>> &buildPrims,
                  const box3f &domain);
    void computeValueRange(ABR &abr,
                           const ExaBrick *bricks,
                           const float *scalarFields);
    
    std::mutex mutex;
    std::vector<ABR> value;
    /*! offset in parent's leaflist class where our leaf list starst */
    std::vector<int> leafList;
  };
  
} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

