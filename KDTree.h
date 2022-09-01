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

#include <string>
#include <vector>
#include "KDTree.cuh"

namespace exa {


  struct KDTree
  {
    typedef std::shared_ptr<KDTree> SP;

    KDTree();
   ~KDTree();

    static KDTree::SP load(const std::string fileName);

    void setLeaves(const std::vector<owl::box3f> &leaves);

    void setModelBounds(const owl::box3f &bounds);

    bool initGPU(int deviceID=0);

    KDTreeTraversable deviceTraversable;

  private:

    std::vector<KDTreeNode> nodes;
    std::vector<PrimRef>    primRefs;
    owl::box3f              modelBounds;

    int deviceID;
  };

} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

