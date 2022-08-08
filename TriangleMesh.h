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

#include <memory>
#include <vector>
#include <owl/common/math/vec.h>

namespace exa {

  struct TriangleMesh {
    typedef std::shared_ptr<TriangleMesh> SP;

    static TriangleMesh::SP loadOne(std::ifstream &in);
    static std::vector<TriangleMesh::SP> load(const std::string &fileName = "none");
    
    std::vector<owl::vec3i> index;
    std::vector<owl::vec3f> vertex;
  };
  
} // ::exa
