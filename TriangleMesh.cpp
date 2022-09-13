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

#include <fstream>
#include <owl/common/math/box.h>
#include "TriangleMesh.h"

using owl::box3f;
using owl::vec3f;
using owl::vec3i;

namespace exa {

  TriangleMesh::SP TriangleMesh::loadOne(std::ifstream &in)
  {
    if (!in.good() || in.eof())
      return TriangleMesh::SP();
      
    TriangleMesh::SP result = std::make_shared<TriangleMesh>();
      
    int numVerts;
    in.read((char *)&numVerts,sizeof(numVerts));
    if (!in.good() || in.eof())
      return TriangleMesh::SP();
    result->vertex.resize(numVerts);
    in.read((char *)result->vertex.data(),numVerts*sizeof(vec3f));

    int numTris;
    in.read((char *)&numTris,sizeof(numTris));
    result->index.resize(numTris);
    in.read((char *)result->index.data(),numTris*sizeof(vec3i));

    box3f bounds;
    for (auto idx : result->index) {
      if (idx.x < 0 || idx.x >= result->vertex.size())
        throw std::runtime_error("broken triangle model");
      bounds.extend(result->vertex[idx.x]);
      if (idx.y < 0 || idx.y >= result->vertex.size())
        throw std::runtime_error("broken triangle model");
      bounds.extend(result->vertex[idx.y]);
      if (idx.z < 0 || idx.z >= result->vertex.size())
        throw std::runtime_error("broken triangle model");
      bounds.extend(result->vertex[idx.z]);
    }
    return result;
  }

  std::vector<TriangleMesh::SP> TriangleMesh::load(const std::string &fileName)
  {
    std::ifstream in(fileName, std::ios::binary);
    std::cout << fileName << '\n';
    if (!in.good())
      throw std::runtime_error("cannot open file");
      
    std::vector<TriangleMesh::SP> result;
    while (1) {
      TriangleMesh::SP next = TriangleMesh::loadOne(in);
      if (!next) break;
      result.push_back(next);
    }
      
    return result;
  }
    
} // ::exa
