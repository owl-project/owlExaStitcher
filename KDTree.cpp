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

#include <float.h>
#include <fstream>
#include <numeric>
#include <cuda_runtime.h>
#include "KDTree.h"

using namespace owl;
using namespace owl::common;

namespace exa {

  KDTree::KDTree()
  {
    deviceTraversable.nodes    = nullptr;
    deviceTraversable.domains  = nullptr;
    deviceTraversable.primRefs = nullptr;
  }

  KDTree::~KDTree()
  {
    int prevDeviceID = -1;
    cudaGetDevice(&prevDeviceID);

    if (deviceID != prevDeviceID)
      cudaSetDevice(deviceID);

    cudaFree(deviceTraversable.nodes);
    cudaFree(deviceTraversable.domains);
    cudaFree(deviceTraversable.primRefs);

    if (prevDeviceID != deviceID)
      cudaSetDevice(prevDeviceID);
  }

  KDTree::SP KDTree::load(const std::string fileName)
  {
    KDTree::SP result = std::make_shared<KDTree>();

    std::ifstream in(fileName, std::ios::binary | std::ios::ate);
    std::streampos size = in.tellg();
    result->nodes.resize(size/sizeof(KDTreeNode));
    in.seekg(0);
    in.read((char*)result->nodes.data(), size);

    return result;
  }

  bool KDTree::initGPU(int deviceID)
  {
    if (nodes.empty())
      return false;

    this->deviceID = deviceID;

    int prevDeviceID = -1;
    cudaGetDevice(&prevDeviceID);

    if (deviceID != prevDeviceID)
      cudaSetDevice(deviceID);

    if (deviceTraversable.nodes != nullptr) {
      cudaFree(deviceTraversable.nodes);
      cudaFree(deviceTraversable.domains);
      cudaFree(deviceTraversable.primRefs);
    }

    cudaMalloc(&deviceTraversable.nodes,nodes.size()*sizeof(nodes[0]));
    cudaMalloc(&deviceTraversable.domains,domains.size()*sizeof(domains[0]));
    cudaMalloc(&deviceTraversable.primRefs,primRefs.size()*sizeof(primRefs[0]));

    cudaMemcpy(deviceTraversable.nodes,
               nodes.data(),
               nodes.size()*sizeof(nodes[0]),
               cudaMemcpyHostToDevice);

    cudaMemcpy(deviceTraversable.domains,
               domains.data(),
               domains.size()*sizeof(domains[0]),
               cudaMemcpyHostToDevice);

    cudaMemcpy(deviceTraversable.primRefs,
               primRefs.data(),
               primRefs.size()*sizeof(primRefs[0]),
               cudaMemcpyHostToDevice);

    if (prevDeviceID != deviceID)
      cudaSetDevice(prevDeviceID);

    return true;
  }

} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

