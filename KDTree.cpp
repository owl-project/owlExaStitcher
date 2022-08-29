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
#include <cuda_runtime.h>
#include "KDTree.h"

using namespace owl;
using namespace owl::common;

namespace exa {

  inline size_t linearIndex(const vec3i index, const vec3i dims)
  {
    return index.z * size_t(dims.x) * dims.y
         + index.y * dims.x
         + index.x;
  }

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

  KDTree::SP KDTree::build(uint32_t numBoxes, const box3f *boxes)
  {
    KDTree::SP result = std::make_shared<KDTree>();

    KDTreeNode root;
    root.first = 0;
    root.last = numBoxes;
    box3f rootDomain;

    result->primRefs.resize(numBoxes);

    for (uint32_t i=0; i<numBoxes; ++i) {
      rootDomain.extend(boxes[i]);
      result->primRefs[i] = {i,boxes[i]};
    }

    result->nodes.push_back(root);
    result->domains.push_back(rootDomain);

    result->doSplit(0);

    std::cout << "Done building kd-tree\n";

    return result;
  }

  bool KDTree::initGPU(int deviceID)
  {
    if (nodes.empty() || domains.empty() || primRefs.empty())
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

  void KDTree::doSplit(size_t nodeID)
  {
    KDTreeNode &node = nodes[nodeID];
    owl::box3f &domain = domains[nodeID];

    int splitAxis = arg_max(domain.size());

    bool foundLeaf = node.last-node.first<=1;

    if (foundLeaf) {
      // make leaf
      node.first = ~node.first;
      return;
    }

    // Find and split off regions that are most homogeneous
    // (i.e., minimize max-min)
    float splitPlane = domain.lower[splitAxis];
    float opt = FLT_MAX;
    float lo = domain.lower[splitAxis]+1;
    float hi = domain.upper[splitAxis]-1;
    float inc = (lo-hi)/16;
    for (float p=lo; p<=hi; ++p) {
      box3f domainL;
      box3f domainR;

      for (int i=node.first; i<node.last; ++i) {
        if (primRefs[i].box.center()[splitAxis] < p)
          domainL.extend(primRefs[i].box);
        else
          domainR.extend(primRefs[i].box);
      }

      box3f intersection(max(domainL.lower,domainR.lower),min(domainL.upper,domainR.upper));
      if (volume(intersection) > 0.f) {
        throw std::runtime_error("Boxes are not allowed to overlap!");
      }

      float volL = volume(domainL);
      float volR = volume(domainR);
      float diff = std::abs(volR-volL);

      if (diff < opt) {
        splitPlane = p;
        opt = diff;
      }
    }

    std::partition(primRefs.begin()+node.first,
                   primRefs.begin()+node.last,
                   [&](const PrimRef &ref) {
                     return ref.box.center()[splitAxis] < splitPlane;
                   });

    // Find first primitive in R node
    int splitIndex = node.first;
    int first = node.first;
    int last = node.last;
    for (int i=first; i<last; ++i) {
      if (primRefs[i].box.center()[splitAxis] >= splitPlane) {
        splitIndex = i;
        break;
      }
    }

    int firstChildID = (int)nodes.size();

    // Make inner node
    node.first      = firstChildID;
    node.last       = node.first+2;
    node.splitPlane = splitPlane;
    node.splitAxis  = splitAxis;

    // Recurse
    KDTreeNode L;
    L.first  = first;
    L.last   = splitIndex;
    L.splitPlane = FLT_MAX;
    L.splitAxis = -1;
    box3f domainL = domain;
    domainL.upper[splitAxis] = splitPlane;

    KDTreeNode R;
    R.first  = splitIndex;
    R.last   = last;
    R.splitPlane = FLT_MAX;
    R.splitAxis = -1;
    box3f domainR = domain;
    domainR.lower[splitAxis] = splitPlane;

    nodes.push_back(L);
    nodes.push_back(R);
    domains.push_back(domainL);
    domains.push_back(domainR);

    doSplit(firstChildID);
    doSplit(firstChildID+1);
  }

} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

