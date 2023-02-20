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

#include <iostream>
#include <ostream>
#include <queue>
#include <vector>
#include "owl/common/parallel/parallel_for.h"

namespace volkd {

constexpr static int NumBins = 8;
constexpr static int NumLeavesDesired = 1500;

struct Node
{
  box3f domain;
  float priority;

  bool operator<(const Node &other) const {
    return priority < other.priority; // not a typo..
  }
};

/*! kd-tree data structure and single-threaded, recursive builder */
struct KDTree
{
  template <typename Volume>
  KDTree(const Volume &vol, const std::vector<float> *cm = nullptr);

  template <typename Volume>
  void build(const Volume &vol);

  /*! an optional RGBA colormap that can be used for classification
    (4-tuples for convenience, only the alpha components are relevant) */
  const std::vector<float> *rgbaCM = nullptr;

  // nodes with priority used for splitting
  std::priority_queue<Node> nodes;
};

inline float surface_area(box3f const& box)
{
  auto s = box.size();
  return (s.x * s.y + s.y * s.z + s.z * s.x) * 2.f;
}

inline float diag(box3f const& box)
{
  auto s = box.size();
  return sqrtf(s.x * s.x + s.y * s.y + s.z * s.z);
}

template <typename Volume>
KDTree::KDTree(const Volume &vol, const std::vector<float> *cm)
  : rgbaCM(cm)
{
  box3f V = vol.getBounds();
  nodes.push({V,FLT_MAX});
  build(vol);
}

template <typename Volume>
void KDTree::build(const Volume &vol) {
  while (nodes.size() < NumLeavesDesired) {

    std::cout << "Leaf nodes generated so far: " << nodes.size() << ", picking another node to split...\n";

    // auto cpy_queue = nodes;
    // float minPriority=1e31f, maxPriority=-1e31f;
    // while (!cpy_queue.empty()) {
    //   Node n = cpy_queue.top();
    //   cpy_queue.pop();
    //   minPriority = fminf(minPriority,n.priority);
    //   maxPriority = fmaxf(maxPriority,n.priority);
    // }

    // std::cout << "Cost range of nodes in queue: [" << minPriority << ',' << maxPriority << "]\n";

    Node node = nodes.top();
    nodes.pop();
    box3f V = node.domain;
    std::cout << "Picking node: " << V << " with costs " << node.priority << '\n';

    struct Costs { float left, right, total; };

    int bestAxis = -1;
    float bestPlane;
    Costs bestCost = {FLT_MAX,FLT_MAX,FLT_MAX};
    for (int axis=0; axis<=2; ++axis) {
      float begin = V.lower[axis];
      float end   = V.upper[axis];

      if (begin == end)
        continue;
      float step  = (end-begin)/NumBins;

      Costs costs[NumBins-1];
      owl::parallel_for(NumBins-1, [&] (int i) {
        float plane = begin+step*(i+1);

        box3f L = V;
        box3f R = V;

        L.lower[axis] = begin;
        L.upper[axis] = plane;

        R.lower[axis] = plane;
        R.upper[axis] = end;

        range1f rangeL = vol.min_max(L,rgbaCM);
        range1f rangeR = vol.min_max(R,rgbaCM);

        float CL = surface_area(L) * diag(L) * rangeL.upper;
        float CR = surface_area(R) * diag(R) * rangeR.upper;
        float C = CL+CR;

        costs[i] = {CL,CR,C};

        std::cout << "Testing axis " << axis << ", plane " << plane
                  << ", muL: " << rangeL.upper << ", muR: " << rangeR.upper
                  << ", CL: " << CL << ", CR: " << CR << ", C: " << C << '\n';

      });

      for (int i=0; i<NumBins-1; ++i) {
        float CL = costs[i].left;
        float CR = costs[i].right;
        float C = costs[i].total;
        if (C < bestCost.total) {
          bestAxis = axis;
          bestPlane = begin+step*(i+1);
          bestCost = {CL,CR,C};
        }
      }
    }

    float CL = bestCost.left;
    float CR = bestCost.right;
    float C  = bestCost.total;

    std::cout << V << ", split at: (" << bestAxis << ',' << bestPlane << ")\n"
              << "SAH costs(L): " << CL
              << ", SAH costs(R): " << CR
              << " (sum: " << C << ")\n\n";

    box3f L = V;
    L.upper[bestAxis] = bestPlane;
    nodes.push({L,CL});

    box3f R = V;
    R.lower[bestAxis] = bestPlane;
    nodes.push({R,CR});
  }
}

} // namespace volkd

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

