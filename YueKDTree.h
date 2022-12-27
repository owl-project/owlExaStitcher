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
#include <vector>
#ifndef VOLKD_HAVE_OWN_MATH
#include "math.h"
#endif

namespace volkd {

typedef box2f Rect;

struct Bin
{
  int binID;
  float height;
};

/*! Function from the Yue et al. suppl. PDF, given a discrete histogram curve,
 computes the largest rectangle underneath */
static Rect findLargestRectInHistogram(const Bin *bins, int numBins)
{
  int i=0;
  struct /*stack*/ {
    int left;
    float h;
  } S[1024];
  int stackPtr = 0;
  Rect R{1e30f,-1e30f};
  float maxArea = 0.f;
  float prevHeight = 0.f;

  while (i < numBins) {
    float h = bins[i].height;

    if (prevHeight < h)
      S[stackPtr++] = {i,h};
    
    if (prevHeight > h || i==numBins-1) {
      int j = S[stackPtr > 0 ? stackPtr-1 : 0].left;
      while (stackPtr && (S[stackPtr-1].h >= h || i==numBins-1)) {
        auto s = S[--stackPtr];
        j = s.left;
        int k = s.h>h ? i : i+1;
        float area = abs(k-j)*s.h;
        Rect temp = {{float(j),0.f},{float(k),s.h}};
        if (area > maxArea) {
          maxArea = area;
          R = {{float(j),0.f},{float(k),s.h}};
        }
      }
      S[stackPtr++] = {j,h};
    }
    prevHeight = h;
    i++;
  }

  // assume that bins are ordered, and we can safely
  // offset the whole rect by the first binID:
  R.lower.x += bins[0].binID;
  R.upper.x += bins[0].binID;

  return R;
}

struct KDTreeNode
{
  box3i domain;
  int splitAxis;
  int splitPlane;
  int childIDs[2];
};

/*! kd-tree data structure and single-threaded, recursive builder */
struct KDTree
{
  template <typename Volume>
  KDTree(Volume vol, const std::vector<float> *cm = nullptr);

  template <typename Volume>
  void buildRec(Volume vol, box3i V);

  /*! compute the reduced area NR (rectangle area - alignment constant for
    the largest rectangle over the majorant curve */
  void computeNR(const std::vector<Bin> &k, int begin, int end, int step, float &NR,
                 Rect &r);

  /*! an optional RGBA colormap that can be used for classification
    (4-tuples for convenience, only the alpha components are relevant) */
  const std::vector<float> *rgbaCM = nullptr;

  std::vector<KDTreeNode> nodes;

  size_t numLeaves = 0;
  uint64_t volumeInLeaves = 0;
};

template <typename Volume>
KDTree::KDTree(Volume vol, const std::vector<float> *cm)
  : rgbaCM(cm)
{
  box3i V = vol.getBounds();
  nodes.push_back({V,-1,-1,{-1,-1}});
  buildRec(vol,V);
}

template <typename Volume>
void KDTree::buildRec(Volume vol, box3i V) {
  int bestAxis = -1;
  float bestNR = -1e31f; // best "reduced" area (benefit of performing a split)
  int bestPlane = -1.f;
  for (int axis=0; axis<=2; ++axis) {
    int begin = V.lower[axis];
    int end   = V.upper[axis];
    int step  = 1;
    // ask volume to adapt the iteration range; e.g., the
    // volume might use a coarser step size b/c the cells
    // in V are all on a coarser level
    vol.iterationRange(V,axis,begin,end,step);

    int num_k = (end-begin-step)/step;
    if (num_k <= 0)
      continue;

    std::vector<Bin> k_plus(num_k);
    std::vector<Bin> k_delta(num_k);

    float k_plus_max=-1e31f, k_delta_max=-1e31f;
    int last=(end-(begin+step))/step;
    #pragma omp parallel for
    for (int ii=0; ii<last; ii++) {
      int i = ii*step+begin+step;
      float minValue=1e31f, maxValue=-1e31f;
      vol.min_max(V,axis,i,minValue,maxValue,rgbaCM);

      int index = (i-begin-step)/step;
      k_plus[index] = {i,maxValue};
      k_delta[index] = {i,maxValue+minValue};

      // Also compute max of k-curves so we can later invert them
      k_plus_max = fmaxf(k_plus_max,k_plus[index].height);
      k_delta_max = fmaxf(k_delta_max,k_delta[index].height);
    }
 
    // Invert, b/c the "empty" rectangles live _above_ the
    // majorant functions, so the histograms are upside down!
    for (size_t i=0; i<k_plus.size(); ++i) {
      k_plus[i].height = k_plus_max-k_plus[i].height;
      k_delta[i].height = k_delta_max-k_delta[i].height;
    }

    Rect r_plus;
    float NR_plus = 0.f;
    computeNR(k_plus,begin,end,step,NR_plus,r_plus);

    Rect r_delta;
    float NR_delta = 0.f;
    computeNR(k_delta,begin,end,step,NR_delta,r_delta);

    Rect r;
    float NR = 0.f;
    const float F = 0.7f; // magic constant from the paper
    if (NR_plus >= NR_delta*F) {
      r = r_plus;
      NR = NR_plus;
    } else {
      r = r_delta;
      NR = NR_delta;
    }

    if (NR > bestNR) {
      bestAxis = axis;
      bestNR = NR;
      // we build kd-trees, i.e., we need one plane, not two!
      // => pick the rectangle's side that is closer to the midpoint
      float midpoint = (end-begin)*.5f;
      bestPlane = fabsf(r.lower.x-midpoint)<fabsf(r.upper.x-1-midpoint)
                        ? r.lower.x
                        : r.upper.x-1;
    }
  }

  if (bestNR > 0.f) {
    std::cout << V << ", split at: (" << bestAxis << ',' << bestPlane
              << "), benefit: " << bestNR << '\n';

    int nodeID = nodes.size()-1;
    nodes[nodeID].splitAxis = bestAxis;
    nodes[nodeID].splitPlane = bestPlane;

    box3i L = V;
    L.upper[bestAxis] = bestPlane;
    nodes.push_back({L,-1,-1,{-1,-1}});
    nodes[nodeID].childIDs[0] = (int)nodes.size();
    buildRec(vol,L);

    box3i R = V;
    R.lower[bestAxis] = bestPlane;
    nodes.push_back({R,-1,-1,{-1,-1}});
    nodes[nodeID].childIDs[1] = (int)nodes.size();
    buildRec(vol,R);
  } else {
    std::cout << "Make leaf: " << V << '\n';
    numLeaves++;
    volumeInLeaves += V.volume();
    std::cout << "# leaves: " << prettyNumber(numLeaves) << '\n';
    std::cout << "# inner: " << prettyNumber(nodes.size()) << '\n';
    std::cout << "Volume in leaves: " << prettyNumber(volumeInLeaves)
              << '/' << prettyNumber(vol.cellBounds.volume()) << '\n';
  }
}

void KDTree::computeNR(const std::vector<Bin> &k, int begin, int end, int step,
                       float &NR, Rect &r)
{
  r = findLargestRectInHistogram(k.data(),k.size());
  if (r.empty()) {
    NR = -1e31f;
    return;
  }
  float AR = area(r)*step; // TODO: "real" area of rectangle?!
  // T(R') is 1 if at most one corner "touches" the majorant curve
  // and is 2 if both lower corners touch the curve
  float TR = 1.f;
  float k1 = k[(int(r.lower.x)-begin-step)/step].height;
  float k2 = k[(int(r.upper.x-1)-begin-step)/step].height;
  if (k1==r.upper.y && k2==r.upper.y) TR = 2.f;
  NR = AR-TR;
}

} // namespac3 volkd


