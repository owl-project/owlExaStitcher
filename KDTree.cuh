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

#include <assert.h>
#include <float.h>
#include <stdint.h>
#include <owl/common/math/box.h>

namespace exa {

  struct KDTreeNode {
    int first, last;
    float splitPlane;
    int splitAxis;

    inline __both__ int get_child(int id) const
    {
      return first+id;
    }

    inline __both__ bool is_leaf() const
    {
      return first<0;
    }

    inline __both__ int first_prim() const
    {
      return ~first;
    }

    inline __both__ int num_prims() const
    {
      return last - first_prim();
    }
  };

  struct PrimRef {
    uint32_t primID;
    owl::box3f box;
  };

  struct KDTreeTraversable {
    KDTreeNode *nodes;
    owl::box3f *domains;
    PrimRef    *primRefs;
  };

  struct KDTreeHitRec {
    bool hit;
    float t;
  };

  namespace kd { // kdtree helpers
    template <typename Ray>
    inline __both__ bool boxTest(const Ray &ray, const owl::vec3f &invDir,
                                 const owl::box3f &box,
                                 float &t0, float &t1)
    {
      using namespace owl;

      const vec3f t_lo = (box.lower - ray.origin) / ray.direction;
      const vec3f t_hi = (box.upper - ray.origin) / ray.direction;

      const vec3f t_nr = min(t_lo,t_hi);
      const vec3f t_fr = max(t_lo,t_hi);

      t0 = max(ray.tmin,reduce_max(t_nr));
      t1 = min(ray.tmax,reduce_min(t_fr));
      return t0 < t1;
    }


    template <typename Ray, typename PRD, typename Isect>
    inline __both__
    void traceRay(KDTreeTraversable tree, const Ray &ray, PRD &prd, Isect isect) {

      using namespace owl;

      struct StackEntry {
        unsigned nodeID;
        float tnear;
        float tfar;
      };
      typedef StackEntry Stack[32];

      box3f bbox = tree.domains[0];

      vec3f invDir = 1.f/ray.direction;

      float t0 = 0.f, t1 = 0.f;
      if (!kd::boxTest(ray,invDir,bbox,t0,t1))
        return;

      Stack stack;
      unsigned ptr = 0;
      // Start at the root
      StackEntry se{0,t0,t1};

      float t = t0;

      // while ray not terminated
      while (1) {
        // while node does not contain primitives
        //     traverse to the next node

        KDTreeNode node = tree.nodes[se.nodeID];

        while (!node.is_leaf()) {
          unsigned nearChild = signbit(ray.direction[node.splitAxis]);
          unsigned farChild  = 1^nearChild;

          float d = (node.splitPlane - ray.origin[node.splitAxis]) * invDir[node.splitAxis];

          if (d <= se.tnear) {
            se.nodeID = node.get_child(farChild);
          } else if (d >= se.tfar) {
            se.nodeID = node.get_child(nearChild);
          } else {
            stack[ptr++] = { (unsigned)node.get_child(farChild), d, se.tfar };
            se.nodeID = node.get_child(nearChild);
            se.tfar = d;
          }
          node = tree.nodes[se.nodeID];
        }

        // found a leaf
        KDTreeHitRec hitRec = {false,FLT_MAX};
        isect(ray,
              prd,
              tree.primRefs[node.first_prim()].primID,
              se.tnear,
              se.tfar,
              hitRec);

        if (hitRec.hit)
          t = max(t,hitRec.t);

        if (se.tfar <= t)
          break;

        if (ptr == 0)
          break;

        se = stack[--ptr];
      }
    }
  } // ::kd
} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

