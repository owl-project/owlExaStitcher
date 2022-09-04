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
    struct index_range {
      unsigned first;
      unsigned last;
    };

    __both__
    inline bool is_inner() const
    {
      return axis >> 30 != 3;
    }

    __both__
    inline bool is_leaf() const
    {
      return axis >> 30 == 3;
    }

    __both__
    inline int get_split() const
    {
      assert(is_inner());
      return split;
    }

    __both__
    inline int get_max_level() const
    {
      assert(is_inner());
      return max_level;
    }

    __both__
    inline unsigned get_axis()
    {
      assert(is_inner());
      return unsigned(axis >> 30);
    }

    __both__
    inline unsigned get_child(unsigned i = 0) const
    {
      assert(is_inner());
      return (first_child & 0x3FFFFFFF) + i;
    }

    __both__
    inline index_range get_indices() const
    {
      assert(is_leaf());
      return { first_prim, first_prim + (num_prims & 0x3FFFFFFF) };
    }

    __both__
    inline unsigned get_first_primitive() const
    {
      assert(is_leaf());
      return first_prim;
    }

    __both__
    inline unsigned get_num_primitives() const
    {
      assert(is_leaf());
      return num_prims & 0x3FFFFFFF;
    }

    __both__
    inline void set_inner(unsigned axis, int split, int max_level)
    {
      assert(/*axis >= 0 &&*/ axis < 3);
      this->axis = axis << 30;
      this->split = split;
      this->max_level = max_level;
    }

    __both__
    inline void set_leaf(unsigned first_primitive_index, unsigned count)
    {
      axis = 3 << 30;
      first_prim = first_primitive_index;
      num_prims |= count;
    }

    __both__
    inline void set_first_child(unsigned index)
    {
      first_child |= index;
    }

    union { 
      int split;
      unsigned first_prim;
    };

    union {
      // AA--.----.----.----
      unsigned axis;

      // --NP.NPNP.NPNP.NPNP
      unsigned num_prims;

      // --FC.FCFC.FCFC.FCFC
      unsigned first_child;
    };

    int max_level;
  };

  struct PrimRef {
    uint32_t primID;
    owl::box3f box;
  };

  struct KDTreeTraversable {
    KDTreeNode *nodes;
    PrimRef    *primRefs;
    owl::box3f  modelBounds;
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

      box3f bbox = tree.modelBounds;

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
          unsigned splitAxis = node.get_axis();
          int splitPlane = node.get_split();
          unsigned nearChild = signbit(ray.direction[splitAxis]);
          unsigned farChild  = 1^nearChild;

          float d = (splitPlane - ray.origin[splitAxis]) * invDir[splitAxis];

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
              tree.primRefs[node.get_first_primitive()].primID,
              se.tnear,
              se.tfar,
              hitRec);

        if (hitRec.hit)
          break;

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

