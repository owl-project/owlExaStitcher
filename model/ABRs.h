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

#include <common.h>

namespace exa {

  struct ExaBrick {
    inline __both__
    size_t numCells() const
    {
      return size.x*size_t(size.y)*size.z;
    }
    
    inline __both__
    box3f getBounds() const
    {
      return box3f(vec3f(lower),
                   vec3f(lower + size*(1<<level)));
    }

    inline __both__
    box3f getDomain() const
    {
      const float cellWidth = (float)(1<<level);
      return box3f(vec3f(lower) - 0.5f*cellWidth,
                   vec3f(lower) + (vec3f(size)+0.5f)*cellWidth);
    }

    inline __both__
    int getIndexIndex(const vec3i &idx) const
    {
      return begin + idx.x + size.x*(idx.y+size.y*idx.z);
    }

    vec3i    lower;
    vec3i    size;
    int      level;
    /*! offset into the scalar data index buffer (in which all bricks
        are stored sequentially) */
    uint32_t begin;
  };

  /*! denotes a region in which a given number of bricks overlap */
  struct ABR {
    /* space covered by this region - should not overlap any other
       brickregion */
    box3f   domain;
    /*! range of values of all cells overlapping this region */
    range1f valueRange;
    /*! offset in parent's leaflist class where our leaf list starts */
    int     leafListBegin;
    int     leafListSize;
    float   finestLevelCellWidth;
  };

  /*! helper class that allows for keeping track which bricks overlap
      in which basis-function region */
  struct ABRs {
    
    void buildFrom(const ExaBrick *bricks,
                   const size_t numBricks,
                   const float *scalarFields,
                   const unsigned numFields,
                   const unsigned numScalarsPerField);
    void addLeaf(std::vector<std::pair<box3f,int>> &buildPrims,
                 const box3f &domain);
    void buildRec(std::vector<std::pair<box3f,int>> &buildPrims,
                  const box3f &domain);
    void computeValueRange(ABR &abr,
                           const ExaBrick *bricks,
                           const float *scalarFields,
                           const unsigned numFields,
                           const unsigned numScalarsPerField);
    
    std::mutex mutex;
    std::vector<ABR> value;
    /*! offset in parent's leaflist class where our leaf list starst */
    std::vector<int> leafList;
  };
  
} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

