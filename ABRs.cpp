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

#include <owl/common/parallel/parallel_for.h>
#include "ABRs.h"

namespace exa {

  int dbg_biggestLeaf = 0;

  std::mutex statMutex;
  size_t stat_numRegions = 0;
  size_t stat_numBricks  = 0;
  size_t stat_numCells  = 0;
  double stat_totalVolumeInRegions = 0;
  double stat_volumeWeightedNumBrickInRegion = 0;
  size_t stat_numBricksInRegions = 0;
  int    stat_maxBricksPerRegion = 0;
  
  void ABRs::addLeaf(std::vector<std::pair<box3f,int>> &buildPrims, const box3f &domain)
  {
    if (domain.lower.x >= domain.upper.x) return;
    if (domain.lower.y >= domain.upper.y) return;
    if (domain.lower.z >= domain.upper.z) return;
    
#if 0
    {
      std::lock_guard<std::mutex> lock(mutex);
      std::cout << "-------------------------------------------------------" << std::endl;
      std::cout << "- making leaf #" << brickRegions.size() << " : " << domain << " num=" << buildPrims.size() << std::endl;
      for (int i=0;i<buildPrims.size();i++) {
        int brickID = buildPrims[i].second;
        auto &brick = *exa->bricks[brickID];
        std::cout << "  brick " << brickID << " bp " << buildPrims[i].first
                  << " cw " << (1<<brick.level)
                  << " bounds " << brick.getBounds()
                  << " domain " << brick.getDomain() << std::endl;
      }
    }
#endif
    /* no need to recompute bounds of all build prims - we've always
    clipped each prims' bounds to the domain, so neither can be
    outside the domain; and since we would have had another valid
    split if any of these biuldprims had had a plane _withint_ the
    domain we also know it must be tight */
    std::set<int> allBrickIDs;
    for (auto &bp : buildPrims)
      allBrickIDs.insert(bp.second);
    if (allBrickIDs.empty()) return;

    ABR newLeaf;
    newLeaf.domain = domain;
    newLeaf.leafListSize = allBrickIDs.size();


    {
      std::lock_guard<std::mutex> lock(statMutex);
      double vol = domain.volume();
      stat_totalVolumeInRegions += vol;
      stat_volumeWeightedNumBrickInRegion += vol*newLeaf.leafListSize;
      stat_numRegions ++;
      stat_numBricksInRegions += newLeaf.leafListSize;
      stat_maxBricksPerRegion = std::max(stat_maxBricksPerRegion,newLeaf.leafListSize);
    }
    
    std::lock_guard<std::mutex> lock(mutex);
    newLeaf.leafListBegin = leafList.size();
    dbg_biggestLeaf = std::max(dbg_biggestLeaf,(int)buildPrims.size());
    for (auto it : allBrickIDs) {
      leafList.push_back(it);
    }
    this->value.push_back(newLeaf);    
  }
    
  void ABRs::buildRec(std::vector<std::pair<box3f,int>> &buildPrims, const box3f &domain)
  {
    // std::cout << "......................................................." << std::endl;
    // PRINT(buildPrims.size());

    // PRINT(domain);
    
    
    if (buildPrims.empty()) return;
    for (int i=0;i<3;i++)
      if (domain.upper[i] == domain.lower[i]) {
        std::cout << "EMPTY DOMAIN!?" << std::endl;
        PRINT(domain);
        return;
      }
    
    vec3f tgtPos   = domain.center();
    vec3f bestPos  = domain.lower;
    vec3f bestDist = domain.span();
    std::mutex localMutex;

    for (auto &bp : buildPrims) {
      for (int dim=0;dim<3;dim++) {
        for (int side=0;side<2;side++) {
          float pos = (side?bp.first.lower:bp.first.upper)[dim];
          if ((pos <= domain.lower[dim]) || (pos >= domain.upper[dim]))
            continue;
          
          float dist = fabsf(tgtPos[dim] - pos);
          if (dist >= bestDist[dim])
            continue;
          
          std::lock_guard<std::mutex> lock(localMutex);
          // test again, because there may have been a race condition!
          if (dist < bestDist[dim]) {
            bestPos[dim]  = pos;
            bestDist[dim] = dist;
          }
        }
      }
    }

    int splitDim = -1;
    float splitPos;
    int widestDim = arg_max(domain.span());
    
    for (int i=0;i<3;i++) {
      int dim = (widestDim+i)%3;
      if (bestPos[dim] <= domain.lower[dim]
          ||
          bestPos[dim] >= domain.upper[dim]
          ) continue; 
      splitDim = dim;
      splitPos = bestPos[dim];
      break;
    }

    // PRINT(splitDim);
    // PRINT(splitPos);
    // PRINT(vec3i(domain.lower));
    // PRINT(vec3i(domain.upper));
    // PRINT(vec3i(domain.span()));
    
    // PRINT(vec3i(bestPos));

    if (splitDim < 0) {
      addLeaf(buildPrims,domain);
      return;
    }

    std::mutex mutex_l, mutex_r;
    std::vector<std::pair<box3f,int>> bp_l, bp_r;
    box3f domain_l = domain; domain_l.upper[splitDim] = splitPos;
    box3f domain_r = domain; domain_r.lower[splitDim] = splitPos;
    serial_for_blocked(0,buildPrims.size(),128*1024,[&](size_t begin, size_t end){
        std::vector<std::pair<box3f,int>> block_bp_l, block_bp_r;
        for (size_t i=begin;i<end;i++) {
          int brickID = buildPrims[i].second;
          {
            const box3f clipped_l = intersection(buildPrims[i].first,domain_l);
            if (clipped_l.lower.x < clipped_l.upper.x &&
                clipped_l.lower.y < clipped_l.upper.y &&
                clipped_l.lower.z < clipped_l.upper.z)
              block_bp_l.push_back({clipped_l,brickID});
          }
          {
            const box3f clipped_r = intersection(buildPrims[i].first,domain_r);
            if (clipped_r.lower.x < clipped_r.upper.x &&
                clipped_r.lower.y < clipped_r.upper.y &&
                clipped_r.lower.z < clipped_r.upper.z)
              block_bp_r.push_back({clipped_r,brickID});
          }
        }
        {
          std::lock_guard<std::mutex> lock(mutex_l);
          for (auto &bp : block_bp_l) bp_l.push_back(bp);
        }
        {
          std::lock_guard<std::mutex> lock(mutex_r);
          for (auto &bp : block_bp_r) bp_r.push_back(bp);
        }
      });
    buildPrims.clear();

    // iw - note it IS absoltely valid for one side to be empty...
    serial_for(2,[&](int side){
        if (side)
          buildRec(bp_l,domain_l);
        else
          buildRec(bp_r,domain_r);
      });
  }

  
  void ABRs::computeValueRange(ABR &region,
                               const ExaBrick *bricks,
                               const float *scalarBuffers)
  {
    region.valueRange = range1f();
    for (int i=0;i<region.leafListSize;i++) {
      int brickID = leafList[region.leafListBegin+i];
      const ExaBrick &brick = bricks[brickID];
      const float cellWidth = 1<<brick.level;
      
      bool valid_z[brick.size.z];
      for (int iz=0;iz<brick.size.z;iz++) {
        float pos_z = brick.lower.z + (iz+.5f)*cellWidth;
        float lo_z  = pos_z - cellWidth;
        float hi_z  = pos_z + cellWidth;
        valid_z[iz] = (lo_z <= region.domain.upper.z) && (hi_z >= region.domain.lower.z);
      }
      
      bool valid_y[brick.size.y];
      for (int iy=0;iy<brick.size.y;iy++) {
        float pos_y = brick.lower.y + (iy+.5f)*cellWidth;
        float lo_y  = pos_y - cellWidth;
        float hi_y  = pos_y + cellWidth;
        valid_y[iy] = (lo_y <= region.domain.upper.y) && (hi_y >= region.domain.lower.y);
      }
      
      bool valid_x[brick.size.x];
      for (int ix=0;ix<brick.size.x;ix++) {
        float pos_x = brick.lower.x + (ix+.5f)*cellWidth;
        float lo_x  = pos_x - cellWidth;
        float hi_x  = pos_x + cellWidth;
        valid_x[ix] = (lo_x <= region.domain.upper.x) && (hi_x >= region.domain.lower.x);
      }
      

      for (int iz=0;iz<brick.size.z;iz++) {
        if (!valid_z[iz]) continue;
        for (int iy=0;iy<brick.size.y;iy++) {
          if (!valid_y[iy]) continue;
          for (int ix=0;ix<brick.size.x;ix++) {
            if (!valid_x[ix]) continue;
            const size_t scalarIndex
              = size_t(brick.begin)
              + ix
              + brick.size.x*iy
              + brick.size.x*brick.size.y*iz;
            const float scalar = scalarBuffers[scalarIndex];
            region.valueRange.extend(scalar);
          }
        }
      }
    }
    
  }

  void ABRs::buildFrom(const ExaBrick *bricks,
                       const size_t numBricks,
                       const float *scalarBuffers)
  {
    {
      // for paper stats - no other purpose
      stat_numBricks = numBricks;
      stat_numCells  = 0;
      for (int i=0;i<numBricks;i++)
        stat_numCells += bricks[i].numCells();
    }
    
    double t0 = getCurrentTime();
    this->value.clear();
    leafList.clear();
    std::mutex mutex;
    box3f bounds;
    std::vector<std::pair<box3f,int>> buildPrims;
    serial_for_blocked(0,numBricks,4*1023,[&](size_t begin, size_t end){
        std::vector<std::pair<box3f,int>> blockBuildPrims;
        box3f blockBounds;
        for (size_t i=begin;i<end;i++) {
            box3f domain = bricks[i].getDomain();
            blockBounds.extend(domain);
            blockBuildPrims.push_back({domain,i});
        }
        std::lock_guard<std::mutex> lock(mutex);
        bounds.extend(blockBounds);
        for (auto &bp : blockBuildPrims)
          buildPrims.push_back(bp);
      });
    // bounds and buildprom built - recurse...
    std::cout << "-------------------------------------------------------" << std::endl;
    std::cout << "starting to build exa overlap regions, #inputs "
              << prettyDouble(buildPrims.size()) << ", bounds " << bounds << std::endl;
    buildRec(buildPrims,bounds);
    double t1 = getCurrentTime();
    std::cout << "(re-)built block basis function domain(s) in "
              << prettyDouble(t1-t0) << "s" << std::endl;


    std::cout << "computing finest level per region" << std::endl;
    parallel_for(this->value.size(),[&](size_t regionID){
        // for (auto &region : brickRegions) {
        auto &region = this->value[regionID];
        int finestLevel = 1<<30;
        for (int i=0;i<region.leafListSize;i++) {
          int brickID = leafList[region.leafListBegin+i];
          const ExaBrick &brick = bricks[brickID];
          finestLevel = std::min(finestLevel,brick.level);
        }
        region.finestLevelCellWidth = 1<<finestLevel;
        computeValueRange(region,bricks,scalarBuffers);
        // if (regionID % 100000 == 0) {
        //   static std::mutex mutex;
        //   std::lock_guard<std::mutex> lock(mutex);
        //   std::cout << "valuerange of region " << regionID << " : " << region.valueRange << std::endl;
        // }
      });
      
    std::cout << "total num leaves " << prettyDouble(this->value.size()) << std::endl;
    std::cout << "avg leaf size " << (leafList.size() / float(this->value.size())) << std::endl;
    
    std::cout << "biggest leaf: "
              << dbg_biggestLeaf << " bricks" << std::endl;

    std::cout << "stat: #cells   " << prettyDouble(stat_numCells) << std::endl;
    std::cout << "stat: #bricks  " << prettyDouble(stat_numBricks) << std::endl;
    std::cout << "stat: #regions " << prettyDouble(stat_numRegions) << std::endl;
    std::cout << "stat: avg bricks/region (by count) : " << (stat_numBricksInRegions/double(stat_numRegions)) << std::endl;
    std::cout << "stat: avg bricks/region (by volume): " << (stat_volumeWeightedNumBrickInRegion/stat_totalVolumeInRegions) << std::endl;
    std::cout << "stat: brick/region MAX : " << stat_maxBricksPerRegion << std::endl;
  }
  
} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

