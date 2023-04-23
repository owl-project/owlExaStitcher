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

#include <fstream>
#include <owl/common/parallel/parallel_for.h>
#include "ExaBrickModel.h"

namespace exa {

  int ExaBrickModel::traversalMode = EXA_STITCH_EXA_BRICK_TRAVERSAL_MODE;
  int ExaBrickModel::samplerMode   = EXA_STITCH_EXA_BRICK_SAMPLER_MODE;

  ExaBrickModel::SP ExaBrickModel::load(const std::string brickFileName,
                                        const std::string scalarFileName,
                                        const std::string kdTreeFileName)
  {
    return load(brickFileName,scalarFileName,"","","","","","","",kdTreeFileName);
  }

  ExaBrickModel::SP ExaBrickModel::load(const std::string brickFileName,
                                        const std::string scalarFileName0,
                                        const std::string scalarFileName1,
                                        const std::string scalarFileName2,
                                        const std::string scalarFileName3,
                                        const std::string scalarFileName4,
                                        const std::string scalarFileName5,
                                        const std::string scalarFileName6,
                                        const std::string scalarFileName7,
                                        const std::string kdTreeFileName)
  {
    ExaBrickModel::SP result = std::make_shared<ExaBrickModel>();

    std::vector<ExaBrick> &bricks = result->bricks;
    std::vector<float> &scalars   = result->scalars;
    ABRs &abrs                    = result->abrs;
    box3f &cellBounds             = result->cellBounds;
    range1f &valueRange           = result->valueRange;
    auto &adjacentBricks          = result->adjacentBricks;


    // Indices/scalars are later flattened
    constexpr int MAX_FIELDS = 8;
    std::vector<float> orderedScalars[MAX_FIELDS];
    std::vector<int> indices;

    auto loadScalars = [](std::string scalarFileName, std::vector<float> &orderedScalars) {
      if (scalarFileName.empty())
        return false;

      std::ifstream scalarFile(scalarFileName, std::ios::binary | std::ios::ate);
      if (!scalarFile.good())
        return false;

      size_t numBytes = scalarFile.tellg();
      scalarFile.close();

      scalarFile.open(scalarFileName, std::ios::binary);
      if (scalarFile.good()) {
        orderedScalars.resize(numBytes/sizeof(float));
        scalarFile.read((char *)orderedScalars.data(),orderedScalars.size()*sizeof(float));
        return true;
      }

      return false;
    };

    result->numFields = 0;
    if (loadScalars(scalarFileName0, orderedScalars[0])) result->numFields++;
    if (loadScalars(scalarFileName1, orderedScalars[1])) result->numFields++;
    if (loadScalars(scalarFileName2, orderedScalars[2])) result->numFields++;
    if (loadScalars(scalarFileName3, orderedScalars[3])) result->numFields++;
    if (loadScalars(scalarFileName4, orderedScalars[4])) result->numFields++;
    if (loadScalars(scalarFileName5, orderedScalars[5])) result->numFields++;
    if (loadScalars(scalarFileName6, orderedScalars[6])) result->numFields++;
    if (loadScalars(scalarFileName7, orderedScalars[7])) result->numFields++;

    // -------------------------------------------------------
    // create brick and index buffers
    // -------------------------------------------------------

    std::ifstream in(brickFileName, std::ios::binary);
    if (!in.good()) return result;
    while (!in.eof()) {
      ExaBrick brick;
      in.read((char*)&brick.size,sizeof(brick.size));
      in.read((char*)&brick.lower,sizeof(brick.lower));
      in.read((char*)&brick.level,sizeof(brick.level));
      brick.begin = (int)indices.size();
      if (!in.good())
        break;
      std::vector<int> cellIDs(owl::volume(brick.size));
      in.read((char*)cellIDs.data(),cellIDs.size()*sizeof(cellIDs[0]));
      indices.insert(indices.end(),cellIDs.begin(),cellIDs.end());
      bricks.push_back(brick);
    }
    std::cout << "#exa: done loading exabricks, found "
              << owl::prettyDouble((double)bricks.size()) << " bricks with "
              << owl::prettyDouble((double)indices.size()) << " cells" << std::endl;

    // -------------------------------------------------------
    // flatten cellIDs
    // -------------------------------------------------------

    assert(!orderedScalars[0].empty());
    size_t numScalars = orderedScalars[0].size();
    size_t numScalarsTotal = 0;
    for (size_t s=0; s<MAX_FIELDS; ++s)
      numScalarsTotal += orderedScalars[s].size();

    scalars.resize(numScalarsTotal);
    for (size_t s=0; MAX_FIELDS; ++s) {
      if (orderedScalars[s].empty()) break;
      parallel_for_blocked(0ull,indices.size(),1024*1024,[&](size_t begin,size_t end){
          for (size_t i=begin;i<end;i++) {
            if (indices[i] < 0) {
              throw std::runtime_error("overflow in index vector...");
            } else {
              int cellID = indices[i];
              if (cellID < 0)
                throw std::runtime_error("negative cell ID");
              if (cellID >= numScalars)
                throw std::runtime_error("invalid cell ID");
              scalars[i+s*numScalars] = orderedScalars[s][cellID];
            }
          }
        });
    }

    result->numScalarsPerField = (unsigned)numScalars;

    // -------------------------------------------------------
    // create regions
    // -------------------------------------------------------

    abrs.buildFrom(bricks.data(),
                   bricks.size(),
                   scalars.data(),
                   result->numFields,
                   result->numScalarsPerField);


    // -------------------------------------------------------
    // Global cellBounds and valueRange
    // -------------------------------------------------------

    for (size_t i=0; i<bricks.size(); ++i) {
      const ExaBrick &brick = bricks[i];
      cellBounds.extend(brick.getBounds());
    }

    for (size_t i=0; i<abrs.value.size(); ++i) {
      const ABR &abr = abrs.value[i];
      valueRange.extend(abr.valueRange);
    }

    // -------------------------------------------------------
    // kd tree, if passed in the constructor
    // -------------------------------------------------------

    if (!kdTreeFileName.empty()) {
      result->kdtree = KDTree::load(kdTreeFileName);
      std::vector<box3f> leaves(bricks.size());
      for (uint64_t i = 0; i < bricks.size(); ++i) {
        leaves[i] = bricks[i].getBounds();
      }
      result->kdtree->setLeaves(leaves);
      result->kdtree->setModelBounds(cellBounds);
    }

    // -------------------------------------------------------
    // Adjacency list, in case we're traversing bricks
    // -------------------------------------------------------

    if (samplerMode == EXA_BRICK_SAMPLER_EXT_BVH ||
        traversalMode == EXABRICK_BVH_TRAVERSAL ||
        traversalMode == EXABRICK_EXT_BVH_TRAVERSAL ||
        traversalMode == EXABRICK_KDTREE_TRAVERSAL) {
      std::cout << "Building adjacent brick list...\n";
      adjacentBricks.resize(bricks.size());

      for (size_t i=0; i<abrs.value.size(); ++i) {
        const ABR &abr = abrs.value[i];
        for (int childID1=abr.leafListBegin;childID1<abr.leafListSize;childID1++) {
          for (int childID2=abr.leafListBegin;childID2<abr.leafListSize;childID2++) {
            if (childID1!=childID2) {
              const box3f d1 = bricks[childID1].getDomain();
              const box3f d2 = bricks[childID2].getDomain();

              if (d1.overlaps(d2)) {
                if (std::find(adjacentBricks[childID1].begin(),
                              adjacentBricks[childID1].end(),childID2)==adjacentBricks[childID1].end())
                  adjacentBricks[childID1].push_back(childID2);
                if (std::find(adjacentBricks[childID2].begin(),
                              adjacentBricks[childID2].end(),childID1)==adjacentBricks[childID2].end())
                  adjacentBricks[childID2].push_back(childID1);
              }
            }
          }
        }
      }

      // for (size_t i=0; i<bricks.size(); ++i) {
      //   for (size_t j=i+1; j<bricks.size(); ++j) {
      //     const box3f di = bricks[i].getDomain();
      //     const box3f dj = bricks[j].getDomain();
      //     if (di.overlaps(dj)) {
      //       if (std::find(adjacentBricks[i].begin(),adjacentBricks[i].end(),j)==adjacentBricks[i].end())
      //         adjacentBricks[i].push_back(j);
      //       if (std::find(adjacentBricks[j].begin(),adjacentBricks[j].end(),i)==adjacentBricks[j].end())
      //         adjacentBricks[j].push_back(i);
      //     }
      //   }
      // }
      std::cout << "Done\n";
    }

    return result;
  }

  void ExaBrickModel::memStats(size_t &bricksBytes,
                               size_t &scalarsBytes,
                               size_t &abrsBytes,
                               size_t &abrLeafListBytes)
  {
    bricksBytes = bricks.empty()   ? 0 : bricks.size()*sizeof(bricks[0]);
    scalarsBytes = scalars.empty() ? 0 : scalars.size()*sizeof(scalars[0]);
    abrsBytes = abrs.value.empty() ? 0 : abrs.value.size()*sizeof(abrs.value[0]);
    abrLeafListBytes = abrs.leafList.empty() ? 0 : abrs.leafList.size()*sizeof(abrs.leafList[0]);
  }

} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

