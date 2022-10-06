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
    ExaBrickModel::SP result = std::make_shared<ExaBrickModel>();

    std::vector<ExaBrick> &bricks = result->bricks;
    std::vector<float> &scalars   = result->scalars;
    ABRs &abrs                    = result->abrs;
    box3f &cellBounds             = result->cellBounds;
    range1f &valueRange           = result->valueRange;
    auto &adjacentBricks          = result->adjacentBricks;


    // Indices/scalars are later flattened
    std::vector<float> orderedScalars;
    std::vector<int> indices;

    std::ifstream scalarFile(scalarFileName, std::ios::binary | std::ios::ate);
    if (scalarFile.good()) {
      size_t numBytes = scalarFile.tellg();
      scalarFile.close();
      scalarFile.open(scalarFileName, std::ios::binary);
      if (scalarFile.good()) {
        orderedScalars.resize(numBytes/sizeof(float));
        scalarFile.read((char *)orderedScalars.data(),orderedScalars.size()*sizeof(float));
      }
    }

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
              << owl::prettyDouble(bricks.size()) << " bricks with "
              << owl::prettyDouble(indices.size()) << " cells" << std::endl;

    // -------------------------------------------------------
    // flatten cellIDs
    // -------------------------------------------------------

    scalars.resize(orderedScalars.size());
    parallel_for_blocked(0ull,indices.size(),1024*1024,[&](size_t begin,size_t end){
        for (size_t i=begin;i<end;i++) {
          if (indices[i] < 0) {
            throw std::runtime_error("overflow in index vector...");
          } else {
            int cellID = indices[i];
            if (cellID < 0)
              throw std::runtime_error("negative cell ID");
            if (cellID >= orderedScalars.size())
              throw std::runtime_error("invalid cell ID");
            scalars[i] = orderedScalars[cellID];
          }
        }
      });

    // -------------------------------------------------------
    // create regions
    // -------------------------------------------------------

    abrs.buildFrom(bricks.data(),
                   bricks.size(),
                   scalars.data());


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

  bool ExaBrickModel::initGPU(OWLContext context, OWLModule module)
  {
    OWLVarDecl geomVars[]
    = {
       { "abrBuffer",  OWL_BUFPTR, OWL_OFFSETOF(ExaBrickGeom,abrBuffer)},
       { "exaBrickBuffer",  OWL_BUFPTR, OWL_OFFSETOF(ExaBrickGeom,exaBrickBuffer)},
       { "maxOpacities",  OWL_BUFPTR, OWL_OFFSETOF(ExaBrickGeom,maxOpacities)},
       { nullptr /* sentinel to mark end of list */ }
    };

    if (bricks.empty())
      return false;

    // ==================================================================
    // exa brick geom
    // ==================================================================

    brickBuffer       = owlDeviceBufferCreate(context, OWL_USER_TYPE(ExaBrick), bricks.size(), bricks.data());
    scalarBuffer      = owlDeviceBufferCreate(context, OWL_FLOAT, scalars.size(), scalars.data());

    if (traversalMode == EXABRICK_ABR_TRAVERSAL || samplerMode == EXA_BRICK_SAMPLER_ABR_BVH) {
      if (abrs.value.empty())
        return false;

      abrBuffer         = owlDeviceBufferCreate(context, OWL_USER_TYPE(ABR), abrs.value.size(), abrs.value.data());
      abrLeafListBuffer = owlDeviceBufferCreate(context, OWL_INT, abrs.leafList.size(), abrs.leafList.data());
      abrMaxOpacities   = owlDeviceBufferCreate(context, OWL_FLOAT, abrs.value.size(), nullptr);

      abrGeomType = owlGeomTypeCreate(context, OWL_GEOM_USER, sizeof(ExaBrickGeom), geomVars, -1);
      owlGeomTypeSetBoundsProg   (abrGeomType, module, "ExaBrickABRGeomBounds");
      owlGeomTypeSetIntersectProg(abrGeomType, RADIANCE_RAY_TYPE, module, "ExaBrickABRGeomIsect");
      owlGeomTypeSetClosestHit   (abrGeomType, RADIANCE_RAY_TYPE, module, "ExaBrickGeomCH");
      owlGeomTypeSetIntersectProg(abrGeomType, SAMPLING_RAY_TYPE, module, "ExaBrickABRGeomSamplingIsect");
      owlGeomTypeSetClosestHit   (abrGeomType, SAMPLING_RAY_TYPE, module, "ExaBrickGeomCH");
      OWLGeom abrGeom = owlGeomCreate(context, abrGeomType);
      owlGeomSetPrimCount(abrGeom, abrs.value.size());
      owlGeomSetBuffer(abrGeom,"abrBuffer", abrBuffer);
      owlGeomSetBuffer(abrGeom,"exaBrickBuffer", brickBuffer);
      owlGeomSetBuffer(abrGeom,"maxOpacities", abrMaxOpacities);

      owlBuildPrograms(context);

      abrBlas = owlUserGeomGroupCreate(context, 1, &abrGeom);
      owlGroupBuildAccel(abrBlas);
#ifdef EXA_STITCH_MIRROR_EXAJET
      abrTlas = owlInstanceGroupCreate(context, 2);
      owlInstanceGroupSetChild(abrTlas, 0, abrBlas);
      owlInstanceGroupSetChild(abrTlas, 1, abrBlas);
      owlInstanceGroupSetTransform(abrTlas, 1, &mirrorTransform);
#else
      abrTlas = owlInstanceGroupCreate(context, 1);
      owlInstanceGroupSetChild(abrTlas, 0, abrBlas);
#endif
      owlGroupBuildAccel(abrTlas);
    }

    if (samplerMode == EXA_BRICK_SAMPLER_EXT_BVH ) {
      // extended brick geometry //
      extGeomType = owlGeomTypeCreate(context, OWL_GEOM_USER, sizeof(ExaBrickGeom), geomVars, -1);
      owlGeomTypeSetBoundsProg   (extGeomType, module, "ExaBrickExtGeomBounds");
      owlGeomTypeSetIntersectProg(extGeomType, SAMPLING_RAY_TYPE, module, "ExaBrickExtGeomSamplingIsect");
      owlGeomTypeSetClosestHit   (extGeomType, SAMPLING_RAY_TYPE, module, "ExaBrickGeomCH");
      OWLGeom extGeom = owlGeomCreate(context, extGeomType);
      owlGeomSetPrimCount(extGeom, bricks.size());
      owlGeomSetBuffer(extGeom,"exaBrickBuffer", brickBuffer);
      if (!brickMaxOpacities)
        brickMaxOpacities = owlDeviceBufferCreate(context, OWL_FLOAT, bricks.size(), nullptr);
      owlGeomSetBuffer(extGeom,"maxOpacities", brickMaxOpacities);

      owlBuildPrograms(context);

      extBlas = owlUserGeomGroupCreate(context, 1, &extGeom);
      owlGroupBuildAccel(extBlas);
#ifdef EXA_STITCH_MIRROR_EXAJET
      extTlas = owlInstanceGroupCreate(context, 2);
      owlInstanceGroupSetChild(extTlas, 0, extBlas);
      owlInstanceGroupSetChild(extTlas, 1, extBlas);
      owlInstanceGroupSetTransform(extTlas, 1, &mirrorTransform);
#else
      extTlas = owlInstanceGroupCreate(context, 1);
      owlInstanceGroupSetChild(extTlas, 0, extBlas);
#endif
      owlGroupBuildAccel(extTlas);
    }

    if (traversalMode == EXABRICK_BVH_TRAVERSAL) {
      // brick geometry //
      brickGeomType = owlGeomTypeCreate(context, OWL_GEOM_USER, sizeof(ExaBrickGeom), geomVars, -1);
      owlGeomTypeSetBoundsProg   (brickGeomType, module, "ExaBrickBrickGeomBounds");
      owlGeomTypeSetIntersectProg(brickGeomType, RADIANCE_RAY_TYPE, module, "ExaBrickBrickGeomIsect");
      owlGeomTypeSetClosestHit   (brickGeomType, RADIANCE_RAY_TYPE, module, "ExaBrickGeomCH");
      OWLGeom brickGeom = owlGeomCreate(context, brickGeomType);
      owlGeomSetPrimCount(brickGeom, bricks.size());
      owlGeomSetBuffer(brickGeom,"exaBrickBuffer", brickBuffer);
      if (!brickMaxOpacities)
        brickMaxOpacities = owlDeviceBufferCreate(context, OWL_FLOAT, bricks.size(), nullptr);
      owlGeomSetBuffer(brickGeom,"maxOpacities", brickMaxOpacities);

      owlBuildPrograms(context);

      brickBlas = owlUserGeomGroupCreate(context, 1, &brickGeom);
      owlGroupBuildAccel(brickBlas);
#ifdef EXA_STITCH_MIRROR_EXAJET
      brickTlas = owlInstanceGroupCreate(context, 2);
      owlInstanceGroupSetChild(brickTlas, 0, brickBlas);
      owlInstanceGroupSetChild(brickTlas, 1, brickBlas);
      owlInstanceGroupSetTransform(brickTlas, 1, &mirrorTransform);
#else
      brickTlas = owlInstanceGroupCreate(context, 1);
      owlInstanceGroupSetChild(brickTlas, 0, brickBlas);
#endif
      owlGroupBuildAccel(brickTlas);
    } else if (traversalMode == EXABRICK_KDTREE_TRAVERSAL) {
      // build KD tree over exabricks
      if (!kdtree)
        return false;

      if (!brickMaxOpacities)
        brickMaxOpacities = owlDeviceBufferCreate(context, OWL_FLOAT, bricks.size(), nullptr);

      kdtree->initGPU();
#ifdef EXA_STITCH_MIRROR_EXAJET
      kdtree->deviceTraversable.mirrorInvTransform = rcp((const affine3f &)mirrorTransform);
      kdtree->deviceTraversable.mirrorPlane.axis = 1;
      kdtree->deviceTraversable.mirrorPlane.offset = cellBounds.upper.y;
#endif
    }

    if (traversalMode == MC_DDA_TRAVERSAL || traversalMode == MC_BVH_TRAVERSAL) {
      // build the grid
      if (!grid || grid->dims==vec3i(0))
        return false;

      grid->build(context,shared_from_this(),grid->dims,cellBounds);
      // build the BVH, for the "traverse grid with optix" mode
      // Also initializes the device traversable
      grid->initGPU(context,module);
#ifdef EXA_STITCH_MIRROR_EXAJET
      if (traversalMode == MC_DDA_TRAVERSAL) {
        grid->deviceTraversable.mirrorInvTransform = rcp((const affine3f &)mirrorTransform);
        grid->deviceTraversable.mirrorPlane.axis = 1;
        grid->deviceTraversable.mirrorPlane.offset = cellBounds.upper.y;
      } else if (traversalMode == MC_BVH_TRAVERSAL) {
        owlInstanceGroupSetTransform(grid->tlas, 1, &mirrorTransform);
        owlGroupBuildAccel(grid->tlas);
      }
#endif
    }


    if (samplerMode == EXA_BRICK_SAMPLER_EXT_BVH ||
        traversalMode == EXABRICK_BVH_TRAVERSAL ||
        traversalMode == EXABRICK_KDTREE_TRAVERSAL) {

      std::vector<range1f> hValueRanges(bricks.size());
      std::fill(hValueRanges.begin(),
                hValueRanges.end(),
                range1f{1e30f,-1e30f});

      for (size_t i=0; i<bricks.size(); ++i) {
        const ExaBrick &brick = bricks[i];
        for (int z=0; z<brick.size.z; ++z) {
          for (int y=0; y<brick.size.y; ++y) {
            for (int x=0; x<brick.size.x; ++x) {
              vec3i index3(x,y,z);
              int idx = brick.getIndexIndex(index3);
              const float value = scalars[idx];

              hValueRanges[i].lower = std::min(hValueRanges[i].lower,value);
              hValueRanges[i].upper = std::max(hValueRanges[i].upper,value);

              vec3i lower = brick.lower + index3*(1<<brick.level);
              vec3i upper = lower + (1<<brick.level);

              const vec3f halfCell = vec3f(1<<brick.level)*.5f;

              box3f domain(vec3f(lower)-halfCell,vec3f(upper)+halfCell);

              for (int j=0; j<adjacentBricks[i].size(); ++j) {
                const ExaBrick &adjacentBrick = bricks[j];
                if (domain.overlaps(adjacentBrick.getDomain())) {
                  hValueRanges[j].lower = std::min(hValueRanges[j].lower,value);
                  hValueRanges[j].upper = std::max(hValueRanges[j].upper,value);
                }
              }
            }
          }
        }

        std::cout << '(' << (i+1) << '/' << bricks.size() << ")\r";
      }

      owlBufferRelease(brickValueRanges);
      brickValueRanges = owlDeviceBufferCreate(context, OWL_USER_TYPE(range1f),
                                               hValueRanges.size(),
                                               hValueRanges.data());
    }

    initBaseModel();

    return true;
  }

  void ExaBrickModel::setNumGridCells(const vec3i numMCs)
  {
    if (grid != nullptr) {
      throw std::runtime_error("must set num grid cells before calling initGPU!");
    }

    grid = std::make_shared<Grid>();
    grid->dims = numMCs;
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

  void ExaBrickModel::initBaseModel()
  {
    Model::sampleBVH = NULL;
    Model::majorantAccel.bvh = NULL;
    Model::majorantAccel.grid = NULL;
    Model::majorantAccel.kdtree = NULL;

    // Set the sampling accel
    switch (samplerMode) {
      
      case EXA_BRICK_SAMPLER_ABR_BVH: {
        Model::sampleBVH = abrTlas;
        break;
      }

      case EXA_BRICK_SAMPLER_EXT_BVH: {
        Model::sampleBVH = extTlas;
        break;
      }
    }

    // Set the majorant traversal accel and majorants buffer
    switch (traversalMode) {
    
      case MC_DDA_TRAVERSAL: {
        Model::majorantAccel.grid = grid;
        Model::maxOpacities = grid->maxOpacities;
        break;
      }

      case MC_BVH_TRAVERSAL: {
        Model::majorantAccel.bvh = grid->tlas;
        Model::maxOpacities = grid->maxOpacities;
        break;
      }

      case EXABRICK_ABR_TRAVERSAL: {
        Model::majorantAccel.bvh = abrTlas;
        Model::maxOpacities = abrMaxOpacities;
        break;
      }

      case EXABRICK_BVH_TRAVERSAL: {
        Model::majorantAccel.bvh = brickTlas;
        Model::maxOpacities = brickMaxOpacities;
        break;
      }

      case EXABRICK_KDTREE_TRAVERSAL: {
        Model::majorantAccel.kdtree = kdtree;
        Model::maxOpacities = brickMaxOpacities;
        break;
      }

      default: throw std::runtime_error("wrong traversal mode?!");
        break;
    }
  }

} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

