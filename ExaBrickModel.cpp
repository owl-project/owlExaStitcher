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

    return result;
  }

  bool ExaBrickModel::initGPU(OWLContext context, OWLModule module)
  {
    OWLVarDecl geomVars[]
    = {
       { "abrBuffer",  OWL_BUFPTR, OWL_OFFSETOF(ExaBrickGeom,abrBuffer)},
       { "exaBrickBuffer",  OWL_BUFPTR, OWL_OFFSETOF(ExaBrickGeom,exaBrickBuffer)},
       { nullptr /* sentinel to mark end of list */ }
    };

    // ==================================================================
    // exa brick geom
    // ==================================================================

    if (!abrs.value.empty() && !bricks.empty()) {

      abrBuffer         = owlDeviceBufferCreate(context, OWL_USER_TYPE(ABR), abrs.value.size(), abrs.value.data());
      brickBuffer       = owlDeviceBufferCreate(context, OWL_USER_TYPE(ExaBrick), bricks.size(), bricks.data());
      scalarBuffer      = owlDeviceBufferCreate(context, OWL_FLOAT, scalars.size(), scalars.data());
      abrLeafListBuffer = owlDeviceBufferCreate(context, OWL_INT, abrs.leafList.size(), abrs.leafList.data());

#if EXA_STITCH_EXA_BRICK_SAMPLER_MODE == EXA_BRICK_SAMPLER_ABR_BVH || EXA_STITCH_EXA_BRICK_TRAVERSAL_MODE == EXABRICK_ABR_TRAVERSAL
      abrMaxOpacities   = owlDeviceBufferCreate(context, OWL_FLOAT, abrs.value.size(), nullptr);
#endif

#if EXA_STITCH_EXA_BRICK_TRAVERSAL_MODE == EXABRICK_BVH_TRAVERSAL || EXA_STITCH_EXA_BRICK_TRAVERSAL_MODE == EXABRICK_KDTREE_TRAVERSAL
      brickMaxOpacities = owlDeviceBufferCreate(context, OWL_FLOAT, bricks.size(), nullptr);
#endif

      // ABR geometry //
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

      // extended brick geometry //
      extGeomType = owlGeomTypeCreate(context, OWL_GEOM_USER, sizeof(ExaBrickGeom), geomVars, -1);
      owlGeomTypeSetBoundsProg   (extGeomType, module, "ExaBrickExtGeomBounds");
      owlGeomTypeSetIntersectProg(extGeomType, SAMPLING_RAY_TYPE, module, "ExaBrickExtGeomSamplingIsect");
      owlGeomTypeSetClosestHit   (extGeomType, SAMPLING_RAY_TYPE, module, "ExaBrickGeomCH");
      OWLGeom extGeom = owlGeomCreate(context, extGeomType);
      owlGeomSetPrimCount(extGeom, bricks.size());
      owlGeomSetBuffer(extGeom,"abrBuffer", abrBuffer);
      owlGeomSetBuffer(extGeom,"exaBrickBuffer", brickBuffer);

      // brick geometry //
      brickGeomType = owlGeomTypeCreate(context, OWL_GEOM_USER, sizeof(ExaBrickGeom), geomVars, -1);
      owlGeomTypeSetBoundsProg   (brickGeomType, module, "ExaBrickBrickGeomBounds");
      owlGeomTypeSetIntersectProg(brickGeomType, RADIANCE_RAY_TYPE, module, "ExaBrickBrickGeomIsect");
      owlGeomTypeSetClosestHit   (brickGeomType, RADIANCE_RAY_TYPE, module, "ExaBrickGeomCH");
      OWLGeom brickGeom = owlGeomCreate(context, brickGeomType);
      owlGeomSetPrimCount(brickGeom, bricks.size());
      owlGeomSetBuffer(brickGeom,"abrBuffer", abrBuffer);
      owlGeomSetBuffer(brickGeom,"exaBrickBuffer", brickBuffer);

      // finalize //
      owlBuildPrograms(context);

#if EXA_STITCH_EXA_BRICK_SAMPLER_MODE == EXA_BRICK_SAMPLER_ABR_BVH || EXA_STITCH_EXA_BRICK_TRAVERSAL_MODE == EXABRICK_ABR_TRAVERSAL
      // 1. ABR geometry 
      abrBlas = owlUserGeomGroupCreate(context, 1, &abrGeom);
      owlGroupBuildAccel(abrBlas);
#ifdef EXA_STITCH_MIRROR_EXAJET
      abrTlas = owlInstanceGroupCreate(context, 2);
#else
      abrTlas = owlInstanceGroupCreate(context, 1);
#endif
      owlInstanceGroupSetChild(abrTlas, 0, abrBlas);
#ifdef EXA_STITCH_MIRROR_EXAJET
      owlInstanceGroupSetChild(abrTlas, 1, abrBlas);
      owlInstanceGroupSetTransform(abrTlas, 1, &mirrorTransform);
#endif
      owlGroupBuildAccel(abrTlas);
#endif

#if EXA_STITCH_EXA_BRICK_SAMPLER_MODE == EXA_BRICK_SAMPLER_EXT_BVH
      // 2. extended brick geometry
      extBlas = owlUserGeomGroupCreate(context, 1, &extGeom);
      owlGroupBuildAccel(extBlas);
#ifdef EXA_STITCH_MIRROR_EXAJET
      extTlas = owlInstanceGroupCreate(context, 2);
#else
      extTlas = owlInstanceGroupCreate(context, 1);
#endif
      owlInstanceGroupSetChild(extTlas, 0, extBlas);
#ifdef EXA_STITCH_MIRROR_EXAJET
      owlInstanceGroupSetChild(extTlas, 1, extBlas);
      owlInstanceGroupSetTransform(extTlas, 1, &mirrorTransform);
#endif
      owlGroupBuildAccel(extTlas);
#endif

#if EXA_STITCH_EXA_BRICK_TRAVERSAL_MODE == EXABRICK_BVH_TRAVERSAL
      // 3. brick geometry
      brickBlas = owlUserGeomGroupCreate(context, 1, &brickGeom);
      owlGroupBuildAccel(brickBlas);
#ifdef EXA_STITCH_MIRROR_EXAJET
      brickTlas = owlInstanceGroupCreate(context, 2);
#else
      brickTlas = owlInstanceGroupCreate(context, 1);
#endif
      owlInstanceGroupSetChild(brickTlas, 0, brickBlas);
#ifdef EXA_STITCH_MIRROR_EXAJET
      owlInstanceGroupSetChild(brickTlas, 1, brickBlas);
      owlInstanceGroupSetTransform(brickTlas, 1, &mirrorTransform);
#endif
      owlGroupBuildAccel(brickTlas);
#endif

#if EXA_STITCH_EXA_BRICK_TRAVERSAL_MODE == EXABRICK_KDTREE_TRAVERSAL
      // 4. build KD tree over exabricks
      if (kdtree) {
        kdtree->initGPU();
#ifdef EXA_STITCH_MIRROR_EXAJET
        kdtree->deviceTraversable.mirrorInvTransform = rcp((const affine3f &)mirrorTransform);
        kdtree->deviceTraversable.mirrorPlane.axis = 1;
        kdtree->deviceTraversable.mirrorPlane.offset = cellBounds.upper.y;
#endif
      }
#endif

#if EXA_STITCH_EXA_BRICK_TRAVERSAL_MODE == MC_DDA_TRAVERSAL || EXA_STITCH_EXA_BRICK_TRAVERSAL_MODE == MC_BVH_TRAVERSAL
      // 5. build the grid
      if (grid && grid->dims != vec3i(0)) {
        grid->build(context,shared_from_this(),grid->dims,cellBounds);
        // build the BVH, for the "traverse grid with optix" mode
        // Also initializes the device traversable
        grid->initGPU(context,module);
#ifdef EXA_STITCH_MIRROR_EXAJET
#if EXA_STITCH_EXA_BRICK_TRAVERSAL_MODE == MC_DDA_TRAVERSAL
        grid->deviceTraversable.mirrorInvTransform = rcp((const affine3f &)mirrorTransform);
        grid->deviceTraversable.mirrorPlane.axis = 1;
        grid->deviceTraversable.mirrorPlane.offset = cellBounds.upper.y;
#elif EXA_STITCH_EXA_BRICK_TRAVERSAL_MODE == MC_BVH_TRAVERSAL
        owlInstanceGroupSetTransform(grid->tlas, 1, &mirrorTransform);
        owlGroupBuildAccel(grid->tlas);
#endif
#endif
      }
#endif

      initBaseModel();

      return true;
    }

    return false;
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
    switch (EXA_STITCH_EXA_BRICK_SAMPLER_MODE) {
      
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
    switch (EXA_STITCH_EXA_BRICK_TRAVERSAL_MODE) {
    
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

