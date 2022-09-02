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
                                        const std::string scalarFileName)
  {
    ExaBrickModel::SP result = std::make_shared<ExaBrickModel>();

    std::vector<ExaBrick> &bricks = result->bricks;
    std::vector<float> &scalars   = result->scalars;
    ABRs &abrs                    = result->abrs;
    box3f &modelBounds            = result->modelBounds;
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
    // Global modelBounds and valueRange
    // -------------------------------------------------------

    for (size_t i=0; i<bricks.size(); ++i) {
      const ExaBrick &brick = bricks[i];
      modelBounds.extend(brick.getBounds());
    }

    for (size_t i=0; i<abrs.value.size(); ++i) {
      const ABR &abr = abrs.value[i];
      valueRange.extend(abr.valueRange);
    }

    return result;
  }

  bool ExaBrickModel::initGPU(OWLContext context, OWLModule module)
  {
    OWLVarDecl abrGeomVars[]
    = {
       { "abrBuffer",  OWL_BUFPTR, OWL_OFFSETOF(ExaBrickABRGeom,abrBuffer)},
       { nullptr /* sentinel to mark end of list */ }
    };

    OWLVarDecl extGeomVars[]
    = {
       { "exaBrickBuffer",  OWL_BUFPTR, OWL_OFFSETOF(ExaBrickExtGeom,exaBrickBuffer)},
       { nullptr /* sentinel to mark end of list */ }
    };

    // ==================================================================
    // exa brick geom
    // ==================================================================

    if (!abrs.value.empty() && !bricks.empty()) {
      // ABR geometry //

      abrGeomType = owlGeomTypeCreate(context,
                                   OWL_GEOM_USER,
                                   sizeof(ExaBrickABRGeom),
                                   abrGeomVars, -1);
      owlGeomTypeSetBoundsProg(abrGeomType, module, "ExaBrickABRGeomBounds");
      owlGeomTypeSetIntersectProg(abrGeomType,
                                  RADIANCE_RAY_TYPE,
                                  module,
                                  "ExaBrickABRGeomIsect");
      owlGeomTypeSetClosestHit(abrGeomType,
                               RADIANCE_RAY_TYPE,
                               module,
                               "ExaBrickGeomCH");
      owlGeomTypeSetIntersectProg(abrGeomType,
                                  SAMPLING_RAY_TYPE,
                                  module,
                                  "ExaBrickABRGeomSamplingIsect");
      owlGeomTypeSetClosestHit(abrGeomType,
                               SAMPLING_RAY_TYPE,
                               module,
                               "ExaBrickGeomCH");

      OWLGeom abrGeom = owlGeomCreate(context, abrGeomType);
      owlGeomSetPrimCount(abrGeom, abrs.value.size());

      // extended brick geometry //

      extGeomType = owlGeomTypeCreate(context,
                                   OWL_GEOM_USER,
                                   sizeof(ExaBrickExtGeom),
                                   extGeomVars, -1);
      owlGeomTypeSetBoundsProg(extGeomType, module, "ExaBrickExtGeomBounds");
      owlGeomTypeSetIntersectProg(extGeomType,
                                  RADIANCE_RAY_TYPE,
                                  module,
                                  "ExaBrickExtGeomIsect");
      owlGeomTypeSetClosestHit(extGeomType,
                               RADIANCE_RAY_TYPE,
                               module,
                               "ExaBrickGeomCH");
      owlGeomTypeSetIntersectProg(extGeomType,
                                  SAMPLING_RAY_TYPE,
                                  module,
                                  "ExaBrickExtGeomSamplingIsect");
      owlGeomTypeSetClosestHit(extGeomType,
                               SAMPLING_RAY_TYPE,
                               module,
                               "ExaBrickGeomCH");

      OWLGeom extGeom = owlGeomCreate(context, extGeomType);
      owlGeomSetPrimCount(extGeom, bricks.size());

      // finalize //

      abrBuffer = owlDeviceBufferCreate(context, OWL_USER_TYPE(ABR),
                                        abrs.value.size(),
                                        abrs.value.data());

      brickBuffer = owlDeviceBufferCreate(context, OWL_USER_TYPE(ExaBrick),
                                          bricks.size(),
                                          bricks.data());

      scalarBuffer = owlDeviceBufferCreate(context, OWL_FLOAT,
                                           scalars.size(),
                                           scalars.data());

      abrLeafListBuffer = owlDeviceBufferCreate(context, OWL_INT,
                                                abrs.leafList.size(),
                                                abrs.leafList.data());

      abrMaxOpacities = owlDeviceBufferCreate(context, OWL_FLOAT, abrs.value.size(), nullptr);
      brickMaxOpacities = owlDeviceBufferCreate(context, OWL_FLOAT, bricks.size(), nullptr);

      owlGeomSetBuffer(abrGeom,"abrBuffer",abrBuffer);
      owlGeomSetBuffer(extGeom,"exaBrickBuffer",brickBuffer);
      owlBuildPrograms(context);

      // 1. ABR geometry 
      abrBlas = owlUserGeomGroupCreate(context, 1, &abrGeom);
      owlGroupBuildAccel(abrBlas);
      abrTlas = owlInstanceGroupCreate(context, 1);
      owlInstanceGroupSetChild(abrTlas, 0, abrBlas);
      owlGroupBuildAccel(abrTlas);

      // 2. extended brick geometry
      extBlas = owlUserGeomGroupCreate(context, 1, &extGeom);
      owlGroupBuildAccel(extBlas);
      extTlas = owlInstanceGroupCreate(context, 1);
      owlInstanceGroupSetChild(extTlas, 0, extBlas);
      owlGroupBuildAccel(extTlas);

      // 3. build KD tree over exabricks
      // std::vector<box3f> bounds(bricks.size());
      // owl::parallel_for(bricks.size(), [&](size_t i) {
      //   bounds[i] = bricks[i].getDomain();
      // });
      // kdtree = KDTree::build(bounds.size(),bounds.data());
      // if (!kdtree->initGPU()) throw std::runtime_error("kd-tree failed");

      return true;
    }

    return false;
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

