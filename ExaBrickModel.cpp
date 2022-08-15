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

    std::ifstream in(brickFileName);
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
    OWLVarDecl exaBrickGeomVars[]
    = {
       { "abrBuffer",  OWL_BUFPTR, OWL_OFFSETOF(ExaBrickGeom,abrBuffer)},
       { nullptr /* sentinel to mark end of list */ }
    };

    // ==================================================================
    // exa brick geom
    // ==================================================================

    if (!abrs.value.empty()) {
      geomType = owlGeomTypeCreate(context,
                                   OWL_GEOM_USER,
                                   sizeof(ExaBrickGeom),
                                   exaBrickGeomVars, -1);
      owlGeomTypeSetBoundsProg(geomType, module, "ExaBrickGeomBounds");
      owlGeomTypeSetIntersectProg(geomType, 0, module, "ExaBrickGeomIsect");
      owlGeomTypeSetClosestHit(geomType, 0, module, "ExaBrickGeomCH");

      OWLGeom geom = owlGeomCreate(context, geomType);
      owlGeomSetPrimCount(geom, abrs.value.size());

      abrBuffer = owlDeviceBufferCreate(context, OWL_USER_TYPE(ABR),
                                        abrs.value.size(),
                                        abrs.value.data());

      brickBuffer = owlDeviceBufferCreate(context, OWL_USER_TYPE(ExaBrick),
                                          bricks.size(),
                                          bricks.data());

      scalarBuffer = owlDeviceBufferCreate(context, OWL_FLOAT,
                                           scalars.size(),
                                           scalars.data());

      abrLeafListBuffer = owlDeviceBufferCreate(context, OWL_FLOAT,
                                                abrs.leafList.size(),
                                                abrs.leafList.data());

      owlGeomSetBuffer(geom,"abrBuffer",abrBuffer);

      owlBuildPrograms(context);

      blas = owlUserGeomGroupCreate(context, 1, &geom);
      owlGroupBuildAccel(blas);

      tlas = owlInstanceGroupCreate(context, 1);
      owlInstanceGroupSetChild(tlas, 0, blas);

      owlGroupBuildAccel(tlas);
      return true;
    }

    return false;
  }

  void ExaBrickModel::memStats(size_t &bricksBytes,
                               size_t &scalarsBytes,
                               size_t &abrsBytes)
  {
    bricksBytes = bricks.empty()   ? 0 : bricks.size()*sizeof(bricks[0]);
    scalarsBytes = scalars.empty() ? 0 : scalars.size()*sizeof(scalars[0]);
    abrsBytes = abrs.value.empty() ? 0 : abrs.value.size()*sizeof(abrs.value[0]);
  }

} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

