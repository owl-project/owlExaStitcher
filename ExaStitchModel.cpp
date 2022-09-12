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
#include "umesh/UMesh.h"
#include "ExaStitchModel.h"

namespace exa {


  ExaStitchModel::SP ExaStitchModel::load(const std::string umeshFileName,
                                          const std::string gridsFileName,
                                          const std::string scalarFileName,
                                          const vec3f mirrorAxis)
  {
    ExaStitchModel::SP result = std::make_shared<ExaStitchModel>();

    std::vector<int> &indices          = result->indices;
    std::vector<vec4f> &vertices       = result->vertices;
    std::vector<Gridlet> &gridlets     = result->gridlets;
    std::vector<float> &gridletScalars = result->gridletScalars;
    box3f &cellBounds                  = result->cellBounds;
    range1f &valueRange                = result->valueRange;

    result->mirrorAxis = mirrorAxis;

    // Load scalars
    std::vector<float> scalars;

    std::ifstream scalarFile(scalarFileName, std::ios::binary | std::ios::ate);
    if (scalarFile.good()) {
      size_t numBytes = scalarFile.tellg();
      scalarFile.close();
      scalarFile.open(scalarFileName, std::ios::binary);
      if (scalarFile.good()) {
        scalars.resize(numBytes/sizeof(float));
        scalarFile.read((char *)scalars.data(),scalars.size()*sizeof(float));
      }
    }

    unsigned numElems = 0;

    if (!umeshFileName.empty()) {
      std::cout << "#mm: loading umesh from " << umeshFileName << std::endl;
      umesh::UMesh::SP mesh = umesh::UMesh::loadFrom(umeshFileName);
      std::cout << "#mm: got umesh w/ " << mesh->toString() << std::endl;

      vertices.resize(mesh->vertices.size());
      for (size_t i=0; i<mesh->vertices.size(); ++i) {
        float value = 0.f;
        if (!scalars.empty() && !mesh->vertexTag.empty())
          value = scalars[mesh->vertexTag[i]];
        else if (!mesh->perVertex->values.empty())
          value = mesh->perVertex->values[i];

        vertices[i] = vec4f(mesh->vertices[i].x,
                            mesh->vertices[i].y,
                            mesh->vertices[i].z,
                            value);
      }

      cellBounds = box3f();
      indices.resize(mesh->size()*8,-1);

      size_t elem = 0;

      valueRange = range1f(1e30f,-1e30f);

      // ==================================================================
      // Unstructured elems
      // ==================================================================

      auto buildIndices = [&](const auto &elems) {
        if (elems.empty())
          return;

        unsigned numVertices = elems[0].numVertices;
        for (size_t i=0; i<elems.size(); ++i) {
          for (size_t j=0; j<numVertices; ++j) {
            indices[elem*8+j] = elems[i][j];
            cellBounds.extend(vec3f(vertices[indices[elem*8+j]]));
            valueRange.lower = std::min(valueRange.lower,vertices[indices[elem*8+j]].w);
            valueRange.upper = std::max(valueRange.upper,vertices[indices[elem*8+j]].w);
          }
          elem++;
        }
      };

      buildIndices(mesh->tets);
      buildIndices(mesh->pyrs);
      buildIndices(mesh->wedges);
      buildIndices(mesh->hexes);

      numElems = indices.size()/8;

      std::cout << "Got " << numElems
                << " elements. Value range is: " << valueRange << '\n';
    }

    // ==================================================================
    // Compact unused vertices
    // ==================================================================

    if (!gridsFileName.empty()) {
      std::vector<int> uniqueIndices(vertices.size(),-1);
      std::map<int,int> newIndicesMap;

      for (size_t i=0; i<indices.size(); ++i) {
        if (indices[i] >= 0) {
          uniqueIndices[indices[i]] = indices[i];
        }
      }

      int newIndex = 0;
      for (size_t i=0; i<uniqueIndices.size(); ++i) {
        if (uniqueIndices[i] >= 0) {
          newIndicesMap.insert({uniqueIndices[i],newIndex++});
        }
      }

      std::vector<vec4f> newVertices(newIndicesMap.size());
      for (size_t i=0; i<uniqueIndices.size(); ++i) {
        if (uniqueIndices[i] >= 0) {
          newVertices[newIndicesMap[uniqueIndices[i]]] = vertices[uniqueIndices[i]];
        }
      }

      std::vector<int> newIndices(indices.size(),-1);
      for (size_t i=0; i<indices.size(); ++i) {
        if (indices[i] >= 0) {
          newIndices[i] = newIndicesMap[indices[i]];
        }
      }

      std::cout << "#verts before compaction: " <<vertices.size() << ' '
                << "#verts after compaction: " << newVertices.size() << '\n';

      vertices = newVertices;
      indices = newIndices;
    }

    // ==================================================================
    // Gridlets
    // ==================================================================

    size_t numEmptyTotal = 0;
    size_t numNonEmptyTotal = 0;
    gridletScalars.clear();
    if (!gridsFileName.empty()) {
      std::ifstream in(gridsFileName, std::ios::binary);
      while (!in.eof()) {
        Gridlet gridlet;
        in.read((char *)&gridlet.lower,sizeof(gridlet.lower));
        if (!in.good())
          break;
        in.read((char *)&gridlet.level,sizeof(gridlet.level));
        in.read((char *)&gridlet.dims,sizeof(gridlet.dims));

        size_t numScalars = (gridlet.dims.x+1)
                    * (size_t(gridlet.dims.y)+1)
                          * (gridlet.dims.z+1);
        std::vector<int> scalarIDs(numScalars);
        in.read((char *)scalarIDs.data(),scalarIDs.size()*sizeof(scalarIDs[0]));

        std::vector<float> gscalars(scalarIDs.size());

        size_t numEmpty = 0;
        size_t numNonEmpty = 0;
        for (size_t i=0; i<scalarIDs.size(); ++i) {
          int scalarID = scalarIDs[i];

          float value = 0.f;
          if ((unsigned)scalarID < scalars.size()) {
            value = scalars[scalarID];
            numNonEmpty++;
          } else {
            value = NAN;
            numEmpty++;
          }
          gscalars[i] = value;

          valueRange.lower = std::min(valueRange.lower,value);
          valueRange.upper = std::max(valueRange.upper,value);
        }

        gridlet.begin = gridletScalars.size();

        gridletScalars.insert(gridletScalars.end(),
                              gscalars.begin(),
                              gscalars.end());

        numEmptyTotal += numEmpty;
        numNonEmptyTotal += numNonEmpty;

        // std::cout << '(' << numEmpty << '/' << scalarIDs.size() << ") empty\n";


        gridlets.push_back(gridlet);

        const box3f bounds = gridlet.getBounds();

        cellBounds.extend(bounds.lower);
        cellBounds.extend(bounds.upper);
      }
    }

    result->numScalarsTotal = numEmptyTotal+numNonEmptyTotal;
    result->numEmptyScalars = numEmptyTotal;

    std::cout << "Got " << gridlets.size()
              << " gridlets with " << result->numScalarsTotal
              << " scalars total. Value range is: " << valueRange << '\n';

    return result; 
  }

  bool ExaStitchModel::initGPU(OWLContext context, OWLModule module)
  {
    OWLVarDecl gridletGeomVars[]
    = {
       { "gridletBuffer",  OWL_BUFPTR, OWL_OFFSETOF(GridletGeom,gridletBuffer)},
       { "gridletScalarBuffer",  OWL_BUFPTR, OWL_OFFSETOF(GridletGeom,gridletScalarBuffer)},
       { nullptr /* sentinel to mark end of list */ }
    };

    OWLVarDecl stitchGeomVars[]
    = {
       { "indexBuffer",  OWL_BUFPTR, OWL_OFFSETOF(StitchGeom,indexBuffer)},
       { "vertexBuffer",  OWL_BUFPTR, OWL_OFFSETOF(StitchGeom,vertexBuffer)},
       { nullptr /* sentinel to mark end of list */ }
    };

    // ==================================================================
    // gridlet geom
    // ==================================================================

    if (!gridlets.empty()) {
      gridletGeom.geomType = owlGeomTypeCreate(context,
                                               OWL_GEOM_USER,
                                               sizeof(GridletGeom),
                                               gridletGeomVars, -1);
      owlGeomTypeSetBoundsProg(gridletGeom.geomType, module, "GridletGeomBounds");
      owlGeomTypeSetIntersectProg(gridletGeom.geomType,
                                  SAMPLING_RAY_TYPE,
                                  module,
                                  "GridletGeomIsect");
      owlGeomTypeSetClosestHit(gridletGeom.geomType,
                               SAMPLING_RAY_TYPE,
                               module,
                               "GridletGeomCH");

      OWLGeom geom = owlGeomCreate(context, gridletGeom.geomType);
      owlGeomSetPrimCount(geom, gridlets.size());

      gridletBuffer = owlDeviceBufferCreate(context, OWL_USER_TYPE(Gridlet),
                                            gridlets.size(),
                                            gridlets.data());

      gridletScalarBuffer = owlDeviceBufferCreate(context, OWL_FLOAT,
                                                  gridletScalars.size(),
                                                  gridletScalars.data());

      owlGeomSetBuffer(geom,"gridletBuffer",gridletBuffer);
      owlGeomSetBuffer(geom,"gridletScalarBuffer",gridletScalarBuffer);

      owlBuildPrograms(context);

      gridletGeom.blas = owlUserGeomGroupCreate(context, 1, &geom);
      owlGroupBuildAccel(gridletGeom.blas);
    }

    // ==================================================================
    // stitching geom
    // ==================================================================

    if (!vertices.empty() && !indices.empty()) {
      stitchGeom.geomType = owlGeomTypeCreate(context,
                                              OWL_GEOM_USER,
                                              sizeof(StitchGeom),
                                              stitchGeomVars, -1);
      owlGeomTypeSetBoundsProg(stitchGeom.geomType, module, "StitchGeomBounds");
      owlGeomTypeSetIntersectProg(stitchGeom.geomType,
                                  SAMPLING_RAY_TYPE,
                                  module,
                                  "StitchGeomIsect");
      owlGeomTypeSetClosestHit(stitchGeom.geomType,
                               SAMPLING_RAY_TYPE,
                               module,
                               "StitchGeomCH");

      OWLGeom geom = owlGeomCreate(context, stitchGeom.geomType);
      owlGeomSetPrimCount(geom, indices.size()/8);

      vertexBuffer = owlDeviceBufferCreate(context, OWL_FLOAT4,
                                           vertices.size(),
                                           vertices.data());

      indexBuffer = owlDeviceBufferCreate(context, OWL_INT,
                                          indices.size(),
                                          indices.data());

      owlGeomSetBuffer(geom,"vertexBuffer",vertexBuffer);
      owlGeomSetBuffer(geom,"indexBuffer",indexBuffer);

      owlBuildPrograms(context);

      stitchGeom.blas = owlUserGeomGroupCreate(context, 1, &geom);
      owlGroupBuildAccel(stitchGeom.blas);
    }

    const vec3f mirrorTransform = vec3f(mirrorAxis.x == 0.f ? 1.f : -mirrorAxis.x,
                                    mirrorAxis.y == 0.f ? 1.f : -mirrorAxis.y,
                                    mirrorAxis.z == 0.f ? 1.f : -mirrorAxis.z);
                                    
    affine3f transform =
            affine3f::translate(cellBounds.upper * mirrorAxis)
            * affine3f::scale(mirrorTransform)
            * affine3f::translate(-cellBounds.upper * mirrorAxis);

    owl4x3f tfm;
    tfm.t = owl3f{0.f, transform.p.y, 0.f};
    tfm.vx = owl3f{1.f, 0.f, 0.f};
    tfm.vy = owl3f{0.f, transform.l.vy.y, 0.f};
    tfm.vz = owl3f{0.f, 0.f, 1.f};

    bool mirror =  mirrorAxis.x != 0.0f || 
                  mirrorAxis.y != 0.0f ||
                  mirrorAxis.z != 0.0f;

    if (gridletGeom.blas && stitchGeom.blas) {
      tlas = owlInstanceGroupCreate(context, mirror ? 4 : 2);
      owlInstanceGroupSetChild(tlas, 0, gridletGeom.blas);
      owlInstanceGroupSetChild(tlas, 1, stitchGeom.blas);
      if(mirror)
      {
        owlInstanceGroupSetChild(tlas, 2, gridletGeom.blas);
        owlInstanceGroupSetChild(tlas, 3, stitchGeom.blas);
        owlInstanceGroupSetTransform(tlas, 2, &tfm);
        owlInstanceGroupSetTransform(tlas, 3, &tfm);
      }
    } else if (gridletGeom.blas) {
      tlas = owlInstanceGroupCreate(context, mirror ? 2 : 1);
      owlInstanceGroupSetChild(tlas, 0, gridletGeom.blas);
      if(mirror)
      {
        owlInstanceGroupSetChild(tlas, 1, gridletGeom.blas);
        owlInstanceGroupSetTransform(tlas, 1, &tfm);
      }
    } else if (stitchGeom.blas) {
      tlas = owlInstanceGroupCreate(context, mirror ? 2 : 1);
      owlInstanceGroupSetChild(tlas, 0, stitchGeom.blas);
      if(mirror)
      {
        owlInstanceGroupSetChild(tlas, 1, stitchGeom.blas);
        owlInstanceGroupSetTransform(tlas, 1, &tfm);
      }
    } else {
      return false;
    }

    //Extend bounds to mirrored model
    if(mirror)
      cellBounds.extend(
        (cellBounds.upper + abs(cellBounds.upper - cellBounds.lower)) * mirrorAxis);

    owlGroupBuildAccel(tlas);

    // All tests passed => success
    return true;
  }

  void ExaStitchModel::memStats(size_t &elemVertexBytes,
                                size_t &elemIndexBytes,
                                size_t &gridletBytes,
                                size_t &emptyScalarsBytes,
                                size_t &nonEmptyScalarsBytes)
  {
    elemVertexBytes = vertices.empty() ? 0 : vertices.size()*sizeof(vertices[0]);
    elemIndexBytes = indices.empty()   ? 0 : indices.size()*sizeof(indices[0]);
    gridletBytes = gridlets.empty()    ? 0 : gridlets.size()*sizeof(gridlets[0]);
    emptyScalarsBytes = numEmptyScalars*sizeof(float);
    nonEmptyScalarsBytes = (numScalarsTotal-numEmptyScalars)*sizeof(float);

  }

} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

