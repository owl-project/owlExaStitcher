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
#include "OWLRenderer.h"

extern "C" char embedded_deviceCode[];

namespace exa {

  OWLVarDecl rayGenVars[]
  = {
     { nullptr /* sentinel to mark end of list */ }
  };

  OWLVarDecl gridletGeomVars[]
  = {
     { "gridletBuffer",  OWL_BUFPTR, OWL_OFFSETOF(GridletGeom,gridletBuffer)},
     { nullptr /* sentinel to mark end of list */ }
  };

  OWLVarDecl stitchGeomVars[]
  = {
     { "indexBuffer",  OWL_BUFPTR, OWL_OFFSETOF(StitchGeom,indexBuffer)},
     { "vertexBuffer",  OWL_BUFPTR, OWL_OFFSETOF(StitchGeom,vertexBuffer)},
     { "bounds.lower",  OWL_FLOAT3, OWL_OFFSETOF(StitchGeom,bounds.lower)},
     { "bounds.upper",  OWL_FLOAT3, OWL_OFFSETOF(StitchGeom,bounds.upper)},
     { nullptr /* sentinel to mark end of list */ }
  };

  OWLVarDecl amrCellGeomVars[]
  = {
     { "amrCellBuffer",  OWL_BUFPTR, OWL_OFFSETOF(AMRCellGeom,amrCellBuffer)},
     { "scalarBuffer",  OWL_BUFPTR, OWL_OFFSETOF(AMRCellGeom,scalarBuffer)},
     { nullptr /* sentinel to mark end of list */ }
  };

  OWLVarDecl launchParamsVars[]
  = {
     { "fbPointer",   OWL_RAW_POINTER, OWL_OFFSETOF(LaunchParams,fbPointer) },
     { "fbDepth",   OWL_BUFPTR, OWL_OFFSETOF(LaunchParams,fbDepth) },
     { "accumBuffer",   OWL_BUFPTR, OWL_OFFSETOF(LaunchParams,accumBuffer) },
     { "accumID",   OWL_INT, OWL_OFFSETOF(LaunchParams,accumID) },
     { "gridletBVH",    OWL_GROUP,  OWL_OFFSETOF(LaunchParams,gridletBVH)},
     { "boundaryCellBVH",    OWL_GROUP,  OWL_OFFSETOF(LaunchParams,boundaryCellBVH)},
     { "amrCellBVH",    OWL_GROUP,  OWL_OFFSETOF(LaunchParams,amrCellBVH)},
     { "modelBounds.lower",  OWL_FLOAT3, OWL_OFFSETOF(LaunchParams,modelBounds.lower)},
     { "modelBounds.upper",  OWL_FLOAT3, OWL_OFFSETOF(LaunchParams,modelBounds.upper)},
     // xf data
     { "transferFunc.domain",OWL_FLOAT2, OWL_OFFSETOF(LaunchParams,transferFunc.domain) },
     { "transferFunc.texture",   OWL_USER_TYPE(cudaTextureObject_t),OWL_OFFSETOF(LaunchParams,transferFunc.texture) },
     { "transferFunc.opacityScale", OWL_FLOAT, OWL_OFFSETOF(LaunchParams,transferFunc.opacityScale) },
     // render settings
     { "render.dt",           OWL_FLOAT,   OWL_OFFSETOF(LaunchParams,render.dt) },
     { "render.spp",           OWL_INT,   OWL_OFFSETOF(LaunchParams,render.spp) },
     { "render.heatMapEnabled", OWL_INT, OWL_OFFSETOF(LaunchParams,render.heatMapEnabled) },
     { "render.heatMapScale", OWL_FLOAT, OWL_OFFSETOF(LaunchParams,render.heatMapScale) },
     // grid for DDA/spatially varying majorants
     { "grid.dims",     OWL_INT3,   OWL_OFFSETOF(LaunchParams,grid.dims) },
     { "grid.valueRanges", OWL_BUFPTR, OWL_OFFSETOF(LaunchParams,grid.valueRanges) },
     { "grid.maxOpacities", OWL_BUFPTR, OWL_OFFSETOF(LaunchParams,grid.maxOpacities) },
     // clip planes
     { "clipPlane0.enabled",     OWL_INT,   OWL_OFFSETOF(LaunchParams,clipPlanes[0].enabled) },
     { "clipPlane0.N",     OWL_FLOAT3,   OWL_OFFSETOF(LaunchParams,clipPlanes[0].N) },
     { "clipPlane0.d",     OWL_FLOAT,   OWL_OFFSETOF(LaunchParams,clipPlanes[0].d) },
     { "clipPlane1.enabled",     OWL_INT,   OWL_OFFSETOF(LaunchParams,clipPlanes[1].enabled) },
     { "clipPlane1.N",     OWL_FLOAT3,   OWL_OFFSETOF(LaunchParams,clipPlanes[1].N) },
     { "clipPlane1.d",     OWL_FLOAT,   OWL_OFFSETOF(LaunchParams,clipPlanes[1].d) },
     // camera settings
     { "camera.org",    OWL_FLOAT3, OWL_OFFSETOF(LaunchParams,camera.org) },
     { "camera.dir_00", OWL_FLOAT3, OWL_OFFSETOF(LaunchParams,camera.dir_00) },
     { "camera.dir_du", OWL_FLOAT3, OWL_OFFSETOF(LaunchParams,camera.dir_du) },
     { "camera.dir_dv", OWL_FLOAT3, OWL_OFFSETOF(LaunchParams,camera.dir_dv) },
     { nullptr /* sentinel to mark end of list */ }
  };
  
  // ==================================================================
  // Renderer class
  // ==================================================================

  OWLRenderer::OWLRenderer(const std::string inFileName,
                           const std::string gridsFileName,
                           const std::string amrCellFileName,
                           const std::string scalarFileName)
  {
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
    std::vector<vec4f> vertices;
    std::vector<int> indices;

    if (!inFileName.empty()) {
    std::cout << "#mm: loading umesh from " << inFileName << std::endl;
    umesh::UMesh::SP mesh = umesh::UMesh::loadFrom(inFileName);
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

    modelBounds = box3f();
    indices.resize(mesh->size()*8,-1);

    size_t elem = 0;

    valueRange = range1f(1e30f,-1e30f);

    // Unstructured elems
    auto buildIndices = [&](const auto &elems) {
      if (elems.empty())
        return;

      unsigned numVertices = elems[0].numVertices;
      for (size_t i=0; i<elems.size(); ++i) {
        for (size_t j=0; j<numVertices; ++j) {
          indices[elem*8+j] = elems[i][j];
          modelBounds.extend(vec3f(vertices[indices[elem*8+j]]));
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

    // Gridlets
    size_t numScalarsInGrids = 0;
    std::vector<Gridlet> gridlets;
    if (!gridsFileName.empty()) {
      std::ifstream in(gridsFileName);
      while (!in.eof()) {
        Gridlet gridlet;
        in.read((char *)&gridlet.lower,sizeof(gridlet.lower));
        in.read((char *)&gridlet.level,sizeof(gridlet.level));
        in.read((char *)&gridlet.dims,sizeof(gridlet.dims));

        size_t numScalars = (gridlet.dims.x+1)
                    * (size_t(gridlet.dims.y)+1)
                          * (gridlet.dims.z+1);
        std::vector<int> scalarIDs(numScalars);
        in.read((char *)scalarIDs.data(),scalarIDs.size()*sizeof(scalarIDs[0]));

        std::vector<float> gridScalars(scalarIDs.size());

        size_t numEmpty = 0;
        for (size_t i=0; i<scalarIDs.size(); ++i) {
          int scalarID = scalarIDs[i];

          float value = 0.f;
          if ((unsigned)scalarID < scalars.size())
            value = scalars[scalarID];
          else {
            value = NAN;
            numEmpty++;
          }
          gridScalars[i] = value;

          valueRange.lower = std::min(valueRange.lower,value);
          valueRange.upper = std::max(valueRange.upper,value);
        }

        // std::cout << '(' << numEmpty << '/' << scalarIDs.size() << ") empty\n";


        cudaMalloc(&gridlet.scalars,gridScalars.size()*sizeof(gridScalars[0]));
        cudaMemcpy(gridlet.scalars,gridScalars.data(),
                   gridScalars.size()*sizeof(gridScalars[0]),
                   cudaMemcpyHostToDevice);

        numScalarsInGrids += scalarIDs.size();
        gridlets.push_back(gridlet);

        vec3i lower = gridlet.lower * (1<<gridlet.level);
        vec3i upper = lower + gridlet.dims * (1<<gridlet.level);

        modelBounds.extend(vec3f(lower));
        modelBounds.extend(vec3f(upper));
      }
    }

    std::cout << "Got " << gridlets.size()
              << " gridlets with " << numScalarsInGrids
              << " scalars total. Value range is: " << valueRange << '\n';

    // AMR cells (for basis function comparison)
    std::vector<AMRCell> amrCells;

    std::ifstream amrCellFile(amrCellFileName, std::ios::binary | std::ios::ate);
    if (amrCellFile.good()) {
      size_t numBytes = amrCellFile.tellg();
      amrCellFile.close();
      amrCellFile.open(amrCellFileName, std::ios::binary);
      if (amrCellFile.good()) {
        amrCells.resize(numBytes/sizeof(AMRCell));
        amrCellFile.read((char *)amrCells.data(),amrCells.size()*sizeof(AMRCell));
      }
    }



    owl = owlContextCreate(nullptr,1);
    module = owlModuleCreate(owl,embedded_deviceCode);
    lp = owlParamsCreate(owl,sizeof(LaunchParams),launchParamsVars,-1);
    rayGen = owlRayGenCreate(owl,module,"renderFrame",sizeof(RayGen),rayGenVars,-1);

    // ----------------------------------------------------
    // gridlet geom
    // ----------------------------------------------------

    gridletGeom.geomType = owlGeomTypeCreate(owl,
                                             OWL_GEOM_USER,
                                             sizeof(GridletGeom),
                                             gridletGeomVars, -1);
    owlGeomTypeSetBoundsProg(gridletGeom.geomType, module, "GridletGeomBounds");
    owlGeomTypeSetIntersectProg(gridletGeom.geomType, 0, module, "GridletGeomIsect");
    owlGeomTypeSetClosestHit(gridletGeom.geomType, 0, module, "GridletGeomCH");

    OWLGeom ggeom = owlGeomCreate(owl, gridletGeom.geomType);
    owlGeomSetPrimCount(ggeom, gridlets.size());

    OWLBuffer gridletBuffer = owlDeviceBufferCreate(owl, OWL_USER_TYPE(Gridlet),
                                                    gridlets.size(),
                                                    gridlets.data());

    owlGeomSetBuffer(ggeom,"gridletBuffer",gridletBuffer);

    owlBuildPrograms(owl);

    gridletGeom.blas = owlUserGeomGroupCreate(owl, 1, &ggeom);
    owlGroupBuildAccel(gridletGeom.blas);

    gridletGeom.tlas = owlInstanceGroupCreate(owl, 1);
    owlInstanceGroupSetChild(gridletGeom.tlas, 0, gridletGeom.blas);

    owlGroupBuildAccel(gridletGeom.tlas);

    owlParamsSetGroup(lp, "gridletBVH", gridletGeom.tlas);

    // ----------------------------------------------------
    // stitching geom
    // ----------------------------------------------------

    stitchGeom.geomType = owlGeomTypeCreate(owl,
                                            OWL_GEOM_USER,
                                            sizeof(StitchGeom),
                                            stitchGeomVars, -1);
    owlGeomTypeSetBoundsProg(stitchGeom.geomType, module, "StitchGeomBounds");
    owlGeomTypeSetIntersectProg(stitchGeom.geomType, 0, module, "StitchGeomIsect");
    owlGeomTypeSetClosestHit(stitchGeom.geomType, 0, module, "StitchGeomCH");

    OWLGeom sgeom = owlGeomCreate(owl, stitchGeom.geomType);
    owlGeomSetPrimCount(sgeom, numElems);

    OWLBuffer vertexBuffer = owlDeviceBufferCreate(owl, OWL_FLOAT4,
                                                   vertices.size(),
                                                   vertices.data());

    OWLBuffer indexBuffer = owlDeviceBufferCreate(owl, OWL_INT,
                                                  indices.size(),
                                                  indices.data());

    owlGeomSetBuffer(sgeom,"vertexBuffer",vertexBuffer);
    owlGeomSetBuffer(sgeom,"indexBuffer",indexBuffer);
    owlGeomSet3f(sgeom,"bounds.lower",
                 modelBounds.lower.x,
                 modelBounds.lower.y,
                 modelBounds.lower.z);
    owlGeomSet3f(sgeom,"bounds.upper",
                 modelBounds.upper.x,
                 modelBounds.upper.y,
                 modelBounds.upper.z);

    owlBuildPrograms(owl);

    stitchGeom.blas = owlUserGeomGroupCreate(owl, 1, &sgeom);
    owlGroupBuildAccel(stitchGeom.blas);

    stitchGeom.tlas = owlInstanceGroupCreate(owl, 1);
    owlInstanceGroupSetChild(stitchGeom.tlas, 0, stitchGeom.blas);

    owlGroupBuildAccel(stitchGeom.tlas);

    owlParamsSetGroup(lp, "boundaryCellBVH", stitchGeom.tlas);

    // ----------------------------------------------------
    // AMR cell geom (non-dual, for eval!)
    // ----------------------------------------------------

    OWLBuffer amrCellBuffer = 0, scalarBuffer = 0;
    if (!amrCells.empty()) {
      for (size_t i=0; i<amrCells.size(); ++i) {
        box3f bounds(vec3f(amrCells[i].pos),
                     vec3f(amrCells[i].pos+vec3i(1<<amrCells[i].level)));
        modelBounds.extend(bounds.lower);
        modelBounds.extend(bounds.upper);
        if (i < scalars.size())
          valueRange.extend(scalars[i]);
      }

      amrCellGeom.geomType = owlGeomTypeCreate(owl,
                                               OWL_GEOM_USER,
                                               sizeof(AMRCellGeom),
                                               amrCellGeomVars, -1);
      owlGeomTypeSetBoundsProg(amrCellGeom.geomType, module, "AMRCellGeomBounds");
      owlGeomTypeSetIntersectProg(amrCellGeom.geomType, 0, module, "AMRCellGeomIsect");
      owlGeomTypeSetClosestHit(amrCellGeom.geomType, 0, module, "AMRCellGeomCH");

      OWLGeom ageom = owlGeomCreate(owl, amrCellGeom.geomType);
      owlGeomSetPrimCount(ageom, amrCells.size());

      amrCellBuffer = owlDeviceBufferCreate(owl, OWL_USER_TYPE(AMRCell),
                                            amrCells.size(),
                                            amrCells.data());

      scalarBuffer = owlDeviceBufferCreate(owl, OWL_FLOAT,
                                           scalars.size(),
                                           scalars.data());

      owlGeomSetBuffer(ageom,"amrCellBuffer",amrCellBuffer);
      owlGeomSetBuffer(ageom,"scalarBuffer",scalarBuffer);

      owlBuildPrograms(owl);

      amrCellGeom.blas = owlUserGeomGroupCreate(owl, 1, &ageom);
      owlGroupBuildAccel(amrCellGeom.blas);

      amrCellGeom.tlas = owlInstanceGroupCreate(owl, 1);
      owlInstanceGroupSetChild(amrCellGeom.tlas, 0, amrCellGeom.blas);

      owlGroupBuildAccel(amrCellGeom.tlas);

      owlParamsSetGroup(lp, "amrCellBVH", amrCellGeom.tlas);
    }


    owlParamsSet3f(lp,"modelBounds.lower",
                   modelBounds.lower.x,
                   modelBounds.lower.y,
                   modelBounds.lower.z);
    owlParamsSet3f(lp,"modelBounds.upper",
                   modelBounds.upper.x,
                   modelBounds.upper.y,
                   modelBounds.upper.z);

    grid.build(owl,
               vertexBuffer,
               indexBuffer,
               gridletBuffer,
               amrCellBuffer,
               scalarBuffer,
               {1024,1024,256},
               modelBounds);

    setRange(valueRange);

    owlParamsSet3i(lp,"grid.dims",
                   grid.dims.x,
                   grid.dims.y,
                   grid.dims.z);
    owlParamsSetBuffer(lp,"grid.valueRanges",grid.valueRanges);

    for (int i=0; i<CLIP_PLANES_MAX; ++i) {
      setClipPlane(i,false,vec3f{0,0,1},modelBounds.center().z);
    }

    owlBuildPipeline(owl);
    owlBuildSBT(owl);
  }

  OWLRenderer::~OWLRenderer()
  {
  }

  void OWLRenderer::setCamera(const vec3f &org,
                              const vec3f &dir_00,
                              const vec3f &dir_du,
                              const vec3f &dir_dv)
  {
    owlParamsSet3f(lp,"camera.org",   org.x,org.y,org.z);
    owlParamsSet3f(lp,"camera.dir_00",dir_00.x,dir_00.y,dir_00.z);
    owlParamsSet3f(lp,"camera.dir_du",dir_du.x,dir_du.y,dir_du.z);
    owlParamsSet3f(lp,"camera.dir_dv",dir_dv.x,dir_dv.y,dir_dv.z);
  }

  void OWLRenderer::resize(const vec2i &newSize)
  {
    if (newSize != this->fbSize) {
      if (!accumBuffer)
        accumBuffer = owlDeviceBufferCreate(owl,OWL_FLOAT4,1,nullptr);
      owlBufferResize(accumBuffer,newSize.x*newSize.y);
      owlParamsSetBuffer(lp,"accumBuffer",accumBuffer);
      this->fbSize = newSize;
    }
  }

  void OWLRenderer::render(uint32_t *fbPointer)
  {
    owlParamsSetPointer(lp,"fbPointer",fbPointer);

    owlParamsSet1i(lp,"accumID",accumID);
    accumID++;
    owlParamsSet1f(lp,"render.dt",2.f);
    owlParamsSet1i(lp,"render.spp",max(spp,1));
    owlParamsSet1i(lp,"render.heatMapEnabled",heatMapEnabled);
    owlParamsSet1f(lp,"render.heatMapScale",heatMapScale);

    owlLaunch2D(rayGen,fbSize.x,fbSize.y,lp);
  }

  void OWLRenderer::setColorMap(const std::vector<vec4f> &newCM)
  {
    xf.colorMap = newCM;

    if (!xf.colorMapBuffer)
      xf.colorMapBuffer = owlDeviceBufferCreate(owl,OWL_FLOAT4,
                                                newCM.size(),nullptr);

    owlBufferUpload(xf.colorMapBuffer,newCM.data());

    range1f r{
     xf.absDomain.lower + (xf.relDomain.lower/100.f) * (xf.absDomain.upper-xf.absDomain.lower),
     xf.absDomain.lower + (xf.relDomain.upper/100.f) * (xf.absDomain.upper-xf.absDomain.lower)
    };
    grid.computeMaxOpacities(owl,xf.colorMapBuffer,r);
    owlParamsSetBuffer(lp,"grid.maxOpacities",grid.maxOpacities);
    
    if (xf.colorMapTexture != 0) {
      cudaDestroyTextureObject(xf.colorMapTexture);
      xf.colorMapTexture = 0;
    }

    cudaResourceDesc res_desc = {};
    cudaChannelFormatDesc channel_desc
      = cudaCreateChannelDesc<float4>();

    // cudaArray_t   voxelArray;
    if (xf.colorMapArray == 0) {
      cudaMallocArray(&xf.colorMapArray,
                      &channel_desc,
                      newCM.size(),1);
    }
    
    int pitch = newCM.size()*sizeof(newCM[0]);
    cudaMemcpy2DToArray(xf.colorMapArray,
                        /* offset */0,0,
                        newCM.data(),
                        pitch,pitch,1,
                        cudaMemcpyHostToDevice);

    res_desc.resType          = cudaResourceTypeArray;
    res_desc.res.array.array  = xf.colorMapArray;
    
    cudaTextureDesc tex_desc     = {};
    tex_desc.addressMode[0]      = cudaAddressModeBorder;
    tex_desc.addressMode[1]      = cudaAddressModeBorder;
    tex_desc.filterMode          = cudaFilterModeLinear;
    tex_desc.normalizedCoords    = 1;
    tex_desc.maxAnisotropy       = 1;
    tex_desc.maxMipmapLevelClamp = 99;
    tex_desc.minMipmapLevelClamp = 0;
    tex_desc.mipmapFilterMode    = cudaFilterModePoint;
    tex_desc.borderColor[0]      = 0.0f;
    tex_desc.borderColor[1]      = 0.0f;
    tex_desc.borderColor[2]      = 0.0f;
    tex_desc.borderColor[3]      = 0.0f;
    tex_desc.sRGB                = 0;
    cudaCreateTextureObject(&xf.colorMapTexture, &res_desc, &tex_desc,
                            nullptr);

    owlParamsSetRaw(lp,"transferFunc.texture",&xf.colorMapTexture);

    accumID = 0;
  }

  void OWLRenderer::setRange(interval<float> xfDomain)
  {
    xf.absDomain = xfDomain;
    range1f r{
     xf.absDomain.lower + (xf.relDomain.lower/100.f) * (xf.absDomain.upper-xf.absDomain.lower),
     xf.absDomain.lower + (xf.relDomain.upper/100.f) * (xf.absDomain.upper-xf.absDomain.lower)
    };
    owlParamsSet2f(lp,"transferFunc.domain",r.lower,r.upper);

    if (xf.colorMapBuffer) {
      grid.computeMaxOpacities(owl,xf.colorMapBuffer,r);
      owlParamsSetBuffer(lp,"grid.maxOpacities",grid.maxOpacities);
    }
  }

  void OWLRenderer::setRelDomain(interval<float> relDomain)
  {
    xf.relDomain = relDomain;
    setRange(xf.absDomain);
  }

  void OWLRenderer::setOpacityScale(float scale)
  {
    owlParamsSet1f(lp,"transferFunc.opacityScale",scale);
  }

  void OWLRenderer::setClipPlane(int id, bool enabled, vec3f N, float d)
  {
    if (id==0) {
      owlParamsSet1i(lp,"clipPlane0.enabled",(int)enabled);
      owlParamsSet3f(lp,"clipPlane0.N",N.x,N.y,N.z);
      owlParamsSet1f(lp,"clipPlane0.d",d);
      accumID = 0;
    } else if (id==1) {
      owlParamsSet1i(lp,"clipPlane1.enabled",(int)enabled);
      owlParamsSet3f(lp,"clipPlane1.N",N.x,N.y,N.z);
      owlParamsSet1f(lp,"clipPlane1.d",d);
      accumID = 0;
    } else {
      std::cerr << "Clip plane " << id << " not valid" << std::endl;
    }
  }
} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

