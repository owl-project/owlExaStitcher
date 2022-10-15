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

#include "model/AMRCellModel.h"
#include "model/ExaBrickModel.h"
#include "model/ExaStitchModel.h"
#include "sampler/AMRCellSampler.h"
#include "sampler/ExaBrickSampler.h"
#include "sampler/ExaStitchSampler.h"
#include "LaunchParams.h"
#include "OWLRenderer.h"
#include "TriangleMesh.h"

extern "C" char embedded_deviceCode[];

namespace exa {

  OWLVarDecl rayGenVars[]
  = {
     { nullptr /* sentinel to mark end of list */ }
  };

  OWLVarDecl meshGeomVars[]
  = {
     { "indexBuffer",  OWL_BUFPTR, OWL_OFFSETOF(MeshGeom,indexBuffer)},
     { "vertexBuffer",  OWL_BUFPTR, OWL_OFFSETOF(MeshGeom,vertexBuffer)},
     { nullptr /* sentinel to mark end of list */ }
  };

  std::vector<OWLVarDecl> commonLPVars
  = {
     { "fbPointer",   OWL_RAW_POINTER, OWL_OFFSETOF(LaunchParams,fbPointer) },
     { "fbDepth",   OWL_BUFPTR, OWL_OFFSETOF(LaunchParams,fbDepth) },
     { "accumBuffer",   OWL_BUFPTR, OWL_OFFSETOF(LaunchParams,accumBuffer) },
     { "accumID",   OWL_INT, OWL_OFFSETOF(LaunchParams,accumID) },
     { "shadeMode",  OWL_INT, OWL_OFFSETOF(LaunchParams,shadeMode)},
     { "integrator",  OWL_INT, OWL_OFFSETOF(LaunchParams,integrator)},
     { "maxOpacities", OWL_BUFPTR, OWL_OFFSETOF(LaunchParams,maxOpacities) },
     { "meshBVH",    OWL_GROUP,  OWL_OFFSETOF(LaunchParams,meshBVH)},
     { "majorantBVH",    OWL_GROUP,  OWL_OFFSETOF(LaunchParams,majorantBVH)},
     { "majorantKDTree", OWL_USER_TYPE(KDTreeTraversableHandle),  OWL_OFFSETOF(LaunchParams,majorantKDTree)},
     { "majorantGrid", OWL_USER_TYPE(GridTraversableHandle),  OWL_OFFSETOF(LaunchParams,majorantGrid)},
     { "gridletBuffer",    OWL_BUFPTR,  OWL_OFFSETOF(LaunchParams,gridletBuffer)},
     { "worldSpaceBounds.lower",  OWL_FLOAT3, OWL_OFFSETOF(LaunchParams,worldSpaceBounds.lower)},
     { "worldSpaceBounds.upper",  OWL_FLOAT3, OWL_OFFSETOF(LaunchParams,worldSpaceBounds.upper)},
     { "voxelSpaceTransform", OWL_USER_TYPE(affine3f), OWL_OFFSETOF(LaunchParams,voxelSpaceTransform)},
     { "lightSpaceTransform", OWL_USER_TYPE(affine3f), OWL_OFFSETOF(LaunchParams,lightSpaceTransform)},
     // xf data
     { "transferFunc.domain",OWL_FLOAT2, OWL_OFFSETOF(LaunchParams,transferFunc.domain) },
     { "transferFunc.texture",   OWL_USER_TYPE(cudaTextureObject_t),OWL_OFFSETOF(LaunchParams,transferFunc.texture) },
     { "transferFunc.opacityScale", OWL_FLOAT, OWL_OFFSETOF(LaunchParams,transferFunc.opacityScale) },
     // render settings
     { "render.dt",           OWL_FLOAT,   OWL_OFFSETOF(LaunchParams,render.dt) },
     { "render.spp",           OWL_INT,   OWL_OFFSETOF(LaunchParams,render.spp) },
     { "render.heatMapEnabled", OWL_INT, OWL_OFFSETOF(LaunchParams,render.heatMapEnabled) },
     { "render.heatMapScale", OWL_FLOAT, OWL_OFFSETOF(LaunchParams,render.heatMapScale) },
     // clip planes
     { "clipPlane0.enabled",     OWL_INT,   OWL_OFFSETOF(LaunchParams,clipPlanes[0].enabled) },
     { "clipPlane0.N",     OWL_FLOAT3,   OWL_OFFSETOF(LaunchParams,clipPlanes[0].N) },
     { "clipPlane0.d",     OWL_FLOAT,   OWL_OFFSETOF(LaunchParams,clipPlanes[0].d) },
     { "clipPlane1.enabled",     OWL_INT,   OWL_OFFSETOF(LaunchParams,clipPlanes[1].enabled) },
     { "clipPlane1.N",     OWL_FLOAT3,   OWL_OFFSETOF(LaunchParams,clipPlanes[1].N) },
     { "clipPlane1.d",     OWL_FLOAT,   OWL_OFFSETOF(LaunchParams,clipPlanes[1].d) },
     // lights
     { "light0.pos", OWL_FLOAT3, OWL_OFFSETOF(LaunchParams,lights[0].pos)},
     { "light0.intensity", OWL_FLOAT, OWL_OFFSETOF(LaunchParams,lights[0].intensity)},
     { "light0.on", OWL_INT, OWL_OFFSETOF(LaunchParams,lights[0].on)},
     { "light1.pos", OWL_FLOAT3, OWL_OFFSETOF(LaunchParams,lights[1].pos)},
     { "light1.intensity", OWL_FLOAT, OWL_OFFSETOF(LaunchParams,lights[1].intensity)},
     { "light1.on", OWL_INT, OWL_OFFSETOF(LaunchParams,lights[1].on)},
     { "light2.pos", OWL_FLOAT3, OWL_OFFSETOF(LaunchParams,lights[2].pos)},
     { "light2.intensity", OWL_FLOAT, OWL_OFFSETOF(LaunchParams,lights[2].intensity)},
     { "light2.on", OWL_INT, OWL_OFFSETOF(LaunchParams,lights[2].on)},
     { "light3.pos", OWL_FLOAT3, OWL_OFFSETOF(LaunchParams,lights[3].pos)},
     { "light3.intensity", OWL_FLOAT, OWL_OFFSETOF(LaunchParams,lights[3].intensity)},
     { "light3.on", OWL_INT, OWL_OFFSETOF(LaunchParams,lights[3].on)},
     // camera settings
     { "camera.org",    OWL_FLOAT3, OWL_OFFSETOF(LaunchParams,camera.org) },
     { "camera.dir_00", OWL_FLOAT3, OWL_OFFSETOF(LaunchParams,camera.dir_00) },
     { "camera.dir_du", OWL_FLOAT3, OWL_OFFSETOF(LaunchParams,camera.dir_du) },
     { "camera.dir_dv", OWL_FLOAT3, OWL_OFFSETOF(LaunchParams,camera.dir_dv) },
     // sub-image region selection
     { "subImage.value.lower", OWL_FLOAT2, OWL_OFFSETOF(LaunchParams,subImage.value.lower) },
     { "subImage.value.upper", OWL_FLOAT2, OWL_OFFSETOF(LaunchParams,subImage.value.upper) },
     { "subImage.selection.lower", OWL_FLOAT2, OWL_OFFSETOF(LaunchParams,subImage.selection.lower) },
     { "subImage.selection.upper", OWL_FLOAT2, OWL_OFFSETOF(LaunchParams,subImage.selection.upper) },
     { "subImage.active", OWL_INT, OWL_OFFSETOF(LaunchParams,subImage.active) },
     { "subImage.selecting", OWL_INT, OWL_OFFSETOF(LaunchParams,subImage.selecting) },
  };
  
  // ==================================================================
  // Renderer class
  // ==================================================================

  OWLRenderer::OWLRenderer(const std::string umeshFileName,
                           const std::string gridsFileName,
                           const std::string amrCellFileName,
                           const std::string exaBrickFileName,
                           const std::string meshFileName,
                           const std::string scalarFileName,
                           const std::string kdtreeFileName,
                           const box3f remap_from,
                           const box3f remap_to,
                           const vec3i numMCs)
  {
    // ==================================================================
    // AMR/UMesh models etc
    // ==================================================================

    if (!umeshFileName.empty() || !gridsFileName.empty()) { // only need one of them
      model = ExaStitchModel::load(umeshFileName,gridsFileName,scalarFileName);
    }
    else if (!exaBrickFileName.empty() && !scalarFileName.empty()) {
      model = ExaBrickModel::load(exaBrickFileName,scalarFileName,kdtreeFileName);
    }
    else if (!amrCellFileName.empty() && !scalarFileName.empty()) {
      model = AMRCellModel::load(amrCellFileName,scalarFileName);
    }

    if (!model) {
      throw std::runtime_error("Could not load module");
    }

    vec3f lightSpaceScale = 1.f;
    vec3f cellSpaceSize = model->cellBounds.size();
    while (reduce_max(cellSpaceSize) > 1000.f) {
      cellSpaceSize /= 1000.f;
      lightSpaceScale /= 1000.f;
    }

    model->setVoxelSpaceTransform(remap_from,remap_to);
#ifdef EXA_STITCH_MIRROR_EXAJET
    model->initMirrorExajet(); // before extending model bounds!
#endif
    lightSpaceTransform = lightSpaceTransform.scale(lightSpaceScale);
    modelBounds.extend(model->getBounds());
    valueRange.extend(model->valueRange);

    model->setNumGridCells(numMCs);

    // ==================================================================
    // Meshes
    // ==================================================================

    std::vector<TriangleMesh::SP> meshes;
    if (!meshFileName.empty()) {
      meshes = TriangleMesh::load(meshFileName);
    }

    // ==================================================================
    // Print memory stats
    // ==================================================================

    if (printMemoryStats) {
      size_t elemVertexBytes = 0;
      size_t elemIndexBytes = 0;
      size_t gridletBytes = 0;
      size_t emptyScalarsBytes = 0;
      size_t nonEmptyScalarsBytes = 0;
      size_t exaBrickBytes = 0;
      size_t exaScalarBytes = 0;
      size_t abrBytes = 0;
      size_t abrLeafListBytes = 0;
      size_t amrCellBytes = 0;
      size_t amrScalarBytes = 0;
      size_t meshIndexBytes = 0;
      size_t meshVertexBytes = 0;
      for (const auto &mesh : meshes) {
        meshIndexBytes += mesh->index.size()*sizeof(mesh->index[0]);
        meshVertexBytes += mesh->vertex.size()*sizeof(mesh->vertex[0]);
      }

      if (auto mod = model->as<ExaStitchModel>()) {
        mod->memStats(elemVertexBytes,elemIndexBytes,gridletBytes,
                      emptyScalarsBytes,nonEmptyScalarsBytes);
      }
      else if (auto mod = model->as<ExaBrickModel>()) {
        mod->memStats(exaBrickBytes,exaScalarBytes,abrBytes,abrLeafListBytes);
      }
      else if (auto mod = model->as<AMRCellModel>()) {
        mod->memStats(amrCellBytes,amrScalarBytes);
      }

      size_t totalBytes = elemVertexBytes+elemIndexBytes+emptyScalarsBytes+nonEmptyScalarsBytes
                        + gridletBytes+amrCellBytes+amrScalarBytes
                        + exaBrickBytes+exaScalarBytes+abrBytes
                        + meshIndexBytes+meshVertexBytes;

      std::cout << " ====== Memory Stats (bytes) ======= \n";
      std::cout << "elem.vertex.........: " << owl::prettyBytes(elemVertexBytes) << '\n';
      std::cout << "elem.index..........: " << owl::prettyBytes(elemIndexBytes) << '\n';
      std::cout << "Non-empty scalars...: " << owl::prettyBytes(nonEmptyScalarsBytes) << '\n';
      std::cout << "Empty scalars.......: " << owl::prettyBytes(emptyScalarsBytes) << '\n';
      std::cout << "Gridlets............: " << owl::prettyBytes(gridletBytes) << '\n';
      std::cout << "AMR cells...........: " << owl::prettyBytes(amrCellBytes) << '\n';
      std::cout << "AMR cells...........: " << owl::prettyBytes(amrCellBytes) << '\n';
      std::cout << "EXA bricks..........: " << owl::prettyBytes(exaBrickBytes) << '\n';
      std::cout << "EXA scalars.........: " << owl::prettyBytes(exaScalarBytes) << '\n';
      std::cout << "EXA ABRs............: " << owl::prettyBytes(abrBytes) << '\n';
      std::cout << "EXA ABR leaf list...: " << owl::prettyBytes(abrLeafListBytes) << '\n';
      std::cout << "mesh.vertex.........: " << owl::prettyBytes(meshVertexBytes) << '\n';
      std::cout << "mesh.index..........: " << owl::prettyBytes(meshIndexBytes) << '\n';
      std::cout << "TOTAL...............: " << owl::prettyBytes(totalBytes) << '\n';
    }


    // ==================================================================
    // Upload to GPU
    // ==================================================================

    if (model->as<AMRCellModel>()) {
      sampler = std::make_shared<AMRCellSampler>();
    } else if (model->as<ExaBrickModel>()) {
      sampler = std::make_shared<ExaBrickSampler>();
    } else if (model->as<ExaStitchModel>()) {
      sampler = std::make_shared<ExaStitchSampler>();
    }

    if (!sampler) {
      throw std::runtime_error("Could not load module");
    }

    owl = owlContextCreate(nullptr,1);
    owlContextSetRayTypeCount(owl,2);
    module = owlModuleCreate(owl,embedded_deviceCode);
    const std::string renderFrame
      = "renderFrame_"+sampler->className();
    PRINT(renderFrame);
    rayGen = owlRayGenCreate(owl,module,renderFrame.c_str(),0,rayGenVars,-1);

    // -------------------------------------------------------
    // set up launch params
    // -------------------------------------------------------
    std::vector<OWLVarDecl> samplerLPVars = sampler->getLPVariables();

    std::vector<OWLVarDecl> lpVars;
    for (auto var : commonLPVars)
      if (var.name)
        lpVars.push_back(var);

    for (auto var : samplerLPVars)
      if (var.name) {
        var.offset += OWL_OFFSETOF(LaunchParams,sampler);
        lpVars.push_back(var);
      }
    lpVars.push_back({nullptr}); // sentinel
    lp = owlParamsCreate(owl,sizeof(LaunchParams),lpVars.data(),-1);

    if (sampler->build(owl,model)) {

      // Currently only for ExaBricks, build an optional BVH over the majorant grid
      // (the OPtiX code for that lives in *this* module!)
      if (auto ebs = sampler->as<ExaBrickSampler>()) {
        if (ebs->traversalMode == MC_BVH_TRAVERSAL)
          if (!ebs->buildOptixBVH(owl,module))
            throw std::runtime_error("Building BVH from grid failed");
      }

      // Set the traversal accel (whatever that _is_)
      if (sampler->majorantAccel.bvh) {
        owlParamsSetGroup(lp,"majorantBVH",sampler->majorantAccel.bvh);
      } else if (sampler->majorantAccel.grid) {
        owlParamsSetRaw(lp,"majorantGrid",&sampler->majorantAccel.grid->deviceTraversable);
      } else if (sampler->majorantAccel.kdtree) {
        owlParamsSetRaw(lp,"majorantKDTree",&sampler->majorantAccel.kdtree->deviceTraversable);
      } else {
        fprintf(stderr,"Model's accels not set, forgot to call sampler->build()?\n");
      }

      sampler->setLPs(lp);
    } else {
      throw std::runtime_error("GPU upload failed");
    }

    // ==================================================================
    // mesh geom
    // ==================================================================

    if (!meshes.empty()) {
      box3f meshBounds;
      for (auto &mesh : meshes) {
        box3f bounds;
        for (size_t i=0; i<mesh->index.size(); ++i) {
          const vec3f v1 = mesh->vertex[mesh->index[i].x];
          const vec3f v2 = mesh->vertex[mesh->index[i].y];
          const vec3f v3 = mesh->vertex[mesh->index[i].z];

          bounds.extend(v1);
          bounds.extend(v2);
          bounds.extend(v3);
        }
        meshBounds.extend(bounds);
      }

      meshGeom.geomType = owlGeomTypeCreate(owl,
                                            OWL_TRIANGLES,
                                            sizeof(MeshGeom),
                                            meshGeomVars, -1);
      owlGeomTypeSetClosestHit(meshGeom.geomType,
                               RADIANCE_RAY_TYPE,
                               module,
                               "MeshGeomCH");

      owlBuildPrograms(owl);

#ifdef EXA_STITCH_MIRROR_EXAJET

      meshGeom.tlas = owlInstanceGroupCreate(owl,meshes.size() * 2);
#else
      meshGeom.tlas = owlInstanceGroupCreate(owl,meshes.size());
#endif

      for (int meshID=0;meshID<meshes.size();meshID++) {
        auto &mesh = meshes[meshID];
        OWLGroup blas;
        OWLGeom geom;
        OWLBuffer vertexBuffer;
        OWLBuffer indexBuffer;

        indexBuffer = owlDeviceBufferCreate(owl,
                                            OWL_INT3,
                                            mesh->index.size(),
                                            mesh->index.data());

        vertexBuffer = owlDeviceBufferCreate(owl,
                                             OWL_FLOAT3,
                                             mesh->vertex.size(),
                                             mesh->vertex.data());

        geom = owlGeomCreate(owl, meshGeom.geomType);
        owlGeomSetBuffer(geom, "indexBuffer", indexBuffer);
        owlGeomSetBuffer(geom, "vertexBuffer", vertexBuffer);
        owlTrianglesSetIndices(geom,
                               indexBuffer,
                               mesh->index.size(),
                               sizeof(vec3i),
                               0);
        owlTrianglesSetVertices(geom,
                                vertexBuffer,
                                mesh->vertex.size(),
                                sizeof(vec3f),
                                0);

        blas = owlTrianglesGeomGroupCreate(owl, 1, &geom);
        owlGroupBuildAccel(blas);
        owlInstanceGroupSetChild(meshGeom.tlas, meshID, blas);

        modelBounds.extend(meshBounds);

#ifdef EXA_STITCH_MIRROR_EXAJET
        owl4x3f tfm;
        tfm.t = owl3f{0.f, 0.f, 0.f};
        tfm.vx = owl3f{1.f, 0.f, 0.f};
        tfm.vy = owl3f{0.f, -1.f, 0.f};
        tfm.vz = owl3f{0.f, 0.f, 1.f};
        owlInstanceGroupSetChild(meshGeom.tlas, meshes.size() + meshID, blas);
        owlInstanceGroupSetTransform(meshGeom.tlas, meshes.size() + meshID, &tfm);

        const affine3f &a3f = (const affine3f&)tfm;
        vec3f lower = xfmPoint(a3f,meshBounds.lower);
        vec3f upper = xfmPoint(a3f,meshBounds.upper);
        box3f mirrorBounds{
          min(lower,upper),
          max(lower,upper)
        };
        modelBounds.extend(mirrorBounds);
#endif
      }
      owlGroupBuildAccel(meshGeom.tlas);

      owlParamsSetGroup(lp, "meshBVH", meshGeom.tlas);
    }


    owlParamsSet3f(lp,"worldSpaceBounds.lower",
                   modelBounds.lower.x,
                   modelBounds.lower.y,
                   modelBounds.lower.z);
    owlParamsSet3f(lp,"worldSpaceBounds.upper",
                   modelBounds.upper.x,
                   modelBounds.upper.y,
                   modelBounds.upper.z);


    for (int i=0; i<CLIP_PLANES_MAX; ++i) {
      setClipPlane(i,false,vec3f{0,0,1},modelBounds.center().z);
    }

    owlParamsSet2f(lp,"subImage.value.lower",0.f,0.f);
    owlParamsSet2f(lp,"subImage.value.upper",0.f,1.f);
    owlParamsSet2f(lp,"subImage.selection.lower",0.f,0.f);
    owlParamsSet2f(lp,"subImage.selection.upper",1.f,1.f);
    owlParamsSet1i(lp,"subImage.active",0);
    owlParamsSet1i(lp,"subImage.selecting",0);

    owlParamsSet1i(lp,"integrator",0);

    owlParamsSet1i(lp,"shadeMode",0);
    owlParamsSet1i(lp,"light0.on",0);
    owlParamsSet1i(lp,"light1.on",0);
    owlParamsSet1i(lp,"light2.on",0);
    owlParamsSet1i(lp,"light3.on",0);

    owlParamsSetRaw(lp,"voxelSpaceTransform",&model->voxelSpaceTransform);
    owlParamsSetRaw(lp,"lightSpaceTransform",&lightSpaceTransform);

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

  void OWLRenderer::setType(const OWLRenderer::Type type)
  {
    owlParamsSet1i(lp,"integrator",(int)type);
    accumID = 0;
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

    sampler->computeMaxOpacities(owl,xf.colorMapBuffer,r);

    owlParamsSetBuffer(lp,"maxOpacities",sampler->maxOpacities);

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

    sampler->computeMaxOpacities(owl,xf.colorMapBuffer,r);

    owlParamsSetBuffer(lp,"maxOpacities",sampler->maxOpacities);
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

  void OWLRenderer::setShadeMode(int sm)
  {
    owlParamsSet1i(lp,"shadeMode",sm);
    accumID = 0;
  }

  std::map<int,std::string> OWLRenderer::shadeModes()
  {
    return {{0,"default"},
            {1,"gridlets"},
            {2,"teaser"}};
  }

  void OWLRenderer::setSubImage(const box2f si, bool active)
  {
    owlParamsSet2f(lp,"subImage.value.lower",si.lower.x,si.lower.y);
    owlParamsSet2f(lp,"subImage.value.upper",si.upper.x,si.upper.y);
    owlParamsSet1i(lp,"subImage.active",(int)active);
    accumID = 0;
  }

  void OWLRenderer::setSubImageSelection(const box2f si, bool active)
  {
    owlParamsSet2f(lp,"subImage.selection.lower",si.lower.x,si.lower.y);
    owlParamsSet2f(lp,"subImage.selection.upper",si.upper.x,si.upper.y);
    owlParamsSet1i(lp,"subImage.selecting",(int)active);
    accumID = 0;
  }

  void OWLRenderer::setLightSource(int lightID, const owl::vec3f &pos, float intensity, bool on)
  {
    if (lightID==0) {
      owlParamsSet3f(lp,"light0.pos",pos.x,pos.y,pos.z);
      owlParamsSet1f(lp,"light0.intensity",intensity);
      owlParamsSet1i(lp,"light0.on",(int)on);
    } else assert(0); // TODO!
    accumID = 0;
  }

  void OWLRenderer::setLightSpaceTransform(const affine3f xform)
  {
    lightSpaceTransform = xform;
    owlParamsSetRaw(lp,"lightSpaceTransform",&lightSpaceTransform);
    accumID = 0;
  }

} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

