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

#include "OWLRenderer.h"

extern "C" char embedded_deviceCode[];

namespace exa {

  OWLVarDecl rayGenVars[]
  = {
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

  OWLVarDecl launchParamsVars[]
  = {
     { "fbPointer",   OWL_RAW_POINTER, OWL_OFFSETOF(LaunchParams,fbPointer) },
     { "fbDepth",   OWL_BUFPTR, OWL_OFFSETOF(LaunchParams,fbDepth) },
     { "accumBuffer",   OWL_BUFPTR, OWL_OFFSETOF(LaunchParams,accumBuffer) },
     { "accumID",   OWL_INT, OWL_OFFSETOF(LaunchParams,accumID) },
     { "world",    OWL_GROUP,  OWL_OFFSETOF(LaunchParams,world)},
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

  OWLRenderer::OWLRenderer()
  {
    unsigned numElems = 2;
    vec3f vertices[12] = {
      {0.f,0.f,0.f},
      {1.f,0.f,0.f},
      {1.f,1.f,0.f},
      {0.f,1.f,0.f},
      {0.f,0.f,-1.f},
      {1.f,0.f,-1.f},
      {1.f,1.f,-1.f},
      {0.f,1.f,-1.f},
      {2.f,0.f,0.f},
      {2.f,1.f,0.f},
      {2.f,0.f,-1.f},
      {2.f,1.f,-1.f}
    };

    int indices[16] = {
      0,1,2,3, 4,5,6,7,
      1,8,9,2, 5,10,11,6
    };

    modelBounds = box3f();
    for (int i=0; i<16; ++i) {
      modelBounds.extend(vertices[indices[i]]);
    }

    owl = owlContextCreate(nullptr,1);
    module = owlModuleCreate(owl,embedded_deviceCode);
    lp = owlParamsCreate(owl,sizeof(LaunchParams),launchParamsVars,-1);
    rayGen = owlRayGenCreate(owl,module,"renderFrame",sizeof(RayGen),rayGenVars,-1);

    stitchGeom.geomType = owlGeomTypeCreate(owl,
                                            OWL_GEOM_USER,
                                            sizeof(StitchGeom),
                                            stitchGeomVars, -1);
    owlGeomTypeSetBoundsProg(stitchGeom.geomType, module, "StitchGeomBounds");
    owlGeomTypeSetIntersectProg(stitchGeom.geomType, 0, module, "StitchGeomIsect");
    owlGeomTypeSetClosestHit(stitchGeom.geomType, 0, module, "StitchGeomCH");

    OWLGeom geom = owlGeomCreate(owl, stitchGeom.geomType);
    owlGeomSetPrimCount(geom, numElems);

    OWLBuffer vertexBuffer = owlDeviceBufferCreate(owl, OWL_FLOAT3,
                                                   sizeof(vertices)/sizeof(vertices[0]),
                                                   vertices);

    OWLBuffer indexBuffer = owlDeviceBufferCreate(owl, OWL_INT,
                                                  sizeof(indices)/sizeof(indices[0]),
                                                  indices);

    owlGeomSetBuffer(geom,"vertexBuffer",vertexBuffer);
    owlGeomSetBuffer(geom,"indexBuffer",indexBuffer);
    owlGeomSet3f(geom,"bounds.lower",
                 modelBounds.lower.x,
                 modelBounds.lower.y,
                 modelBounds.lower.z);
    owlGeomSet3f(geom,"bounds.upper",
                 modelBounds.upper.x,
                 modelBounds.upper.y,
                 modelBounds.upper.z);

    owlBuildPrograms(owl);

    stitchGeom.blas = owlUserGeomGroupCreate(owl, 1, &geom);
    owlGroupBuildAccel(stitchGeom.blas);

    stitchGeom.tlas = owlInstanceGroupCreate(owl, 1);
    owlInstanceGroupSetChild(stitchGeom.tlas, 0, stitchGeom.blas);

    owlGroupBuildAccel(stitchGeom.tlas);

    owlParamsSetGroup(lp, "world", stitchGeom.tlas);
    owlParamsSet3f(lp,"modelBounds.lower",
                   modelBounds.lower.x,
                   modelBounds.lower.y,
                   modelBounds.lower.z);
    owlParamsSet3f(lp,"modelBounds.upper",
                   modelBounds.upper.x,
                   modelBounds.upper.y,
                   modelBounds.upper.z);

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

} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

