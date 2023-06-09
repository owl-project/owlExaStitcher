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

#pragma once

#include <cuda_runtime.h>
#include "qtOWL/ColorMaps.h"
#include "model/Model.h"
#include "sampler/Sampler.h"
#include "TriangleMesh.h"

namespace exa {

  struct ExaBrick;

  struct OWLRenderer
  {
    //! Different modes the renderer will run in
    enum class Type { PathTracer, DirectLighting, RayMarcher, };

    // Construct from files
    OWLRenderer(const std::string umeshFileName,
                const std::string gridsFileName = "",
                const std::string amrCellFileName = "",
                const std::string exaBrickFileName = "",
                const std::string bigMeshFileName = "",
                const std::string quickClustersFileName = "",
                const std::string meshFileName = "",
                const std::string scalarFileName = "",
                const std::string kdtreeFileName = "",
                const std::string majorantsFileName = "",
                const box3f remap_from = {{0.f,0.f,0.f},{1.f,1.f,1.f}},
                const box3f remap_to = {{0.f,0.f,0.f},{1.f,1.f,1.f}},
                const vec3i numMCs = {128,128,128});

    //! Construct as ExaBricks
    OWLRenderer(const ExaBrick *bricks,
                const float *scalars,
                size_t numBricks,
                const box3f remap_from = {{0.f,0.f,0.f},{1.f,1.f,1.f}},
                const box3f remap_to = {{0.f,0.f,0.f},{1.f,1.f,1.f}},
                const vec3i numMCs = {128,128,128});

    /*! \brief  Construct as block-structured (like OSPRay, or VTK)
     * TODO: as of now this constructor isn't implemented yet
     * but will eventually generate an ExaStitch model internally! */
    struct BlockData { vec3i size; float *data; };
    OWLRenderer(const box3i *blockBounds,
                const int *blockLevel,
                const BlockData *blockData,
                size_t numBlocks,
                const box3f remap_from = {{0.f,0.f,0.f},{1.f,1.f,1.f}},
                const box3f remap_to = {{0.f,0.f,0.f},{1.f,1.f,1.f}},
                const vec3i numMCs = {128,128,128});

   ~OWLRenderer();

    //! Init GPU data structures using OptiX/OWL
    void initGPU();

    //! Set camera parameters, compatible with SimpleCamera from cuteeOWL
    void setCamera(const vec3f &org,
                   const vec3f &dir_00,
                   const vec3f &dir_du,
                   const vec3f &dir_dv);

    //! Resize the rendering viewport
    void resize(const vec2i &newSize);

    //! Render into framebuffer (device pointer!)
    void render(uint32_t *fbPointer);

    //! Set the rendering type/algorithm
    void setType(const Type t);

    //! Set colormap/RGBA transfer function
    void setColorMap(const std::vector<vec4f> &newCM);

    //! Set the valid range of the transfer function
    void setRange(interval<float> xfDomain);

    //! Set the relative range of the transfer function
    void setRelDomain(interval<float> relDomain);

    //! Set opacity scale parameter that applies to the alpha TF
    void setOpacityScale(float scale);

    void setClipPlane(int id, bool enabled, vec3f N, float d);
    void setShadeMode(int sm);
    std::map<int,std::string> shadeModes();
    void setSampler(int sampler);
    void setSubImage(const box2f si, bool active);
    void setSubImageSelection(const box2f si, bool active);

    void setLightSource(int lightID, const owl::vec3f &pos, float intensity, bool on);
    void setLightSpaceTransform(const affine3f xform);

    // Internal data structures
    OWLContext owl;
    OWLModule  module;
    OWLParams  lp;
    OWLRayGen  rayGen;

    Type type = Type::DirectLighting;

    // Optional triangle mesh; this can be supplied through
    // the constructore taking file names (proprietary triangle
    // list format!)
    struct {
      OWLGeomType geomType;
      OWLGroup tlas;
    } meshGeom;

    // We support a prototypical mode where the user can generate
    // their own majorants. These can e.g. be generated using the
    // SAHBuilder class and kdtreeBuilder tool. If own-majorants
    // and associated boxes are supplied (file-name constructor)
    // these are used for traversal. own-majorants (usually) are
    // specific to a single, custom transfer function
    struct {
      OWLGeomType geomType;
      OWLGroup blas;
      OWLGroup tlas;
      OWLBuffer domainBuffer { 0 };
      OWLBuffer maxOpacityBuffer { 0 };
    } ownMajorants;

    // Internal data structures to represent the transfer function
    struct {
      std::vector<vec4f> colorMap;
      range1f absDomain { 0.f, 1.f };
      range1f relDomain { 0.f, 100.f };
      OWLBuffer colorMapBuffer { 0 };
      cudaArray_t colorMapArray { 0 };
      cudaTextureObject_t colorMapTexture { 0 };
    } xf;

    // Model can e.g. be ExaStitcher, ExaBricks, etc.
    Model::SP model { 0 };
    // Sampler is generated automatically from the given model
    Sampler::SP sampler { 0 };
    std::vector<TriangleMesh::SP> meshes;
    std::vector<std::pair<box3f,float>> majorants; // (optional) majorant regions

    OWLBuffer accumBuffer { 0 };
    int accumID { 0 };
    vec2i fbSize { 1 };
    
    int   spp = 1;
    bool  heatMapEnabled = 0;
    float heatMapScale = 1.f;

    // Call resetAccum() to reset Monte Carlo convergence frame rendering
    void resetAccum() { accumID = 0; }

    box3f    modelBounds;
    range1f  valueRange;
    affine3f lightSpaceTransform;

    bool printMemoryStats = true;
    vec3f mirrorAxis;
  };

} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

