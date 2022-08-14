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

#include "qtOWL/ColorMaps.h"
#include "AMRCellModel.h"
#include "ExaStitchModel.h"
#include "Grid.h"

namespace exa {

  struct OWLRenderer
  {
    OWLRenderer(const std::string inFileName,
                const std::string gridsFileName = "",
                const std::string amrCellFileName = "",
                const std::string meshFileName = "",
                const std::string scalarFileName = "");

   ~OWLRenderer();

    void setCamera(const vec3f &org,
                   const vec3f &dir_00,
                   const vec3f &dir_du,
                   const vec3f &dir_dv);
    void resize(const vec2i &newSize);
    void render(uint32_t *fbPointer);

    void setColorMap(const std::vector<vec4f> &newCM);
    void setRange(interval<float> xfDomain);
    void setRelDomain(interval<float> relDomain);
    void setOpacityScale(float scale);

    void setClipPlane(int id, bool enabled, vec3f N, float d);
    void setShadeMode(int sm);
    std::map<int,std::string> shadeModes();
    void setSampler(int sampler);

    void setSubImage(const box2f si, bool active);
    void setSubImageSelection(const box2f si, bool active);

    OWLContext owl;
    OWLModule  module;
    OWLParams  lp;
    OWLRayGen  rayGen;

    struct {
      OWLGeomType geomType;
      OWLGroup tlas;
    } meshGeom;

    struct {
      std::vector<vec4f> colorMap;
      range1f absDomain { 0.f, 1.f };
      range1f relDomain { 0.f, 100.f };
      OWLBuffer colorMapBuffer { 0 };
      cudaArray_t colorMapArray { 0 };
      cudaTextureObject_t colorMapTexture { 0 };
    } xf;

    ExaStitchModel::SP exaStitchModel { 0 };
    AMRCellModel::SP amrCellModel { 0 };
    Grid grid;

    OWLBuffer accumBuffer { 0 };
    int accumID { 0 };
    vec2i fbSize { 1 };
    
    int   spp = 1;
    bool  heatMapEnabled = 0;
    float heatMapScale = 1.f;

    void resetAccum() { accumID = 0; }

    box3f   modelBounds;
    range1f valueRange;

    bool printMemoryStats = true;
  };

} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

