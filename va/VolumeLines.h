#pragma once

#include <cuda_runtime.h>
#include <owl/common/math/vec.h>
#include "common.h"
#include "model/ExaBrickModel.h"

namespace exa {
  struct VolumeLines {

    enum Mode { Bars, Lines, };

    // 1D (AMR) cell
    struct Cell {
      int lower;
      float value;
      int level;
      uint64_t hilbertID;

      inline __both__
      range1f getBounds() const
      {
        return range1f(float(lower),
                       float(lower + (1<<level)));
      }

      inline __both__
      range1f getDomain() const
      {
        const float cellWidth = (float)(1<<level);
        return range1f(float(lower) - 0.5f*cellWidth,
                       float(lower) + 1.5f*cellWidth);
      }
    };

    VolumeLines();
   ~VolumeLines();

    void reset(const ExaBrickModel::SP &model);
    void cleanup();

    void draw(cudaSurfaceObject_t surfaceObj, int w, int h);

    void setColorMap(const std::vector<vec4f> &newCM);
    void setRange(interval<float> xfDomain);
    void setRelDomain(interval<float> relDomain);
    void setOpacityScale(float scale);

    void setMinImportance(float mi);
    void setP(float P);
    void setMode(Mode m);

    void computeHilbertROIs();

    void onMouseMove(int x, int y, int button);
    void onMouseDrag(int x, int y, int button);
    void onMousePress(int x, int y, int button);
    void onMouseRelease(int x, int y, int button);

    float minImportance = 0.025f;
    float P = 1.f;
    Mode mode = Lines;
    owl::vec2i canvasSize{0,0};

    struct Map1Dto3D {
      size_t cellID_1D;
      size_t brickID;
      vec3i xyz; // relative to brick.lower
      vec3i centroid; // in cell space
    };
    std::vector<Map1Dto3D> map1Dto3D;
    std::vector<Cell *> cells; // on device
    int numCells{0}; // same for each channel!
    range1f cellBounds;
    range1f centroidBounds;
    box3f cellBounds3D;
    box3f centroidBounds3D;

    typedef owl::interval<int> ROI;
    std::vector<ROI> rois;
    std::vector<owl::interval<uint64_t>> roisToHilbertIDs;
    std::vector<owl::box3f> worldSpaceROIs;
    ROI highlight{-1,-1};
    int pressX = -1;

    struct {
      float *perCell = nullptr;
      float *cumulative = nullptr;
      void *tempStorage = nullptr; // for CUB scan!
      size_t tempStorageSizeInBytes = 0;
    } importance;

    // A cell as projected onto the 1D grid
    // that later becomes the line plot
    struct GridCell {
      float value; // before classification
      vec4f color; // after classification
    };
    std::vector<GridCell *> grids1D;
    bool updated_ = true;

    struct {
      std::vector<vec4f> colorMap;
      range1f absDomain { 0.f, 1.f };
      range1f relDomain { 0.f, 100.f };
      float opacityScale{ 1.f };
      vec4f *deviceColorMap{ nullptr };
      float alphaMax{ 0.f };
    } xf;


  };
} // ::exa
// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

