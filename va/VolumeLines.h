#pragma once

#include <cuda_runtime.h>
#include <owl/common/math/vec.h>
#include "common.h"
#include "model/ExaBrickModel.h"

namespace exa {
  struct VolumeLines {

    // 1D (AMR) cell
    struct Cell {
      int lower;
      float value;
      int level;

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

    void draw(cudaSurfaceObject_t surfaceObj, int w, int h);

    std::vector<Cell *> cells;
    int numCells{0}; // same for each channel!
    range1f cellBounds;

    std::vector<float *> grids1D;
    bool updated_ = true;
  };
} // ::exa
// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

