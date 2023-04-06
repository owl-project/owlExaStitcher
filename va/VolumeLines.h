#pragma once

#include <cuda_runtime.h>
#include <owl/common/math/vec.h>

namespace exa {
  struct VolumeLines {
    VolumeLines();
   ~VolumeLines();

    void draw(cudaSurfaceObject_t surfaceObj, int w, int h);

    std::vector<owl::vec2f *> polyLines;
    int numLineSegments{0}; // same for each line!
  };
} // ::exa
// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

