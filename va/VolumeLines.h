#pragma once

#include <cuda_runtime.h>

namespace exa {
  struct VolumeLines {
    void draw(cudaSurfaceObject_t surfaceObj, int w, int h);
  };
} // ::exa
// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

