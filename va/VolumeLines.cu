#include "VolumeLines.h"

inline int64_t __host__ __device__ iDivUp(int64_t a, int64_t b)
{
  return (a + b - 1) / b;
}

__global__ void drawGPU(cudaSurfaceObject_t surfaceObj, int w, int h)
{
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.y * blockDim.y + threadIdx.y;
  if(x >= w || y >= h)
    return;

  float4 data = make_float4(x/(float)(w-1),y/(float)(h-1),1,1);
  surf2Dwrite(data, surfaceObj, x * sizeof(float4), y);
}

namespace exa {
  void VolumeLines::draw(cudaSurfaceObject_t surfaceObj, int w, int h)
  {
    dim3 blockSize;
    blockSize.x = 16;
    blockSize.y = 16;
    
    dim3 gridSize;
    gridSize.x = iDivUp(w,blockSize.x);
    gridSize.y = iDivUp(w,blockSize.y);

    drawGPU<<<gridSize, blockSize>>>(surfaceObj,w,h);
  }
} // ::exa
// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0
