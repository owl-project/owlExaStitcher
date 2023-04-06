#include <vector>
#include "VolumeLines.h"

inline int64_t __host__ __device__ iDivUp(int64_t a, int64_t b)
{
  return (a + b - 1) / b;
}

__global__ void fillGPU(cudaSurfaceObject_t surfaceObj, int w, int h)
{
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.y * blockDim.y + threadIdx.y;
  if(x >= w || y >= h)
    return;

  if ((x/16) % 2 == (y/16) % 2)
    surf2Dwrite(make_float4(1.f,1.f,1.f,1.f), surfaceObj, x * sizeof(float4), h-y-1);
  else
    surf2Dwrite(make_float4(.9f,.9f,.9f,.9f), surfaceObj, x * sizeof(float4), h-y-1);
}

__global__ void rasterPolyLines(cudaSurfaceObject_t surfaceObj,
                                int w,
                                int h,
                                const owl::vec2f *polyLines,
                                int numLineSegments)
{
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  if (x >= w)
    return;

  int N=10;
  float incf = 1.f/N;
  float xf = x;
  float minValue =  1e30f;
  float maxValue = -1e30f;
  float avg = 0.f;
  for (int i=0; i<N; ++i) {
    int l = (xf/w)*numLineSegments;
    int p1 = l*(w/numLineSegments);
    float f = (xf-p1)/(w/float(numLineSegments));
    float val1 = polyLines[l].y;
    float val2 = polyLines[l+1].y;
    float v = (1-f)*val1 + f*val2;
    minValue = fminf(minValue,v);
    maxValue = fmaxf(maxValue,v);
    avg += v;
    xf += incf;
  }
  avg /= N;
  int Y = avg*h;

  for (int y=Y-2; y<=Y+2; ++y) {
    float dist = fabsf(float(y)-avg*h)/2;
    float4 prev;
    surf2Dread(&prev, surfaceObj, x * sizeof(float4), owl::clamp(h-y-1,0,h-1));
    owl::vec3f c = (1-dist)*owl::vec3f(0.f) + dist*owl::vec3f(prev.x,prev.y,prev.z);
    surf2Dwrite(make_float4(c.x,c.y,c.z,1.f), surfaceObj, x * sizeof(float4), owl::clamp(h-y-1,0,h-1));
  }
}

namespace exa {
  VolumeLines::VolumeLines()
  {
    polyLines.resize(2);
    for (size_t i=0; i<polyLines.size(); ++i) {
      std::vector<owl::vec2f> pl(10);
      pl[0] = 0.f; // x-coords not used (yet??), ok that they're bogus..
      pl[1] = 1.f;
      pl[2] = 0.25f;
      pl[3] = 0.75f;
      pl[4] = 0.5f;
      pl[5] = 0.25f;
      pl[6] = 0.25f;
      pl[7] = 0.35f;
      pl[8] = 0.25f;
      pl[9] = 0.35f;

      if (i>0) {
        for (size_t j=0; j<pl.size(); ++j) {
          pl[j].y = 1.f-pl[j].y;
        }
      }

      cudaMalloc(&polyLines[i], pl.size()*sizeof(pl[0]));
      cudaMemcpy(polyLines[i], pl.data(), pl.size()*sizeof(pl[0]), cudaMemcpyHostToDevice);

      numLineSegments = pl.size()-1;
    }
  }

  VolumeLines::~VolumeLines()
  {
    for (size_t i=0; i<polyLines.size(); ++i) {
      cudaFree(polyLines[i]);
    }
  }

  void VolumeLines::draw(cudaSurfaceObject_t surfaceObj, int w, int h)
  {
    // Fill background
    {
      dim3 blockSize;
      blockSize.x = 16;
      blockSize.y = 16;
      
      dim3 gridSize;
      gridSize.x = iDivUp(w,blockSize.x);
      gridSize.y = iDivUp(w,blockSize.y);

      fillGPU<<<gridSize, blockSize>>>(surfaceObj,w,h);
    }

    // raster polylines on a 1D grid
    for (size_t i=0; i<polyLines.size(); ++i) {
      size_t numThreads = 1024;
      rasterPolyLines<<<iDivUp(w,numThreads),numThreads>>>(
        surfaceObj, w, h, polyLines[i], numLineSegments);
    }
  }
} // ::exa
// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0
