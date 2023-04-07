#include <vector>
#include "VolumeLines.h"
#include "atomicOp.cuh"

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

__global__ void renderGPU(cudaSurfaceObject_t surfaceObj, float *grid, int w, int h)
{
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  
  if (x >= w)
    return;

  int H=owl::clamp(int(grid[x]*h),0,h-1);
  for (int y=0; y<=H; ++y) {
    float4 src;
    surf2Dread(&src, surfaceObj, x * sizeof(float4), h-y-1);
    owl::vec3f c(0.f);
    float4 color = make_float4(src.x*0.5f+c.x*0.5f,
                               src.y*0.5f+c.y*0.5f,
                               src.z*0.5f+c.z*0.5f,
                               1.f);
    surf2Dwrite(color, surfaceObj, x * sizeof(float4), h-y-1);
  }
}

__global__ void basisRasterCells(float *grid,
                                 float *weights,
                                 int dims,
                                 const exa::VolumeLines::Cell *cells,
                                 int numCells,
                                 range1f cellBounds)
{
  int primID = blockIdx.x * blockDim.x + threadIdx.x;

  if (primID >= numCells)
    return;

  const auto &cell = cells[primID];
  range1f bounds = cell.getBounds();
  float p1 = bounds.lower;
  float p2 = bounds.upper;

  // Project onto grid (TODO: move to function..)
  float x1_01 = (p1-cellBounds.lower)/(cellBounds.upper-cellBounds.lower);
  float x2_01 = (p2-cellBounds.lower)/(cellBounds.upper-cellBounds.lower);

  int x1 = owl::clamp(int(x1_01*float(dims)),0,dims-1);
  int x2 = owl::clamp(int(x2_01*float(dims)),0,dims-1);

  for (int x=x1; x<=x2; ++x) {
    // TODO: that's a box-shaped basis function
    // this _might_ be ok, but only if we have
    // many cells..
    atomicAdd(&grid[x], cell.value);
    atomicAdd(&weights[x], 1.f);
  }
}

__global__ void basisAverageGridCells(float *grid,
                                      float *weights,
                                      int dims)
{
  int x = blockIdx.x * blockDim.x + threadIdx.x;

  if (x >= dims)
    return;

  if (weights[x] > 0.f)
    grid[x] /= weights[x];
}

namespace exa {
  VolumeLines::VolumeLines()
  {
    // cells.resize(2);
    // for (size_t i=0; i<cells.size(); ++i) {
    //   std::vector<Cell> cs(2000000);
    //   for (size_t j=0; j<cs.size(); ++j) {
    //     int level=0;
    //     //float value(cs.size()/2);
    //     float value = rand()/float(RAND_MAX);
    //     cs[j] = {(int)j,value,level};
    //     cellBounds.extend(cs[j].getBounds());
    //   }

    //   cudaMalloc(&cells[i], cs.size()*sizeof(cs[0]));
    //   cudaMemcpy(cells[i], cs.data(), cs.size()*sizeof(cs[0]), cudaMemcpyHostToDevice);

    //   numCells = cs.size();
    // }
  }

  VolumeLines::~VolumeLines()
  {
    for (size_t i=0; i<cells.size(); ++i) {
      cudaFree(cells[i]);
    }
  }

  void VolumeLines::reset(const ExaBrickModel::SP &model)
  {
    for (size_t i=0; i<cells.size(); ++i) {
      cudaFree(cells[i]);
    }

    cells.resize(1);
    cellBounds = {};
    std::vector<Cell> cs;
    for (size_t i=0; i<model->bricks.size(); ++i) {
      const ExaBrick &brick = model->bricks[i];
      for (int z=0; z<brick.size.z; ++z) {
        for (int y=0; y<brick.size.y; ++y) {
          for (int x=0; x<brick.size.x; ++x) {
            int idx = brick.getIndexIndex({x,y,z});
            cs.push_back({idx,model->scalars[idx],brick.level});
            cellBounds.extend(cs.back().getBounds());
          }
        }
      }
    }

    cudaMalloc(&cells[0], cs.size()*sizeof(cs[0]));
    cudaMemcpy(cells[0], cs.data(), cs.size()*sizeof(cs[0]), cudaMemcpyHostToDevice);
    numCells = cs.size();
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

    // raster cells onto 1D grids
    std::vector<float *> grids1D;
    for (size_t i=0; i<cells.size(); ++i) {
      float *grid;
      cudaMalloc(&grid, sizeof(float)*w);
      cudaMemset(grid, 0, sizeof(float)*w);

      float *weights;
      cudaMalloc(&weights, sizeof(float)*w);
      cudaMemset(weights, 0, sizeof(float)*w);

      size_t numThreads = 1024;
      basisRasterCells<<<iDivUp(numCells,numThreads),numThreads>>>(
        grid, weights, w, cells[i], numCells, cellBounds);

      grids1D.push_back(grid);

      basisAverageGridCells<<<iDivUp(w,numThreads),numThreads>>>(
        grid, weights, w);

      cudaFree(weights);
    }

    // render to texture
    for (size_t i=0; i<grids1D.size(); ++i) {
      size_t numThreads = 1024;
      renderGPU<<<iDivUp(w,numThreads),numThreads>>>(
        surfaceObj,grids1D[i],w,h);
    }

    // temp. grids aren't needed anymore
    for (size_t i=0; i<grids1D.size(); ++i) {
      cudaFree(grids1D[i]);
    }
  }
} // ::exa
// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0
