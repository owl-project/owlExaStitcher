#include <vector>
#include "VolumeLines.h"
#include "atomicOp.cuh"
#include "hilbert.h"

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
    surf2Dwrite(make_float4(.4f,.4f,.4f,1.f), surfaceObj, x * sizeof(float4), h-y-1);
  else
    surf2Dwrite(make_float4(.3f,.3f,.3f,1.f), surfaceObj, x * sizeof(float4), h-y-1);
}

__global__ void renderGPU(cudaSurfaceObject_t surfaceObj,
                          exa::VolumeLines::GridCell *grid, int w, int h)
{
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  
  if (x >= w)
    return;

  int H=owl::clamp(int(grid[x].color.w*h),0,h-1);
  for (int y=0; y<=H; ++y) {
    float4 src;
    surf2Dread(&src, surfaceObj, x * sizeof(float4), h-y-1);
    owl::vec3f c(grid[x].color);
    float4 color = make_float4(src.x*0.f+c.x*1.f,
                               src.y*0.f+c.y*1.f,
                               src.z*0.f+c.z*1.f,
                               1.f);
    surf2Dwrite(color, surfaceObj, x * sizeof(float4), h-y-1);
  }
}

inline __device__
vec4f lookupTransferFunction(float f,
                             const vec4f *colorMap,
                             const int numColors,
                             const range1f xfDomain)
{
  if (xfDomain.lower >= xfDomain.upper)
    return vec4f(0.f);

  f -= xfDomain.lower;
  f /= (xfDomain.upper-xfDomain.lower);
  if (numColors == 0)
    return vec4f(0.f);

  f = max(0.f,min(1.f,f));
  int i = min(numColors-1,int(f * numColors));
  return colorMap[i];
}

__global__ void basisRasterCells(exa::VolumeLines::GridCell *grid,
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
    atomicAdd(&grid[x].value, cell.value);
    atomicAdd(&weights[x], 1.f);
  }
}

__global__ void basisAverageGridCells(exa::VolumeLines::GridCell *grid,
                                      float *weights,
                                      int dims)
{
  int x = blockIdx.x * blockDim.x + threadIdx.x;

  if (x >= dims)
    return;

  if (weights[x] > 0.f)
    grid[x].value /= weights[x];
}

__global__ void postClassifyCells(exa::VolumeLines::GridCell *grid,
                                  int dims,
                                  const vec4f *colorMap,
                                  const int numColors,
                                  const range1f xfDomain)
{
  int x = blockIdx.x * blockDim.x + threadIdx.x;

  if (x >= dims)
    return;

  grid[x].color = lookupTransferFunction(grid[x].value,colorMap,numColors,xfDomain);
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

    for (size_t i=0; i<grids1D.size(); ++i) {
      cudaFree(grids1D[i]);
    }
  }

  void VolumeLines::reset(const ExaBrickModel::SP &model)
  {
    for (size_t i=0; i<cells.size(); ++i) {
      cudaFree(cells[i]);
    }

    cells.resize(1);
    cellBounds = {};
    centroidBounds = {};
    std::vector<Cell> cs;
    for (size_t i=0; i<model->bricks.size(); ++i) {
      const ExaBrick &brick = model->bricks[i];
      for (int z=0; z<brick.size.z; ++z) {
        for (int y=0; y<brick.size.y; ++y) {
          for (int x=0; x<brick.size.x; ++x) {
            int idx = brick.getIndexIndex({x,y,z});
            range1f vr = model->valueRange;
            #if 1
            float val = model->scalars[idx];
            #else
            float val = (model->scalars[idx]-vr.lower)/(vr.upper-vr.lower);
            #endif
            Cell c{idx,val,brick.level};
            cs.push_back(c);
            centroidBounds.extend(c.getBounds().center());
            cellBounds.extend(c.getBounds());
          }
        }
      }
    }

    #pragma omp parallel for
    for (size_t i=0; i<cs.size(); ++i) {
      Cell &c = cs[i];
      vec3i centroid = c.getBounds().center();
      vec3f centroid01(centroid);
      centroid01 = (centroid01-vec3f(centroidBounds.lower)) / vec3f(centroidBounds.upper-centroidBounds.lower);

      vec3f quantized(centroid01);
      quantized *= float(1<<16);
      const bitmask_t coord[3] = {
        bitmask_t(quantized.x),
        bitmask_t(quantized.y),
        bitmask_t(quantized.z)
      };
      c.hilbertID = hilbert_c2i(3, 16, coord);
    }

    std::sort(cs.begin(),cs.end(),
              [](const Cell &a, const Cell &b)
              { return a.hilbertID < b.hilbertID; }
            );

    cudaMalloc(&cells[0], cs.size()*sizeof(cs[0]));
    cudaMemcpy(cells[0], cs.data(), cs.size()*sizeof(cs[0]), cudaMemcpyHostToDevice);
    numCells = cs.size();
    updated_ = true;
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

    if (updated_ && xf.deviceColorMap) {
      for (size_t i=0; i<grids1D.size(); ++i) {
        cudaFree(grids1D[i]);
      }

      grids1D.clear();

      // raster cells onto 1D grids
      for (size_t i=0; i<cells.size(); ++i) {
        GridCell *grid;
        cudaMalloc(&grid, sizeof(GridCell)*w);
        cudaMemset(grid, 0, sizeof(GridCell)*w);

        float *weights;
        cudaMalloc(&weights, sizeof(float)*w);
        cudaMemset(weights, 0, sizeof(float)*w);

        // TODO: set per channel!
        range1f r{
         xf.absDomain.lower + (xf.relDomain.lower/100.f) * (xf.absDomain.upper-xf.absDomain.lower),
         xf.absDomain.lower + (xf.relDomain.upper/100.f) * (xf.absDomain.upper-xf.absDomain.lower)
        };

        size_t numThreads = 1024;
        basisRasterCells<<<iDivUp(numCells,numThreads),numThreads>>>(
          grid, weights, w, cells[i], numCells, cellBounds);

        grids1D.push_back(grid);

        basisAverageGridCells<<<iDivUp(w,numThreads),numThreads>>>(
          grid, weights, w);

        cudaFree(weights);

        postClassifyCells<<<iDivUp(numCells,numThreads),numThreads>>>(
          grid, w, xf.deviceColorMap, xf.colorMap.size(), r);

      }

      updated_ = false;
    }

    // render to texture
    for (size_t i=0; i<grids1D.size(); ++i) {
      size_t numThreads = 1024;
      renderGPU<<<iDivUp(w,numThreads),numThreads>>>(
        surfaceObj,grids1D[i],w,h);
    }
  }

  void VolumeLines::setColorMap(const std::vector<vec4f> &newCM)
  {
    xf.colorMap = newCM;

    cudaFree(xf.deviceColorMap);
    cudaMalloc(&xf.deviceColorMap, newCM.size()*sizeof(newCM[0]));
    cudaMemcpy(xf.deviceColorMap, newCM.data(), newCM.size()*sizeof(newCM[0]),
               cudaMemcpyHostToDevice);

    updated_ = true;
  }

  void VolumeLines::setRange(interval<float> xfDomain)
  {
    xf.absDomain = xfDomain;
    updated_ = true;
  }

  void VolumeLines::setRelDomain(interval<float> relDomain)
  {
    xf.relDomain = relDomain;
    updated_ = true;
  }

  void VolumeLines::setOpacityScale(float scale)
  {
    xf.opacityScale = scale;
    updated_ = true;
  }
} // ::exa
// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0
