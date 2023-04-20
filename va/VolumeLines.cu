#include <vector>
#include <cub/cub.cuh>
#include "VolumeLines.h"
#include "atomicOp.cuh"
#include "hilbert.h"

inline int64_t __host__ __device__ iDivUp(int64_t a, int64_t b)
{
  return (a + b - 1) / b;
}

inline __device__
const float* upper_bound(const float *first, const float *last, float val)
{
  const float* it;
  int count, step;
  count = (last-first);
  while (count > 0) {
    it = first;
    step=count/2;
    it = it + step;
    if (!(val < *it)) {
      first=++it;
      count-=step+1;
    }
    else count=step;
  }
  return first;
}

__global__ void fillGPU(cudaSurfaceObject_t surfaceObj, int w, int h,
                        float4 color1, float4 color2)
{
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.y * blockDim.y + threadIdx.y;
  if(x >= w || y >= h)
    return;

  if ((x/16) % 2 == (y/16) % 2)
    surf2Dwrite(color1, surfaceObj, x * sizeof(float4), h-y-1);
  else
    surf2Dwrite(color2, surfaceObj, x * sizeof(float4), h-y-1);
}

__global__ void renderBars(cudaSurfaceObject_t surfaceObj,
                           exa::VolumeLines::GridCell *grid, int w, int h)
{
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  
  if (x >= w)
    return;

  int H=owl::clamp(int(grid[x].color.w*(h-10)),0,h-1);
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

__global__ void renderLines(cudaSurfaceObject_t surfaceObj,
                            exa::VolumeLines::GridCell *grid, int w, int h)
{
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  
  if (x >= w)
    return;

  for (int y=0; y<h; ++y) {
    // line segment
    const vec2f p1(x,grid[x].color.w*(h-10));
    const vec2f p2(x+1,grid[x+1].color.w*(h-10));

    // pixel bounds
    const vec2f lower(x,y);
    const vec2f upper(x+1,y+1);

    // ray from line segment
    const vec2f dir = p2-p1;
    const float tmin = 0.f;
    const float tmax = sqrtf(dot(dir,dir));

    // slab test
    vec2f t_lo = lower - p1;
    vec2f t_hi = upper - p1;
    if (dir.x != 0.f && dir.y != 0.f) {
      t_lo /= dir;
      t_hi /= dir;
    }

    const vec2f t_nr = min(t_lo,t_hi);
    const vec2f t_fr = max(t_lo,t_hi);

    const float t0 = max(tmin,reduce_max(t_nr));
    const float t1 = min(tmax,reduce_min(t_fr));

    if (t0 < t1) {
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
}

__global__ void renderROIs(cudaSurfaceObject_t surfaceObj, int w, int h,
                           exa::VolumeLines::ROI *rois, size_t numROIs,
                           exa::VolumeLines::ROI specialROI,
                           float4 roiColor, float4 specialColor)
{
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  
  if (x >= w)
    return;

  bool overlaps = false;
  float4 color;

  if (specialROI.contains(x)) {
    overlaps = true;
    color = specialColor;
  } else {
    for (size_t i=0; i<numROIs; ++i) {
      if (rois[i].contains(x)) {
        color = roiColor;
        overlaps = true;
        break;
      }
    }
  }

  if (!overlaps)
    return;

  for (int y=0; y<h; ++y) {
    float4 src;
    surf2Dread(&src, surfaceObj, x * sizeof(float4), y);
    // over
    owl::vec4f A(color);
    owl::vec4f B(src);
    float4 dst = A + (1.f-A.w)*B;
    surf2Dwrite(dst, surfaceObj, x * sizeof(float4), y);
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

__global__ void assignImportance(exa::VolumeLines::Cell *cells,
                                 float *importance,
                                 int numCells,
                                 // for the moment use TF to assign importance..
                                 const vec4f *colorMap,
                                 const int numColors,
                                 const range1f xfDomain,
                                 float alphaMax,
                                 float minImportance,
                                 float P)
{
  int primID = blockIdx.x * blockDim.x + threadIdx.x;

  if (primID >= numCells)
    return;

  float cellWidth(1<<cells[primID].level);
  vec4f color = lookupTransferFunction(cells[primID].value,colorMap,numColors,xfDomain);
  // this will later become the diff of two time steps
  importance[primID] = fmaxf(minImportance, powf(color.w/alphaMax*cellWidth,P));
}

__global__ void basisRasterCells(exa::VolumeLines::GridCell *grid,
                                 float *weights,
                                 int dims,
                                 const exa::VolumeLines::Cell *cells,
                                 const float *cumulativeImportance,
                                 int numCells,
                                 range1f cellBounds)
{
  int primID = blockIdx.x * blockDim.x + threadIdx.x;

  if (primID >= numCells)
    return;

  float xf1 = primID == 0? 0.f : cumulativeImportance[primID-1];
  float xf2 = cumulativeImportance[primID];
  float maxImportance = cumulativeImportance[numCells-1];

  // Project onto grid
  float x1_01 = xf1/maxImportance;
  float x2_01 = xf2/maxImportance;

  int x1 = owl::clamp(int(x1_01*float(dims)),0,dims-1);
  int x2 = owl::clamp(int(x2_01*float(dims)),0,dims-1);

  // printf("%f,%f -> %f,%f -> %i,%i (%i): %f\n",
  //        xf1,xf2,x1_01,x2_01,x1,
  //        x2,x2-x1,cells[primID].value);

  for (int x=x1; x<=x2; ++x) {
    // originally we were using basis here
    // keeping the atomicAdd, but in theory these
    // should be exclusive memory accesses
    atomicAdd(&grid[x].value, cells[primID].value);
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

__global__ void computeHilbertROIsGPU(int dims,
                                      const exa::VolumeLines::ROI *rois,
                                      owl::interval<uint64_t> *roisToHilbertIDs,
                                      int numROIs,
                                      const exa::VolumeLines::Cell *cells,
                                      const float *cumulativeImportance,
                                      int numCells,
                                      range1f cellBounds)
{
  int roiID = blockIdx.x * blockDim.x + threadIdx.x;

  if (roiID >= numROIs)
    return;

  float maxImportance = cumulativeImportance[numCells-1];

  float x1_01 = rois[roiID].lower/float(dims);
  float x2_01 = rois[roiID].upper/float(dims);

  float xf1 = x1_01*maxImportance;
  float xf2 = x2_01*maxImportance;

  int first = -1;
  const float *it1 = upper_bound(cumulativeImportance,
                                 cumulativeImportance+numCells,
                                 xf1);
  first = int(it1-cumulativeImportance);
  //for (int i=0; i<numCells-1; ++i) {
  //  if (cumulativeImportance[i] <= xf1 && cumulativeImportance[i+1] > xf1) {
  //    first = i;
  //    break;
  //  }
  //}

  int last = -1;
  const float *it2 = upper_bound(cumulativeImportance,
                                 cumulativeImportance+numCells,
                                 xf2);
  last = int(it2-cumulativeImportance);
  //for (int i=0; i<numCells-1; ++i) {
  //  if (cumulativeImportance[i] <= xf2 && cumulativeImportance[i+1] > xf2) {
  //    last = i;
  //    break;
  //  }
  //}

  assert(first>=0 && last>=0);

  first = owl::clamp(first,0,numCells-1);
  last  = owl::clamp(last,0,numCells-1);
  //printf("%f,%f %i,%i\n",xf1,xf2,first,last);
  roisToHilbertIDs[roiID] = {cells[first].hilbertID,cells[last].hilbertID};
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
    cleanup();
  }

  void VolumeLines::cleanup()
  {
    for (size_t i=0; i<cells.size(); ++i) {
      cudaFree(cells[i]);
    }

    for (size_t i=0; i<grids1D.size(); ++i) {
      cudaFree(grids1D[i]);
    }

    cudaFree(importance.perCell);
    cudaFree(importance.cumulative);
    cudaFree(importance.tempStorage);
    importance.perCell = nullptr;
    importance.cumulative = nullptr;
    importance.tempStorage = nullptr;
  }

  void VolumeLines::reset(const ExaBrickModel::SP &model)
  {
    cleanup();

    cells.resize(1);
    map1Dto3D.clear();
    cellBounds = {};
    centroidBounds = {};
    cellBounds3D = {};
    centroidBounds3D = {};
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
            int lower = cs.empty() ? 0 : cs.back().getBounds().upper;
            Cell c{lower,val,brick.level};
            cs.push_back(c);
            centroidBounds.extend(c.getBounds().center());
            cellBounds.extend(c.getBounds());

            // also 3D!
            vec3i pos3D(brick.lower + vec3i(x,y,z)*(1<<brick.level));
            box3i bounds3D(pos3D,pos3D+vec3i(1<<brick.level));
            centroidBounds3D.extend(vec3f(bounds3D.center()));
            cellBounds3D.extend(box3f(vec3f(bounds3D.lower),vec3f(bounds3D.upper)));

            map1Dto3D.push_back({cs.size()-1, // 1D index
                                i, // brickID
                                {x,y,z},
                                bounds3D.center()});
          }
        }
      }
    }

    #pragma omp parallel for
    for (size_t i=0; i<cs.size(); ++i) {
      Cell &c = cs[i];
      vec3i centroid = map1Dto3D[i].centroid;
      vec3f centroid01(centroid);
      centroid01 = (centroid01-vec3f(centroidBounds3D.lower)) / vec3f(centroidBounds3D.upper-centroidBounds3D.lower);

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
    canvasSize = {w,h};

    // Fill background
    {
      dim3 blockSize;
      blockSize.x = 16;
      blockSize.y = 16;
      
      dim3 gridSize;
      gridSize.x = iDivUp(w,blockSize.x);
      gridSize.y = iDivUp(w,blockSize.y);

      float4 c1, c2;
      if (mode == Bars) {
        c1 = make_float4(.4f,.4f,.4f,1.f);
        c2 = make_float4(.3f,.3f,.3f,1.f);
      } else {
        c1 = c2 = make_float4(1,1,1,1);
      }
      fillGPU<<<gridSize, blockSize>>>(surfaceObj,w,h,c1,c2);
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

        float alphaMax = 0.f;
        for (size_t j=0; j<xf.colorMap.size(); ++j) {
          float alpha = xf.colorMap[j].w;
          alpha -= r.lower;
          alpha /= (r.upper-r.lower);
          alphaMax = fmaxf(alphaMax,alpha);
        }

        size_t numThreads = 1024;

        if (!importance.perCell) {
          cudaMalloc(&importance.perCell, sizeof(float)*numCells);
          cudaMalloc(&importance.cumulative, sizeof(float)*numCells);
          cub::DeviceScan::InclusiveSum(importance.tempStorage,
                                        importance.tempStorageSizeInBytes,
                                        importance.perCell,
                                        importance.cumulative,
                                        numCells);
          cudaMalloc(&importance.tempStorage, importance.tempStorageSizeInBytes);
        }

        assignImportance<<<iDivUp(numCells,numThreads),numThreads>>>(
          cells[i], importance.perCell, numCells, xf.deviceColorMap, xf.colorMap.size(),
          r, alphaMax, minImportance, P);

        cub::DeviceScan::InclusiveSum(importance.tempStorage,
                                      importance.tempStorageSizeInBytes,
                                      importance.perCell,
                                      importance.cumulative,
                                      numCells);

        basisRasterCells<<<iDivUp(numCells,numThreads),numThreads>>>(
          grid, weights, w, cells[i], importance.cumulative, numCells, cellBounds);

        grids1D.push_back(grid);

        basisAverageGridCells<<<iDivUp(w,numThreads),numThreads>>>(
          grid, weights, w);

        cudaFree(weights);

        postClassifyCells<<<iDivUp(numCells,numThreads),numThreads>>>(
          grid, w, xf.deviceColorMap, xf.colorMap.size(), r);

      }

      rois.clear();
      updated_ = false;
    }

    // render to texture
    for (size_t i=0; i<grids1D.size(); ++i) {
      size_t numThreads = 1024;
      if (mode == Bars) {
        renderBars<<<iDivUp(w,numThreads),numThreads>>>(
          surfaceObj,grids1D[i],w,h);
      } else {
        renderLines<<<iDivUp(w,numThreads),numThreads>>>(
          surfaceObj,grids1D[i],w,h);
      }
    }

    if (highlight != ROI{-1,-1} || !rois.empty()) {
      float4 roiColor = mode == Bars ? make_float4(0.5f,0.5f,0.5f,0.5f) // dark "theme"
                                     : make_float4(0.2f,0.2f,0.2f,0.5f); // light "theme"
      float4 specialColor = make_float4(0.5f,0.2f,0.1f,0.5f);
      ROI *d_rois;
      cudaMalloc(&d_rois,rois.size()*sizeof(rois[0]));
      cudaMemcpy(d_rois,rois.data(),rois.size()*sizeof(rois[0]),
                 cudaMemcpyHostToDevice);
      size_t numThreads = 1024;
      renderROIs<<<iDivUp(w,numThreads),numThreads>>>(
        surfaceObj,w,h,d_rois,rois.size(),highlight,
        roiColor,specialColor);
      cudaFree(d_rois);
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

  void VolumeLines::setMinImportance(float mi)
  {
    minImportance = mi;
    updated_ = true;
  }

  void VolumeLines::setP(float P)
  {
    this->P = P;
    updated_ = true;
  }

  void VolumeLines::setMode(VolumeLines::Mode m)
  {
    mode = m;
    updated_ = true;
  }

  void VolumeLines::computeHilbertROIs()
  {
    ROI *d_rois;
    cudaMalloc(&d_rois,rois.size()*sizeof(rois[0]));
    cudaMemcpy(d_rois,rois.data(),rois.size()*sizeof(rois[0]),
               cudaMemcpyHostToDevice);

    owl::interval<uint64_t> *d_roisToHilbertIDs;
    roisToHilbertIDs.resize(rois.size());
    cudaMalloc(&d_roisToHilbertIDs,roisToHilbertIDs.size()*sizeof(roisToHilbertIDs[0]));

    size_t numThreads = 1024;
    computeHilbertROIsGPU<<<iDivUp(rois.size(),numThreads),numThreads>>>(
      canvasSize.x,d_rois,d_roisToHilbertIDs,(int)rois.size(),
      cells[0],importance.cumulative,numCells,cellBounds);

    cudaMemcpy(roisToHilbertIDs.data(),d_roisToHilbertIDs,
               roisToHilbertIDs.size()*sizeof(roisToHilbertIDs[0]),
               cudaMemcpyDeviceToHost);

    cudaFree(d_roisToHilbertIDs);
    cudaFree(d_rois);

    worldSpaceROIs.resize(roisToHilbertIDs.size());

    for (size_t i=0; i<roisToHilbertIDs.size(); ++i) {
      bitmask_t coord1[3], coord2[3];
      hilbert_i2c(3, 16, roisToHilbertIDs[i].lower, coord1);
      hilbert_i2c(3, 16, roisToHilbertIDs[i].upper, coord2);

      const vec3f c1_01(
        coord1[0] / float(1<<16),
        coord1[1] / float(1<<16),
        coord1[2] / float(1<<16)
      );

      const vec3f c2_01(
        coord2[0] / float(1<<16),
        coord2[1] / float(1<<16),
        coord2[2] / float(1<<16)
      );

      const vec3f p1 = c1_01*vec3f(centroidBounds3D.upper-centroidBounds3D.lower)+vec3f(centroidBounds3D.lower);
      const vec3f p2 = c2_01*vec3f(centroidBounds3D.upper-centroidBounds3D.lower)+vec3f(centroidBounds3D.lower);

      worldSpaceROIs[i].extend(p1);
      worldSpaceROIs[i].extend(p2);
      std::cout << worldSpaceROIs[i] << '\n';
    }
  }

  void VolumeLines::onMouseMove(int x, int y, int button)
  {
    pressX = -1;
    highlight = {-1,-1};
  }

  void VolumeLines::onMouseDrag(int x, int y, int button)
  {
    if (pressX >= 0 && x != pressX) {
      highlight.lower = std::min(pressX,x);
      highlight.upper = std::max(pressX,x);
    } else {
      highlight = {-1,-1};
    }
  }

  void VolumeLines::onMousePress(int x, int y, int button)
  {
    if (button != 0)
      return;

    int found = -1;
    for (size_t i=0; i<rois.size(); ++i) {
      if (rois[i].contains(x))
        found = (int)i;
    }

    if (found >= 0)
      rois.erase(rois.begin()+found);

    pressX = x;
    highlight = {-1,-1};
  }

  void VolumeLines::onMouseRelease(int x, int y, int button)
  {
    if (highlight != ROI{-1,-1}) {
      for (size_t i=0; i<rois.size(); ) {
        ROI roi = rois[i];
        if (highlight.contains(rois[i].lower) || highlight.contains(rois[i].upper)) {
          rois.erase(rois.begin()+i);
          highlight.extend(roi);
        } else i++;
      }
      rois.push_back(highlight);

      computeHilbertROIs();
    }

    pressX = -1;
    highlight = {-1,-1};
  }

} // ::exa
// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0
