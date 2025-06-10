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

#include "sampler/AMRCellSampler.h"
#include "sampler/ExaBrickSampler.h"
#include "sampler/ExaStitchSampler.h"
#include "sampler/QuickClustersSampler.h"
#include "common.h"
#include "LaunchParams.h" // for MacroCellGeom (dependency should be removed eventually)
#include "Grid.h"
#include "Grid.cuh"
#include "atomicOp.cuh"

using namespace owl;

namespace exa {

  template<typename T>
  inline T __both__ iDivUp(T a, T b)
  {
    return (a + b - 1) / b;
  }

  template<typename T>
  inline T __both__ iRoundUp(T a, T b)
  {
    return iDivUp(a,b) * b;
  }

  __device__ inline void updateMC(const vec3i    mcID,
                                  const vec3i    gridDims,
                                  const range1f  valueRange,
                                  range1f       *valueRanges)
  {
    atomicMin(&valueRanges[linearIndex(mcID,gridDims)].lower,valueRange.lower);
    atomicMax(&valueRanges[linearIndex(mcID,gridDims)].upper,valueRange.upper);
  }

  __device__ inline void updateMC(const vec3i  mcID,
                                  const vec3i  gridDims,
                                  const float  value,
                                  range1f     *valueRanges)
  {
    updateMC(mcID,gridDims,range1f{value,value},valueRanges);
  }


  __global__ void initGrid(range1f *valueRanges, const vec3i dims)
  {
    size_t threadID = blockIdx.x * size_t(blockDim.x) + threadIdx.x;

    if (threadID >= dims.x*size_t(dims.y)*dims.z)
      return;

    valueRanges[threadID].lower = +1e30f;
    valueRanges[threadID].upper = -1e30f;
  }

  // UMesh overload
  template <int NumVertsMax=8>
  __global__ void buildGrid(range1f       *valueRanges,
                            const vec4f   *vertices,
                            const int     *indices,
                            const size_t   numElems,
                            const vec3i    dims,
                            const box3f    worldBounds)
  {
    size_t threadID = blockIdx.x * size_t(blockDim.x) + threadIdx.x;

    if (threadID >= numElems)
      return;

    const int *I = &indices[threadID*NumVertsMax];

    box3f cellBounds;
    range1f valueRange{+1e30f,-1e30f};

    for (int i=0; i<NumVertsMax; ++i) {
      if (I[i] < 0)
        break;

      const vec4f V = vertices[I[i]];
      if (!isnan(V.w)) {
        cellBounds.extend(vec3f(V));
        valueRange.lower = fminf(valueRange.lower,V.w);
        valueRange.upper = fmaxf(valueRange.upper,V.w);
      }
    }

    const vec3i loMC = projectOnGrid(cellBounds.lower,dims,worldBounds);
    const vec3i upMC = projectOnGrid(cellBounds.upper,dims,worldBounds);
    //printf("%i,%i,%i -- %i,%i,%i,\n",
    //       loMC.x,loMC.y,loMC.z,
    //       upMC.x,upMC.y,upMC.z);

    for (int mcz=loMC.z; mcz<=upMC.z; ++mcz) {
      for (int mcy=loMC.y; mcy<=upMC.y; ++mcy) {
        for (int mcx=loMC.x; mcx<=upMC.x; ++mcx) {
          const vec3i mcID(mcx,mcy,mcz);
          updateMC(mcID,dims,valueRange,valueRanges);
        }
      }
    }
  }

  // Gridlet overload
  __global__ void getMaxGridletSize(const Gridlet *gridlets,
                                    const size_t   numGridlets,
                                    vec3i         *maxGridletSize)
  {
    size_t threadID = blockIdx.x * size_t(blockDim.x) + threadIdx.x;

    if (threadID >= numGridlets)
      return;

    ::atomicMax(&maxGridletSize->x,gridlets[threadID].dims.x);
    ::atomicMax(&maxGridletSize->y,gridlets[threadID].dims.y);
    ::atomicMax(&maxGridletSize->z,gridlets[threadID].dims.z);
  }

  __global__ void buildGrid(range1f       *valueRanges,
                            const Gridlet *gridlets,
                            const float   *scalars,
                            const size_t   numGridlets,
                            const vec3i    maxGridletSize,
                            const vec3i    dims,
                            const box3f    worldBounds)
  {
    size_t gridletID = blockIdx.x * size_t(blockDim.x) + threadIdx.x;

    if (gridletID >= numGridlets)
      return;

    const Gridlet &gridlet = gridlets[gridletID];

    int xyz = blockIdx.y * blockDim.y + threadIdx.y;
    int x = xyz%maxGridletSize.x;
    int y = xyz/maxGridletSize.x%maxGridletSize.y;
    int z = xyz/(maxGridletSize.x*maxGridletSize.y);
    int dimX = min(maxGridletSize.x,gridlet.dims.x);
    int dimY = min(maxGridletSize.y,gridlet.dims.y);
    int dimZ = min(maxGridletSize.z,gridlet.dims.z);

    if (x >= dimX || y >= dimY || z >= dimZ)
      return;

    const box3f &gridletBounds = gridlet.getBounds();

    const vec3f mcSize(worldBounds.size() / vec3f(dims));
    const vec3i loMC = projectOnGrid(gridletBounds.lower,dims,worldBounds);
    const vec3i upMC = projectOnGrid(gridletBounds.upper,dims,worldBounds);

    const vec3i numScalars = gridlet.dims+1;

    for (int mcz=loMC.z; mcz<=upMC.z; ++mcz) {
      for (int mcy=loMC.y; mcy<=upMC.y; ++mcy) {
        for (int mcx=loMC.x; mcx<=upMC.x; ++mcx) {
          const vec3i mcID(mcx,mcy,mcz);
          const box3f mcBounds(worldBounds.lower+vec3f(mcID)*mcSize,
                               worldBounds.lower+vec3f(mcID+1)*mcSize);


          range1f valueRange{+1e30f,-1e30f};
          /*for (int z=0; z<gridlet.dims.z; ++z)*/ {
            /*for (int y=0; y<gridlet.dims.y; ++y)*/ {
              /*for (int x=0; x<gridlet.dims.x; ++x)*/ {
                const float cellWidth = 1<<gridlet.level;
                const box3f cellBounds(vec3f((gridlet.lower+vec3i(x,y,z)) * cellWidth) + .5f*cellWidth,
                                       vec3f((gridlet.lower+vec3i(x+1,y+1,z+1)) * cellWidth) + .5f*cellWidth);
                if (mcBounds.overlaps(cellBounds)) {
                  vec3i imin(x,y,z);
                  vec3i imax(x+1,y+1,z+1);


                  float f1 = scalars[gridlet.begin+linearIndex(vec3i(imin.x,imin.y,imin.z),numScalars)];
                  float f2 = scalars[gridlet.begin+linearIndex(vec3i(imax.x,imin.y,imin.z),numScalars)];
                  float f3 = scalars[gridlet.begin+linearIndex(vec3i(imin.x,imax.y,imin.z),numScalars)];
                  float f4 = scalars[gridlet.begin+linearIndex(vec3i(imax.x,imax.y,imin.z),numScalars)];

                  float f5 = scalars[gridlet.begin+linearIndex(vec3i(imin.x,imin.y,imax.z),numScalars)];
                  float f6 = scalars[gridlet.begin+linearIndex(vec3i(imax.x,imin.y,imax.z),numScalars)];
                  float f7 = scalars[gridlet.begin+linearIndex(vec3i(imin.x,imax.y,imax.z),numScalars)];
                  float f8 = scalars[gridlet.begin+linearIndex(vec3i(imax.x,imax.y,imax.z),numScalars)];

                  if (!isnan(f1)) {
                    valueRange.lower = fminf(valueRange.lower,f1);
                    valueRange.upper = fmaxf(valueRange.upper,f1);
                  }

                  if (!isnan(f2)) {
                    valueRange.lower = fminf(valueRange.lower,f2);
                    valueRange.upper = fmaxf(valueRange.upper,f2);
                  }

                  if (!isnan(f3))  {
                    valueRange.lower = fminf(valueRange.lower,f3);
                    valueRange.upper = fmaxf(valueRange.upper,f3);
                  }

                  if (!isnan(f4)) {
                    valueRange.lower = fminf(valueRange.lower,f4);
                    valueRange.upper = fmaxf(valueRange.upper,f4);
                  }

                  if (!isnan(f5)) {
                    valueRange.lower = fminf(valueRange.lower,f5);
                    valueRange.upper = fmaxf(valueRange.upper,f5);
                  }

                  if (!isnan(f6)) {
                    valueRange.lower = fminf(valueRange.lower,f6);
                    valueRange.upper = fmaxf(valueRange.upper,f6);
                  }

                  if (!isnan(f7)) {
                    valueRange.lower = fminf(valueRange.lower,f7);
                    valueRange.upper = fmaxf(valueRange.upper,f7);
                  }

                  if (!isnan(f8)) {
                    valueRange.lower = fminf(valueRange.lower,f8);
                    valueRange.upper = fmaxf(valueRange.upper,f8);
                  }

                }
              }
            }
          }
          updateMC(mcID,dims,valueRange,valueRanges);
        }
      }
    }
  }

  // AMR cell overload
  __global__ void buildGrid(range1f       *valueRanges,
                            const AMRCell *cells,
                            const float   *scalars,
                            const size_t   numAmrCells,
                            const vec3i    dims,
                            const box3f    worldBounds)
  {
    size_t threadID = blockIdx.x * size_t(blockDim.x) + threadIdx.x;

    if (threadID >= numAmrCells)
      return;

    const AMRCell &cell = cells[threadID];

    vec3i lower = cell.pos;
    vec3i upper = lower + (1<<cell.level);

    const vec3f halfCell = vec3f(1<<cell.level)*.5f;

    const vec3i loMC = projectOnGrid(vec3f(lower)-halfCell,dims,worldBounds);
    const vec3i upMC = projectOnGrid(vec3f(upper)+halfCell,dims,worldBounds);

    const float value = scalars[threadID];

    if (!isnan(value)) {
      for (int mcz=loMC.z; mcz<=upMC.z; ++mcz) {
        for (int mcy=loMC.y; mcy<=upMC.y; ++mcy) {
          for (int mcx=loMC.x; mcx<=upMC.x; ++mcx) {
            const vec3i mcID(mcx,mcy,mcz);
            updateMC(mcID,dims,value,valueRanges);
          }
        }
      }
    }
  }

  // ExaBrick overload
  __global__ void buildGrid(range1f      *valueRanges,
                            const ABR    *abrs,
                            const size_t  numABRs,
                            const vec3i   dims,
                            const box3f   worldBounds)
  {
    size_t threadID = blockIdx.x * size_t(blockDim.x) + threadIdx.x;

    if (threadID >= numABRs)
      return;

    const box3f domain = abrs[threadID].domain;
    const range1f valueRange = abrs[threadID].valueRange;

    const vec3f mcSize(worldBounds.size() / vec3f(dims));
    const vec3i loMC = projectOnGrid(domain.lower,dims,worldBounds);
    const vec3i upMC = projectOnGrid(domain.upper,dims,worldBounds);

    for (int mcz=loMC.z; mcz<=upMC.z; ++mcz) {
      for (int mcy=loMC.y; mcy<=upMC.y; ++mcy) {
        for (int mcx=loMC.x; mcx<=upMC.x; ++mcx) {
          const vec3i mcID(mcx,mcy,mcz);
          updateMC(mcID,dims,valueRange,valueRanges);
        }
      }
    }
  }

  void Grid::build(OWLContext         owl,
                   AMRCellSampler::SP sampler,
                   const owl::vec3i   numMCs,
                   const owl::box3f   bounds)
  {
    dims        = numMCs;
    worldBounds = bounds;

    valueRanges = owlDeviceBufferCreate(owl, OWL_USER_TYPE(range1f),
                                        dims.x*size_t(dims.y)*dims.z,
                                        nullptr);

    // Init with small floats
    {
      size_t numThreads = 1024;
      size_t numMCs = dims.x*size_t(dims.y)*dims.z;
      initGrid<<<(uint32_t)iDivUp(numMCs, numThreads), (uint32_t)numThreads>>>
        ((range1f *)owlBufferGetPointer(valueRanges,0),dims);

      // pre-allocating max-opacity buffer
      maxOpacities = owlDeviceBufferCreate(owl, OWL_FLOAT, numMCs, nullptr);
    }

    // Add contrib from AMR cells
    {
      size_t numThreads = 1024;
      size_t numAmrCells = owlBufferSizeInBytes(sampler->cellBuffer)/sizeof(AMRCell);
      std::cout << "DDA grid: adding " << numAmrCells << " AMR cells (non-dual!)\n";
      buildGrid<<<(uint32_t)iDivUp(numAmrCells, numThreads), (uint32_t)numThreads>>>(
        (range1f *)owlBufferGetPointer(valueRanges,0),
        (const AMRCell *)owlBufferGetPointer(sampler->cellBuffer,0),
        (const float *)owlBufferGetPointer(sampler->scalarBuffer,0),
        numAmrCells,dims,worldBounds);
      cudaDeviceSynchronize();
      std::cout << cudaGetErrorString(cudaGetLastError()) << '\n';
    }

    // init device traversable for DDA
#ifdef EXA_STITCH_MIRROR_EXAJET
    deviceTraversable.traversable.dims = dims;
    deviceTraversable.traversable.bounds = worldBounds;
#else
    deviceTraversable.dims = dims;
    deviceTraversable.bounds = worldBounds;
#endif

  }

  void Grid::build(OWLContext          owl,
                   ExaBrickSampler::SP sampler,
                   const owl::vec3i    numMCs,
                   const owl::box3f    bounds)
  {
    dims        = numMCs;
    worldBounds = bounds;

    valueRanges = owlDeviceBufferCreate(owl, OWL_USER_TYPE(range1f),
                                        dims.x*size_t(dims.y)*dims.z,
                                        nullptr);

    // Init with small floats
    {
      size_t numThreads = 1024;
      size_t numMCs = dims.x*size_t(dims.y)*dims.z;
      initGrid<<<(uint32_t)iDivUp(numMCs, numThreads), (uint32_t)numThreads>>>
        ((range1f *)owlBufferGetPointer(valueRanges,0),dims);

      // pre-allocating max-opacity buffer
      maxOpacities = owlDeviceBufferCreate(owl, OWL_FLOAT, numMCs, nullptr);
    }

    // Add contrib from ExaBricks
    if (0) { // ABR grid projection
      double tfirst = getCurrentTime();
      size_t numThreads = 1024;
      size_t numABRs = owlBufferSizeInBytes(sampler->abrBuffer)/sizeof(ABR);
      std::cout << "DDA grid: adding " << numABRs << " ExaBrick ABRs\n";
      buildGrid<<<(uint32_t)iDivUp(numABRs, numThreads), (uint32_t)numThreads>>>(
        (range1f *)owlBufferGetPointer(valueRanges,0),
        (const ABR *)owlBufferGetPointer(sampler->abrBuffer,0),
        numABRs,dims,worldBounds);

      std::cout << cudaGetErrorString(cudaGetLastError()) << '\n';
      double tlast = getCurrentTime();
      std::cout << tlast-tfirst << '\n';
    } else { // project cells; for now on the CPU
      std::vector<range1f> hValueRanges(dims.x*size_t(dims.y)*dims.z);
      std::fill(hValueRanges.begin(),
                hValueRanges.end(),
                range1f{1e30f,-1e30f});

      for (size_t i=0; i<sampler->model->bricks.size(); ++i) {
        const ExaBrick &brick = sampler->model->bricks[i];
        for (int z=0; z<brick.size.z; ++z) {
          for (int y=0; y<brick.size.y; ++y) {
            for (int x=0; x<brick.size.x; ++x) {
              vec3i index3(x,y,z);
              int idx = brick.getIndexIndex(index3);
              const float value = sampler->model->scalars[idx];
              if (!isfinite(value)) continue;

              vec3i lower = brick.lower + index3*(1<<brick.level);
              vec3i upper = lower + (1<<brick.level);

              const vec3f halfCell = vec3f((float)(1<<brick.level))*.5f;

              const vec3i loMC = projectOnGrid(vec3f(lower)-halfCell,dims,worldBounds);
              const vec3i upMC = projectOnGrid(vec3f(upper)+halfCell,dims,worldBounds);

              for (int mcz=loMC.z; mcz<=upMC.z; ++mcz) {
                for (int mcy=loMC.y; mcy<=upMC.y; ++mcy) {
                  for (int mcx=loMC.x; mcx<=upMC.x; ++mcx) {
                    const vec3i mcID(mcx,mcy,mcz);
                    hValueRanges[linearIndex(mcID,dims)].lower
                      = std::min(hValueRanges[linearIndex(mcID,dims)].lower,value);
                    hValueRanges[linearIndex(mcID,dims)].upper
                      = std::max(hValueRanges[linearIndex(mcID,dims)].upper,value);
                  }
                }
              }
            }
          }
        }

        std::cout << '(' << (i+1) << '/' << sampler->model->bricks.size() << ")\r";
      }

      owlBufferRelease(valueRanges);
      valueRanges = owlDeviceBufferCreate(owl, OWL_USER_TYPE(range1f),
                                          hValueRanges.size(),
                                          hValueRanges.data());
    }

    // init device traversable for DDA
#ifdef EXA_STITCH_MIRROR_EXAJET
    deviceTraversable.traversable.dims = dims;
    deviceTraversable.traversable.bounds = worldBounds;
#else
    deviceTraversable.dims = dims;
    deviceTraversable.bounds = worldBounds;
#endif

  }

  void Grid::build(OWLContext           owl,
                   ExaStitchSampler::SP sampler,
                   const owl::vec3i     numMCs,
                   const owl::box3f     bounds)
  {
    dims        = numMCs;
    worldBounds = bounds;

    valueRanges = owlDeviceBufferCreate(owl, OWL_USER_TYPE(range1f),
                                        dims.x*size_t(dims.y)*dims.z,
                                        nullptr);

    // Init with small floats
    {
      size_t numThreads = 1024;
      size_t numMCs = dims.x*size_t(dims.y)*dims.z;
      initGrid<<<(uint32_t)iDivUp(numMCs, numThreads), (uint32_t)numThreads>>>
        ((range1f *)owlBufferGetPointer(valueRanges,0),dims);

      // pre-allocating max-opacity buffer
      maxOpacities = owlDeviceBufferCreate(owl, OWL_FLOAT, numMCs, nullptr);
    }

    // Add contrib from uelems
#ifdef EXA_STITCH_SEPARATE_INDEX_BUFFERS_PER_UELEM
    for (int i=0; i<4; ++i) {
      if (sampler->vertexBuffer && sampler->indexBuffers[i])
      {
        if (owlBufferSizeInBytes(sampler->indexBuffers[i])==0)
          continue;

        size_t numThreads = 1024;
        // TODO: use some template magic here (?)
        if (i==0) {
          size_t numElems = owlBufferSizeInBytes(sampler->indexBuffers[i])/sizeof(int[4]);
          std::cout << "DDA grid: adding " << numElems << " uelems of type " << i << '\n';
          buildGrid<4><<<iDivUp(numElems, numThreads), numThreads>>>(
            (range1f *)owlBufferGetPointer(valueRanges,0),
            (const vec4f *)owlBufferGetPointer(sampler->vertexBuffer,0),
            (const int *)owlBufferGetPointer(sampler->indexBuffers[i],0),
            numElems,dims,worldBounds);
        }
        else if (i==1) {
          size_t numElems = owlBufferSizeInBytes(sampler->indexBuffers[i])/sizeof(int[5]);
          std::cout << "DDA grid: adding " << numElems << " uelems of type " << i << '\n';
          buildGrid<5><<<iDivUp(numElems, numThreads), numThreads>>>(
            (range1f *)owlBufferGetPointer(valueRanges,0),
            (const vec4f *)owlBufferGetPointer(sampler->vertexBuffer,0),
            (const int *)owlBufferGetPointer(sampler->indexBuffers[i],0),
            numElems,dims,worldBounds);
        }
        else if (i==2) {
          size_t numElems = owlBufferSizeInBytes(sampler->indexBuffers[i])/sizeof(int[6]);
          std::cout << "DDA grid: adding " << numElems << " uelems of type " << i << '\n';
          buildGrid<6><<<iDivUp(numElems, numThreads), numThreads>>>(
            (range1f *)owlBufferGetPointer(valueRanges,0),
            (const vec4f *)owlBufferGetPointer(sampler->vertexBuffer,0),
            (const int *)owlBufferGetPointer(sampler->indexBuffers[i],0),
            numElems,dims,worldBounds);
        }
        else if (i==3) {
          size_t numElems = owlBufferSizeInBytes(sampler->indexBuffers[i])/sizeof(int[8]);
          std::cout << "DDA grid: adding " << numElems << " uelems of type " << i << '\n';
          buildGrid<8><<<iDivUp(numElems, numThreads), numThreads>>>(
            (range1f *)owlBufferGetPointer(valueRanges,0),
            (const vec4f *)owlBufferGetPointer(sampler->vertexBuffer,0),
            (const int *)owlBufferGetPointer(sampler->indexBuffers[i],0),
            numElems,dims,worldBounds);
        }
        cudaDeviceSynchronize();
        std::cout << cudaGetErrorString(cudaGetLastError()) << '\n';
      }
    }
#else
    if (sampler->vertexBuffer && sampler->indexBuffer)
    {
      size_t numThreads = 1024;
      size_t numElems = owlBufferSizeInBytes(sampler->indexBuffer)/sizeof(int[8]);
      std::cout << "DDA grid: adding " << numElems << " uelems\n";
      buildGrid<8><<<iDivUp(numElems, numThreads), numThreads>>>(
        (range1f *)owlBufferGetPointer(valueRanges,0),
        (const vec4f *)owlBufferGetPointer(sampler->vertexBuffer,0),
        (const int *)owlBufferGetPointer(sampler->indexBuffer,0),
        numElems,dims,worldBounds);
      cudaDeviceSynchronize();
      std::cout << cudaGetErrorString(cudaGetLastError()) << '\n';
    }
#endif

    // Add contrib from gridlets
    if (sampler->gridletBuffer && sampler->gridletScalarBuffer)
    {
      double tfirst = getCurrentTime();
      size_t numThreads = 1024;
      size_t numGridlets = owlBufferSizeInBytes(sampler->gridletBuffer)/sizeof(Gridlet);
      std::cout << "DDA grid: adding " << numGridlets << " gridlets\n";
      vec3i *maxGridletSize;
      cudaMalloc(&maxGridletSize,sizeof(vec3i));
      vec3i init = 0;
      cudaMemcpy(maxGridletSize,&init,sizeof(init),cudaMemcpyHostToDevice);

      getMaxGridletSize<<<(uint32_t)iDivUp(numGridlets, numThreads), (uint32_t)numThreads>>>(
        (const Gridlet *)owlBufferGetPointer(sampler->gridletBuffer,0),
        numGridlets,maxGridletSize);

      vec3i hMaxGridletSize;
      cudaMemcpy(&hMaxGridletSize,maxGridletSize,sizeof(hMaxGridletSize),
                 cudaMemcpyDeviceToHost);

      dim3 gridDims((int)numGridlets,
                    (int)(hMaxGridletSize.x*hMaxGridletSize.y*hMaxGridletSize.z));
      dim3 blockDims(64,16);
      dim3 numBlocks(iDivUp(gridDims.x,blockDims.x),
                     iDivUp(gridDims.y,blockDims.y));

      buildGrid<<<numBlocks, blockDims>>>(
        (range1f *)owlBufferGetPointer(valueRanges,0),
        (const Gridlet *)owlBufferGetPointer(sampler->gridletBuffer,0),
        (const float *)owlBufferGetPointer(sampler->gridletScalarBuffer,0),
        numGridlets,hMaxGridletSize,dims,worldBounds);

      cudaFree(maxGridletSize);
      cudaDeviceSynchronize();
      std::cout << cudaGetErrorString(cudaGetLastError()) << '\n';
      double tlast = getCurrentTime();
      std::cout << tlast-tfirst << '\n';
    }

    // init device traversable for DDA
#ifdef EXA_STITCH_MIRROR_EXAJET
    deviceTraversable.traversable.dims = dims;
    deviceTraversable.traversable.bounds = worldBounds;
#else
    deviceTraversable.dims = dims;
    deviceTraversable.bounds = worldBounds;
#endif

  }

  void Grid::build(OWLContext           owl,
                   QuickClustersSampler::SP sampler,
                   const owl::vec3i     numMCs,
                   const owl::box3f     bounds)
  {
    dims        = numMCs;
    worldBounds = bounds;

    valueRanges = owlDeviceBufferCreate(owl, OWL_USER_TYPE(range1f),
                                        dims.x*size_t(dims.y)*dims.z,
                                        nullptr);

    // Init with small floats
    {
      size_t numThreads = 1024;
      size_t numMCs = dims.x*size_t(dims.y)*dims.z;
      initGrid<<<(uint32_t)iDivUp(numMCs, numThreads), (uint32_t)numThreads>>>
        ((range1f *)owlBufferGetPointer(valueRanges,0),dims);

      // pre-allocating max-opacity buffer
      maxOpacities = owlDeviceBufferCreate(owl, OWL_FLOAT, numMCs, nullptr);
    }

    // Add contrib from uelems
    {
      size_t numThreads = 1024;
      size_t numElems = owlBufferSizeInBytes(sampler->indexBuffer)/sizeof(int[8]);
      std::cout << "DDA grid: adding " << numElems << " uelems\n";
      buildGrid<<<(uint32_t)iDivUp(numElems, numThreads), (uint32_t)numThreads>>>(
        (range1f *)owlBufferGetPointer(valueRanges,0),
        (const vec4f *)owlBufferGetPointer(sampler->vertexBuffer,0),
        (const int *)owlBufferGetPointer(sampler->indexBuffer,0),
        numElems,dims,worldBounds);
      cudaDeviceSynchronize();
      std::cout << cudaGetErrorString(cudaGetLastError()) << '\n';
    }

    // init device traversable for DDA
#ifdef EXA_STITCH_MIRROR_EXAJET
    deviceTraversable.traversable.dims = dims;
    deviceTraversable.traversable.bounds = worldBounds;
#else
    deviceTraversable.dims = dims;
    deviceTraversable.bounds = worldBounds;
#endif

  }

  bool Grid::buildOptixBVH(OWLContext owl, OWLModule module)
  {
    // build BVH (tarversal method a)
    OWLVarDecl geomVars[]
    = {
       { "dims", OWL_INT3, OWL_OFFSETOF(MacroCellGeom,dims) },
       { "spacing", OWL_FLOAT3, OWL_OFFSETOF(MacroCellGeom,spacing) },
       { "origin", OWL_FLOAT3, OWL_OFFSETOF(MacroCellGeom,origin) },
       { "maxOpacities", OWL_BUFPTR, OWL_OFFSETOF(MacroCellGeom,maxOpacities) },
       { nullptr /* sentinel to mark end of list */ }
    };

    const vec3f spacing(worldBounds.size() / vec3f(dims));

    geomType = owlGeomTypeCreate(owl, OWL_GEOM_USER, sizeof(MacroCellGeom), geomVars, -1);
    owlGeomTypeSetBoundsProg   (geomType, module, "MacroCellGeomBounds");
    owlGeomTypeSetIntersectProg(geomType, RADIANCE_RAY_TYPE, module, "MacroCellGeomIsect");
    owlGeomTypeSetClosestHit   (geomType, RADIANCE_RAY_TYPE, module, "MacroCellGeomCH");
    OWLGeom geom = owlGeomCreate(owl, geomType);
    owlGeomSetPrimCount(geom, size_t(dims.x)*size_t(dims.y)*size_t(dims.z));
    owlGeomSet3i(geom,"dims", dims.x, dims.y, dims.z);
    owlGeomSet3f(geom,"spacing", spacing.x, spacing.y, spacing.z);
    owlGeomSet3f(geom,"origin", worldBounds.lower.x, worldBounds.lower.y, worldBounds.lower.z);
    owlGeomSetBuffer(geom,"maxOpacities", maxOpacities);

    owlBuildPrograms(owl);

    blas = owlUserGeomGroupCreate(owl, 1, &geom);
#ifdef EXA_STITCH_MIRROR_EXAJET
    owlGroupBuildAccel(blas);
    tlas = owlInstanceGroupCreate(owl, 2);
    owlInstanceGroupSetChild(tlas, 0, blas);
    owlInstanceGroupSetChild(tlas, 1, blas);
#else
    owlGroupBuildAccel(blas);
    tlas = owlInstanceGroupCreate(owl, 1);
    owlInstanceGroupSetChild(tlas, 0, blas);
    owlGroupBuildAccel(tlas);
#endif

    return true;
  }

  __global__ void computeMaxOpacitiesGPU(float         *maxOpacities,
                                         const range1f *valueRanges,
                                         const vec4f   *colorMap,
                                         size_t         numMCs,
                                         size_t         numColors,
                                         range1f        xfRange)
  {
    size_t threadID = blockIdx.x * size_t(blockDim.x) + threadIdx.x;

    if (threadID >= numMCs)
      return;

    range1f valueRange = valueRanges[threadID];

    if (valueRange.upper < valueRange.lower) {
      maxOpacities[threadID] = 0.f;
      return;
    }

    valueRange.lower -= xfRange.lower;
    valueRange.lower /= xfRange.upper-xfRange.lower;
    valueRange.upper -= xfRange.lower;
    valueRange.upper /= xfRange.upper-xfRange.lower;

    int lo = clamp(int(valueRange.lower*(numColors-1)),0,(int)numColors-1);
    int hi = clamp(int(valueRange.upper*(numColors-1))+1,0,(int)numColors-1);

    float maxOpacity = 0.f;
    for (int i=lo; i<=hi; ++i) {
      maxOpacity = fmaxf(maxOpacity,colorMap[i].w);
    }
    maxOpacities[threadID] = maxOpacity;
  }

  void Grid::computeMaxOpacities(OWLContext owl, OWLBuffer colorMap, range1f xfRange)
  {
    size_t numMCs = dims.x*size_t(dims.y)*dims.z;
    size_t numColors = owlBufferSizeInBytes(colorMap)/sizeof(vec4f);

    size_t numThreads = 1024;
    computeMaxOpacitiesGPU<<<(uint32_t)iDivUp(numMCs, numThreads), (uint32_t)numThreads>>>(
      (float *)owlBufferGetPointer(maxOpacities,0),
      (const range1f *)owlBufferGetPointer(valueRanges,0),
      (const vec4f *)owlBufferGetPointer(colorMap,0),
      numMCs,numColors,xfRange);

#if EXA_STITCH_EXA_BRICK_TRAVERSAL_MODE == MC_BVH_TRAVERSAL
    owlGroupBuildAccel(blas);
    owlGroupBuildAccel(tlas);
#endif
  }

} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

