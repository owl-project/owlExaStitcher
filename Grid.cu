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

#include "deviceCode.h"
#include "Grid.h"
#include "Grid.cuh"

using namespace owl;

namespace exa {

  // Lifted from https://github.com/treecode/Bonsai/blob/master/runtime/profiling/derived_atomic_functions.h
  __device__ __forceinline__ float atomicMin(float *address, float val)
  {
      int ret = __float_as_int(*address);
      while(val < __int_as_float(ret))
      {
          int old = ret;
          if((ret = atomicCAS((int *)address, old, __float_as_int(val))) == old)
              break;
      }
      return __int_as_float(ret);
  }
  
  __device__ __forceinline__ float atomicMax(float *address, float val)
  {
      int ret = __float_as_int(*address);
      while(val > __int_as_float(ret))
      {
          int old = ret;
          if((ret = atomicCAS((int *)address, old, __float_as_int(val))) == old)
              break;
      }
      return __int_as_float(ret);
  }


  inline int __both__ iDivUp(int a, int b)
  {
    return (a + b - 1) / b;
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

    const int *I = &indices[threadID*8];

    box3f cellBounds;
    range1f valueRange{+1e30f,-1e30f};

    for (int i=0; i<8; ++i) {
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
          atomicMin(&valueRanges[linearIndex(mcID,dims)].lower,valueRange.lower);
          atomicMax(&valueRanges[linearIndex(mcID,dims)].upper,valueRange.upper);
        }
      }
    }
  }

  // Gridlet overload
  __global__ void buildGrid(range1f       *valueRanges,
                            const Gridlet *gridlets,
                            const size_t   numGridlets,
                            const vec3i    dims,
                            const box3f    worldBounds)
  {
    size_t threadID = blockIdx.x * size_t(blockDim.x) + threadIdx.x;

    if (threadID >= numGridlets)
      return;

    const Gridlet &gridlet = gridlets[threadID];
    vec3i lower = gridlet.lower * (1<<gridlet.level);
    vec3i upper = lower + gridlet.dims * (1<<gridlet.level);

    const vec3f halfCell = vec3f(1<<gridlet.level)*.5f;

    const vec3f mcSize(worldBounds.size() / vec3f(dims));
    const vec3i loMC = projectOnGrid(vec3f(lower)+halfCell,dims,worldBounds);
    const vec3i upMC = projectOnGrid(vec3f(upper)+halfCell,dims,worldBounds);

    const vec3i numScalars = gridlet.dims+1;

    for (int mcz=loMC.z; mcz<=upMC.z; ++mcz) {
      for (int mcy=loMC.y; mcy<=upMC.y; ++mcy) {
        for (int mcx=loMC.x; mcx<=upMC.x; ++mcx) {
          const vec3i mcID(mcx,mcy,mcz);
          const box3f mcBounds(worldBounds.lower+vec3f(mcID)*mcSize,
                               worldBounds.lower+vec3f(mcID+1)*mcSize);

          for (int z=0; z<gridlet.dims.z; ++z) {
            for (int y=0; y<gridlet.dims.y; ++y) {
              for (int x=0; x<gridlet.dims.x; ++x) {
                const box3f cellBounds(vec3f((gridlet.lower+vec3i(x,y,z)) * (1<<gridlet.level))+halfCell,
                                       vec3f((gridlet.lower+vec3i(x+1,y+1,z+1)) * (1<<gridlet.level))+halfCell);
                if (mcBounds.overlaps(cellBounds)) {
                  vec3i imin(x,y,z);
                  vec3i imax(x+1,y+1,z+1);


                  float f1 = gridlet.scalars[linearIndex(vec3i(imin.x,imin.y,imin.z),numScalars)];
                  float f2 = gridlet.scalars[linearIndex(vec3i(imax.x,imin.y,imin.z),numScalars)];
                  float f3 = gridlet.scalars[linearIndex(vec3i(imin.x,imax.y,imin.z),numScalars)];
                  float f4 = gridlet.scalars[linearIndex(vec3i(imax.x,imax.y,imin.z),numScalars)];

                  float f5 = gridlet.scalars[linearIndex(vec3i(imin.x,imin.y,imax.z),numScalars)];
                  float f6 = gridlet.scalars[linearIndex(vec3i(imax.x,imin.y,imax.z),numScalars)];
                  float f7 = gridlet.scalars[linearIndex(vec3i(imin.x,imax.y,imax.z),numScalars)];
                  float f8 = gridlet.scalars[linearIndex(vec3i(imax.x,imax.y,imax.z),numScalars)];

                  range1f valueRange{+1e30f,-1e30f};

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

                  atomicMin(&valueRanges[linearIndex(mcID,dims)].lower,valueRange.lower);
                  atomicMax(&valueRanges[linearIndex(mcID,dims)].upper,valueRange.upper);
                }
              }
            }
          }
        }
      }
    }
  }

  // AMR cell overload
  __global__ void buildGrid(range1f       *valueRanges,
                            const AMRCell *amrCells,
                            const float   *amrScalars,
                            const size_t   numAmrCells,
                            const vec3i    dims,
                            const box3f    worldBounds)
  {
    size_t threadID = blockIdx.x * size_t(blockDim.x) + threadIdx.x;

    if (threadID >= numAmrCells)
      return;

    const AMRCell &cell = amrCells[threadID];

    const vec3f V = vec3f(cell.pos+vec3i(1<<cell.level) - (vec3i(1<<cell.level)/2));

    const vec3i mc = projectOnGrid(V,dims,worldBounds);

    const float value = amrScalars[threadID];

    if (!isnan(value)) {
      valueRanges[linearIndex(mc,dims)].lower = fminf(valueRanges[linearIndex(mc,dims)].lower,value);
      valueRanges[linearIndex(mc,dims)].upper = fmaxf(valueRanges[linearIndex(mc,dims)].upper,value);
    }
  }

  void Grid::build(OWLContext  owl,
                   OWLBuffer   vertices,
                   OWLBuffer   indices,
                   OWLBuffer   gridlets,
                   OWLBuffer   amrCells,
                   OWLBuffer   amrScalars,
                   const vec3i numMCs,
                   const box3f bounds)
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
      initGrid<<<iDivUp(numMCs, numThreads), numThreads>>>
        ((range1f *)owlBufferGetPointer(valueRanges,0),dims);
    }

    // Add contrib from uelems
    if (vertices && indices) {
      size_t numThreads = 1024;
      size_t numElems = owlBufferSizeInBytes(indices)/sizeof(int[8]);
      std::cout << "DDA grid: adding " << numElems << " uelems\n";
      buildGrid<<<iDivUp(numElems, numThreads), numThreads>>>(
        (range1f *)owlBufferGetPointer(valueRanges,0),
        (const vec4f *)owlBufferGetPointer(vertices,0),
        (const int *)owlBufferGetPointer(indices,0),
        numElems,dims,worldBounds);
      cudaDeviceSynchronize();
      std::cout << cudaGetErrorString(cudaGetLastError()) << '\n';
    }

    // Add contrib from gridlets
    if (gridlets) {
      size_t numThreads = 1024;
      size_t numGridlets = owlBufferSizeInBytes(gridlets)/sizeof(Gridlet);
      std::cout << "DDA grid: adding " << numGridlets << " gridlets\n";
      buildGrid<<<iDivUp(numGridlets, numThreads), numThreads>>>(
        (range1f *)owlBufferGetPointer(valueRanges,0),
        (const Gridlet *)owlBufferGetPointer(gridlets,0),
        numGridlets,dims,worldBounds);
      cudaDeviceSynchronize();
      std::cout << cudaGetErrorString(cudaGetLastError()) << '\n';
    }

    // Add contrib from AMR cells
    if (amrCells && amrScalars) {
      size_t numThreads = 1024;
      size_t numAmrCells = owlBufferSizeInBytes(amrCells)/sizeof(AMRCell);
      std::cout << "DDA grid: adding " << numAmrCells << " AMR cells (non-dual!)\n";
      buildGrid<<<iDivUp(numAmrCells, numThreads), numThreads>>>(
        (range1f *)owlBufferGetPointer(valueRanges,0),
        (const AMRCell *)owlBufferGetPointer(amrCells,0),
        (const float *)owlBufferGetPointer(amrScalars,0),
        numAmrCells,dims,worldBounds);
      cudaDeviceSynchronize();
      std::cout << cudaGetErrorString(cudaGetLastError()) << '\n';
    }
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
    int hi = clamp(int(valueRange.upper*(numColors-1)),0,(int)numColors-1);

    float maxOpacity = 0.f;
    for (int i=lo; i<=hi; ++i) {
      maxOpacity = fmaxf(maxOpacity,colorMap[i].w);
    }
    maxOpacities[threadID] = maxOpacity;
  }

  void Grid::computeMaxOpacities(OWLContext owl, OWLBuffer colorMap, range1f xfRange)
  {
    if (maxOpacities) {
      owlBufferDestroy(maxOpacities);
    }

    size_t numMCs = dims.x*size_t(dims.y)*dims.z;
    size_t numColors = owlBufferSizeInBytes(colorMap)/sizeof(vec4f);

    maxOpacities = owlDeviceBufferCreate(owl, OWL_FLOAT,
                                         numMCs,
                                         nullptr);

    size_t numThreads = 1024;
    computeMaxOpacitiesGPU<<<iDivUp(numMCs, numThreads), numThreads>>>(
      (float *)owlBufferGetPointer(maxOpacities,0),
      (const range1f *)owlBufferGetPointer(valueRanges,0),
      (const vec4f *)owlBufferGetPointer(colorMap,0),
      numMCs,numColors,xfRange);
  }

} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

