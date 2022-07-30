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
                            const size_t   numIndices,
                            const vec3i    dims,
                            const box3f    worldBounds)
  {
    size_t threadID = blockIdx.x * size_t(blockDim.x) + threadIdx.x;

    if (threadID >= numIndices || indices[threadID] < 0)
      return;

    const vec4f V = vertices[indices[threadID]];

    const vec3i mc = projectOnGrid(vec3f(V),dims,worldBounds);

    if (!isnan(V.w)) {
      valueRanges[linearIndex(mc,dims)].lower = fminf(valueRanges[linearIndex(mc,dims)].lower,V.w);
      valueRanges[linearIndex(mc,dims)].upper = fmaxf(valueRanges[linearIndex(mc,dims)].upper,V.w);
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
    const vec3i numScalars = gridlet.dims+1;
    const vec3f halfCell = vec3f(1<<gridlet.level)*.5f;

    for (int z=0; z<numScalars.z; ++z) {
      for (int y=0; y<numScalars.y; ++y) {
        for (int x=0; x<numScalars.x; ++x) {
          const vec3f V = vec3f((gridlet.lower+vec3i(x,y,z)) * (1<<gridlet.level))
                        + halfCell;

          const vec3i mc = projectOnGrid(V,dims,worldBounds);

          const float value = gridlet.scalars[linearIndex(vec3i(x,y,z),numScalars)];

          if (!isnan(value)) {
            valueRanges[linearIndex(mc,dims)].lower = fminf(valueRanges[linearIndex(mc,dims)].lower,value);
            valueRanges[linearIndex(mc,dims)].upper = fmaxf(valueRanges[linearIndex(mc,dims)].upper,value);
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
      size_t numIndices = owlBufferSizeInBytes(indices)/sizeof(int);
      std::cout << "DDA grid: adding " << numIndices << " uelems\n";
      buildGrid<<<iDivUp(numIndices, numThreads), numThreads>>>(
        (range1f *)owlBufferGetPointer(valueRanges,0),
        (const vec4f *)owlBufferGetPointer(vertices,0),
        (const int *)owlBufferGetPointer(indices,0),
        numIndices,dims,worldBounds);
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

