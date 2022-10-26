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

#include <atomicOp.cuh>
#include "ExaStitchSampler.h"

namespace exa {

  inline int64_t __both__ iDivUp(int64_t a, int64_t b)
  {
    return (a + b - 1) / b;
  }

  static __global__ void initValueRangesGPU(range1f *valueRanges, size_t numRanges)
  {
    size_t threadID = blockIdx.x * size_t(blockDim.x) + threadIdx.x;

    if (threadID >= numRanges)
      return;

    valueRanges[threadID] = {1e30f,-1e30f};
  }

  // Gridlet overload
  static __global__ void getMaxGridletSize(const Gridlet *gridlets,
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

  static __global__ void computeGridletValueRangesGPU(range1f       *valueRanges,
                                                      const Gridlet *gridlets,
                                                      const float   *scalars,
                                                      const size_t   numGridlets,
                                                      const vec3i    maxGridletSize)
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

    const vec3i numScalars = gridlet.dims+1;

    range1f valueRange{+1e30f,-1e30f};
    /*for (int z=0; z<gridlet.dims.z; ++z)*/ {
      /*for (int y=0; y<gridlet.dims.y; ++y)*/ {
        /*for (int x=0; x<gridlet.dims.x; ++x)*/ {
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

    atomicMin(&valueRanges[gridletID].lower,valueRange.lower);
    atomicMax(&valueRanges[gridletID].upper,valueRange.upper);
  }

  static __global__ void computeGridletMaxOpacitiesGPU(float         *maxOpacities,
                                                       const range1f *valueRanges,
                                                       const vec4f   *colorMap,
                                                       size_t         numValueRanges,
                                                       size_t         numColors,
                                                       range1f        xfRange)
  {
    size_t threadID = blockIdx.x * size_t(blockDim.x) + threadIdx.x;
    if (threadID >= numValueRanges)
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

    int lo = clamp(int(valueRange.lower*(numColors-1)),  0,(int)numColors-1);
    int hi = clamp(int(valueRange.upper*(numColors-1))+1,0,(int)numColors-1);

    float maxOpacity = 0.f;
    for (int i=lo; i<=hi; ++i) {
      maxOpacity = fmaxf(maxOpacity,colorMap[i].w);
    }

    maxOpacities[threadID] = maxOpacity;
  }

  static __global__ void computeUmeshMaxOpacitiesGPU(float       *maxOpacities,
                                                     const vec4f *vertices,
                                                     const int   *indices,
                                                     size_t       numElements,
                                                     const vec4f *colorMap,
                                                     size_t       numColors,
                                                     range1f      xfRange)
  {
    size_t threadID = blockIdx.x * size_t(blockDim.x) + threadIdx.x;
    if (threadID >= numElements)
      return;

    vec4f v[8];
    int numVerts = 0;
    for (int i=0; i<8; ++i) {
      int idx = indices[threadID*8+i];
      if (idx >= 0) {
        numVerts++;
        v[i] = vertices[idx];
      }
    }

    range1f valueRange = {1e30f,-1e30f};

    for (int i=0; i<numVerts; ++i) {
      valueRange.lower = min(valueRange.lower,v[i].w);
      valueRange.upper = max(valueRange.upper,v[i].w);
    }

    if (valueRange.upper < valueRange.lower) {
      maxOpacities[threadID] = 0.f;
      return;
    }

    valueRange.lower -= xfRange.lower;
    valueRange.lower /= xfRange.upper-xfRange.lower;
    valueRange.upper -= xfRange.lower;
    valueRange.upper /= xfRange.upper-xfRange.lower;

    int lo = clamp(int(valueRange.lower*(numColors-1)),  0,(int)numColors-1);
    int hi = clamp(int(valueRange.upper*(numColors-1))+1,0,(int)numColors-1);

    float maxOpacity = 0.f;
    for (int i=lo; i<=hi; ++i) {
      maxOpacity = fmaxf(maxOpacity,colorMap[i].w);
    }

    maxOpacities[threadID] = maxOpacity;
  }

  void ExaStitchSampler::computeGridletValueRanges(OWLContext owl)
  {
    // init gridlet value ranges
    {
      size_t numThreads = 1024;

      initValueRangesGPU<<<iDivUp(model->gridlets.size(), numThreads), numThreads>>>(
        (range1f *)owlBufferGetPointer(gridletValueRanges,0),
        model->gridlets.size());
    }

    // compute vlaue ranges
    {
      double tfirst = getCurrentTime();
      size_t numThreads = 1024;
      size_t numGridlets = model->gridlets.size();
      std::cout << "DDA grid: adding " << numGridlets << " gridlets\n";
      vec3i *maxGridletSize;
      cudaMalloc(&maxGridletSize,sizeof(vec3i));
      vec3i init = 0;
      cudaMemcpy(maxGridletSize,&init,sizeof(init),cudaMemcpyHostToDevice);

      getMaxGridletSize<<<iDivUp(numGridlets, numThreads), numThreads>>>(
        (const Gridlet *)owlBufferGetPointer(gridletBuffer,0),
        numGridlets,maxGridletSize);

      vec3i hMaxGridletSize;
      cudaMemcpy(&hMaxGridletSize,maxGridletSize,sizeof(hMaxGridletSize),
                 cudaMemcpyDeviceToHost);

      dim3 gridDims(numGridlets,hMaxGridletSize.x*hMaxGridletSize.y*hMaxGridletSize.z);
      dim3 blockDims(64,16);
      dim3 numBlocks(iDivUp(gridDims.x,blockDims.x),
                     iDivUp(gridDims.y,blockDims.y));

      computeGridletValueRangesGPU<<<numBlocks, blockDims>>>(
        (range1f *)owlBufferGetPointer(gridletValueRanges,0),
        (const Gridlet *)owlBufferGetPointer(gridletBuffer,0),
        (const float *)owlBufferGetPointer(gridletScalarBuffer,0),
        numGridlets,hMaxGridletSize);

      cudaFree(maxGridletSize);
      cudaDeviceSynchronize();
      std::cout << cudaGetErrorString(cudaGetLastError()) << '\n';
      double tlast = getCurrentTime();
      std::cout << tlast-tfirst << '\n';
    }
  }

  void ExaStitchSampler::computeMaxOpacities(OWLContext owl, OWLBuffer colorMap, range1f xfRange)
  {
    if (!model->grid || model->grid->dims==vec3i(0))
      return;

    model->grid->computeMaxOpacities(owl,colorMap,xfRange);

    {
      size_t numColors = owlBufferSizeInBytes(colorMap)/sizeof(vec4f);
      size_t numThreads = 1024;

      computeGridletMaxOpacitiesGPU<<<iDivUp(model->gridlets.size(), numThreads), numThreads>>>(
        (float *)owlBufferGetPointer(gridletMaxOpacities,0),
        (const range1f *)owlBufferGetPointer(gridletValueRanges,0),
        (const vec4f *)owlBufferGetPointer(colorMap,0),
        model->gridlets.size(),numColors,xfRange);

      owlGroupBuildAccel(gridletGeom.blas);
      owlGroupBuildAccel(tlas);
    }

    {
      size_t numColors = owlBufferSizeInBytes(colorMap)/sizeof(vec4f);
      size_t numThreads = 1024;

      computeUmeshMaxOpacitiesGPU<<<iDivUp(model->indices.size()/8, numThreads), numThreads>>>(
        (float *)owlBufferGetPointer(umeshMaxOpacities,0),
        (const vec4f *)owlBufferGetPointer(vertexBuffer,0),
        (const int *)owlBufferGetPointer(indexBuffer,0),
        model->indices.size()/8,
        (const vec4f *)owlBufferGetPointer(colorMap,0),
        numColors,xfRange);

      owlGroupBuildAccel(stitchGeom.blas);
      owlGroupBuildAccel(tlas);
    }
  }
} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

