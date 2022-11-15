#include "QuickClustersSampler.h"


namespace exa {

  inline int64_t __both__ iDivUp(int64_t a, int64_t b)
  {
    return (a + b - 1) / b;
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

  void QuickClustersSampler::computeMaxOpacities(OWLContext owl, OWLBuffer colorMap, range1f xfRange)
  {
    if (!model->grid || model->grid->dims==vec3i(0))
      return;

    model->grid->computeMaxOpacities(owl,colorMap,xfRange);

    // {
    //   size_t numColors = owlBufferSizeInBytes(colorMap)/sizeof(vec4f);
    //   size_t numThreads = 1024;
    // 
    //   computeGridletMaxOpacitiesGPU<<<iDivUp(model->gridlets.size(), numThreads), numThreads>>>(
    //     (float *)owlBufferGetPointer(gridletMaxOpacities,0),
    //     (const range1f *)owlBufferGetPointer(gridletValueRanges,0),
    //     (const vec4f *)owlBufferGetPointer(colorMap,0),
    //     model->gridlets.size(),numColors,xfRange);
    // 
    //   owlGroupBuildAccel(gridletGeom.blas);
    //   owlGroupBuildAccel(tlas);
    // }

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

      owlGroupBuildAccel(leafGeom.blas);
      owlGroupBuildAccel(tlas);
    }
  }

}
