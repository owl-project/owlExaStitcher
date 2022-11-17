#include "QuickClustersSampler.h"

#include <owl/helper/cuda.h>

#include <cub/cub.cuh>

#include <hilbert.h>

namespace exa {
  
  //-------------------------------------------------------------------------------------------------
  // Stolen from https://github.com/treecode/Bonsai/blob/master/runtime/profiling/derived_atomic_functions.h
  //-------------------------------------------------------------------------------------------------

  static __device__ __forceinline__ float atomicMin(float *address, float val)
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

  static __device__ __forceinline__ float atomicMax(float *address, float val)
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

  //-------------------------------------------------------------------------------------------------
  // space filling curves
  //-------------------------------------------------------------------------------------------------

  __host__ __device__
  inline uint64_t morton64_encode4D(uint64_t x, uint64_t y, uint64_t z, uint64_t w)            
  {                                                                                          
      auto separate_bits = [](uint64_t n)                                                    
      {                                                                                      
          n &= 0b1111111111111111ull;                                                        
          n = (n ^ (n << 24)) & 0b0000000000000000000000001111111100000000000000000000000011111111ull;
          n = (n ^ (n << 12)) & 0b0000000000001111000000000000111100000000000011110000000000001111ull;
          n = (n ^ (n <<  6)) & 0b0000001100000011000000110000001100000011000000110000001100000011ull;
          n = (n ^ (n <<  3)) & 0b0001000100010001000100010001000100010001000100010001000100010001ull;
                                                                                             
          return n;                                                                          
      };                                                                                     
                                                                                             
      uint64_t xb = separate_bits(x);                                                        
      uint64_t yb = separate_bits(y) << 1;                                                   
      uint64_t zb = separate_bits(z) << 2;                                                   
      uint64_t wb = separate_bits(w) << 3;
      uint64_t code = xb | yb | zb | wb;                                                     
                                                                                             
      return code;                                                                           
  } 

  __host__ __device__
  inline uint64_t morton64_encode3D(float x, float y, float z)
  {
    x = x * (float)(1 << 16);
    y = y * (float)(1 << 16);
    z = z * (float)(1 << 16);
    auto separate_bits = [](uint64_t n)
    {
        n &= 0b1111111111111111111111ull;
        n = (n ^ (n << 32)) & 0b1111111111111111000000000000000000000000000000001111111111111111ull;
        n = (n ^ (n << 16)) & 0b0000000011111111000000000000000011111111000000000000000011111111ull;
        n = (n ^ (n <<  8)) & 0b1111000000001111000000001111000000001111000000001111000000001111ull;
        n = (n ^ (n <<  4)) & 0b0011000011000011000011000011000011000011000011000011000011000011ull;
        n = (n ^ (n <<  2)) & 0b1001001001001001001001001001001001001001001001001001001001001001ull;
        return n;
    };  

    return separate_bits(x) | (separate_bits(y) << 1) | (separate_bits(z) << 2); 
  }

  __host__ __device__
  inline uint64_t hilbert64_encode3D(float x, float y, float z)
  {
    x = x * (float)(1 << 16);
    y = y * (float)(1 << 16);
    z = z * (float)(1 << 16);
    const bitmask_t coord[3] = {bitmask_t(x), bitmask_t(y), bitmask_t(z)};
    return hilbert_c2i(3, 16, coord);
  }

  // #include "owl/common/math/random.h"
  // typedef owl::common::LCG<4> Random;
  //   __host__ __device__
  // inline uint64_t random_encode3D(float x, float y, float z)
  // {
  //   Random random(x, y * z * (1ull<<31ull));
  //   return random() * (1ull<<63ull);
  // }

  //-------------------------------------------------------------------------------------------------
  // whatever...
  //-------------------------------------------------------------------------------------------------

  template<typename T>
  static T __both__ iDivUp(T a, T b)
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
    const size_t threadID = blockIdx.x * size_t(blockDim.x) + threadIdx.x;
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
    {
      const uint32_t numColors = (uint32_t)owlBufferSizeInBytes(colorMap)/sizeof(vec4f);
      const uint32_t numThreads = 1024; // it seems CUDA kernel launch only accepts uint32_t, so no need for uint64_t

      computeUmeshMaxOpacitiesGPU<<<iDivUp((uint32_t)(model->indices.size()/8), numThreads), numThreads>>>(
        (float *)owlBufferGetPointer(umeshMaxOpacities,0),
        (const vec4f *)owlBufferGetPointer(vertexBuffer,0),
        (const int *)owlBufferGetPointer(indexBuffer,0),
        model->indices.size()/8,
        (const vec4f *)owlBufferGetPointer(colorMap,0),
        numColors, xfRange);

      owlGroupBuildAccel(leafGeom.blas);
      owlGroupBuildAccel(tlas);
    }
  }

  __global__ void computeCentroidsAndIndices(float4 *centroids, uint32_t *indices,
                                             const int   *indexBuffer, 
                                             const vec4f *vertexBuffer, 
                                             const uint64_t numElements)
  {
    const uint64_t threadID = blockIdx.x * (uint64_t)blockDim.x + threadIdx.x;
    const uint64_t primID = threadID;

    if (primID >= numElements) return;

    vec4f v[8];
    int numVerts = 0;
    for (int i=0; i<8; ++i) {
      int idx = indexBuffer[primID*8+i];
      if (idx >= 0) {
        numVerts++;
        v[i] = vertexBuffer[idx];
      }
    }

    box4f primBounds4 = box4f();
    for (int i = 0; i < numVerts; ++i) {
      primBounds4 = primBounds4.extend(v[i]);
    }

    const float4 pt = make_float4(
      (primBounds4.upper.x + primBounds4.lower.x) * .5f,
      (primBounds4.upper.y + primBounds4.lower.y) * .5f,
      (primBounds4.upper.z + primBounds4.lower.z) * .5f,
      (primBounds4.upper.w + primBounds4.lower.w) * .5f);
    
    centroids[primID] = pt;
    indices[primID] = primID;
  }

  __global__ void computeCentroidBounds(const float4* centroids, uint64_t N, box4f* centroidBoundsPtr)
  {
    const uint64_t index = blockIdx.x * (uint64_t)blockDim.x + threadIdx.x;

    if (index < N)
    {
      float4 pt = centroids[index];
      box4f& centroidBounds = *centroidBoundsPtr;

      // this assumes for an anisotropic bound, we still want isotropic bounding boxes.
      // this should improve on surface area of the splits...
      float mn = min(pt.x, min(pt.y, pt.z));
      float mx = max(pt.x, max(pt.y, pt.z));
      atomicMin(&centroidBounds.lower.x, mn);
      atomicMin(&centroidBounds.lower.y, mn);
      atomicMin(&centroidBounds.lower.z, mn);
      atomicMin(&centroidBounds.lower.w, pt.w);
      atomicMax(&centroidBounds.upper.x, mx);
      atomicMax(&centroidBounds.upper.y, mx);
      atomicMax(&centroidBounds.upper.z, mx);
      atomicMax(&centroidBounds.upper.w, pt.w);
    }
  }

  __global__ void assignCodes(uint64_t* codes, const float4* centroids, unsigned N, box4f* centroidBoundsPtr)
  {
    const uint64_t index = blockIdx.x * (uint64_t)blockDim.x + threadIdx.x;

    if (index < N)
    {
      const box4f& centroidBounds = *centroidBoundsPtr;

      // Project node to [0..1]
      vec4f pt = centroids[index];
      vec4f pt0 = pt;

      pt -= centroidBounds.center();
      pt = (pt + centroidBounds.size() * .5f) / centroidBounds.size();

      // edge case where all elements fall on the same plane, or have the same max data value
      if (centroidBounds.size().x == 0.f) pt.x = 0.f;
      if (centroidBounds.size().y == 0.f) pt.y = 0.f;
      if (centroidBounds.size().z == 0.f) pt.z = 0.f;
      if (centroidBounds.size().w == 0.f) pt.w = 0.f;

      // turns out including data value causes overlap which is really bad for performance...
      // uint64_t code = morton64_encode4D((uint64_t)pt.x, (uint64_t)pt.y, (uint64_t)pt.z, (uint64_t)pt.w);
      // uint64_t code = morton64_encode3D(pt.x, pt.y, pt.z);
      uint64_t code = hilbert64_encode3D(pt.x, pt.y, pt.z);
      // uint64_t code = random_encode3D(pt.x, pt.y, pt.z); // BAD, just a test...      
      codes[index] = code;
    }
  }

  void QuickClustersSampler::sortLeafPrimitives(uint64_t* &codesSorted, uint32_t* &elementIdsSorted)
  {
    const std::vector<int>   &indices  = model->indices;
    const std::vector<vec4f> &vertices = model->vertices;

    const int   *d_indices  = (const int*  )owlBufferGetPointer(indexBuffer,0);
    const vec4f *d_vertices = (const vec4f*)owlBufferGetPointer(vertexBuffer,0);

    const uint32_t numElements = (uint32_t)indices.size()/8;
    const uint32_t numThreads = 1024;

    // one centroid per element
    float4* centroids;
    cudaMalloc((void**)&centroids, numElements * sizeof(float4));
    box4f* centroidBounds;
    box4f emptyBounds = box4f();
    cudaMalloc((void**)&centroidBounds, sizeof(box4f));
    cudaMemcpy(centroidBounds, &emptyBounds, sizeof(box4f),cudaMemcpyHostToDevice);
    OWL_CUDA_SYNC_CHECK();

    // also a buffer of element indices we'll be sorting
    uint32_t* elementIdsUnsorted;
    cudaMalloc((void**)&elementIdsUnsorted, numElements * sizeof(uint32_t)); // lol, 32bit indices
    cudaMalloc((void**)&elementIdsSorted,   numElements * sizeof(uint32_t));

    // Compute element centroids and indices
    computeCentroidsAndIndices<<<iDivUp(numElements, numThreads), numThreads>>>(
      centroids, elementIdsUnsorted, d_indices, d_vertices, numElements
    );
    OWL_CUDA_SYNC_CHECK();

    // Compute centroid bounds 
    computeCentroidBounds<<<iDivUp(numElements, numThreads), numThreads>>>(
      centroids, numElements, centroidBounds
    );
    cudaMemcpy(&emptyBounds, centroidBounds, sizeof(box4f), cudaMemcpyDeviceToHost);
    printf("centroidBounds %f %f %f %f -- %f %f %f %f \n", 
           emptyBounds.lower.x, emptyBounds.lower.y, emptyBounds.lower.z, emptyBounds.lower.w,
           emptyBounds.upper.x, emptyBounds.upper.y, emptyBounds.upper.z, emptyBounds.upper.w);
    OWL_CUDA_SYNC_CHECK();

    // Project on morton curve
    uint64_t* codesUnsorted;
    cudaMalloc((void**)&codesUnsorted, numElements * sizeof(uint64_t));
    cudaMalloc((void**)&codesSorted, numElements * sizeof(uint64_t));
    {      
      assignCodes<<<iDivUp(numElements, numThreads), numThreads>>>(
        codesUnsorted,
        centroids,
        numElements,
        centroidBounds);
    }
    cudaFree(centroids);
    cudaFree(centroidBounds);
    OWL_CUDA_SYNC_CHECK();

    //
    // Sort code
    //

    // Determine temporary device storage requirements
    static size_t oldN = 0;

    // Allocate temporary storage
    static void *d_temp_storage = NULL;
    static size_t temp_storage_bytes = 0;
    if (oldN < numElements)
    {
      cub::DeviceRadixSort::SortPairs(d_temp_storage, temp_storage_bytes,
                                      codesUnsorted, codesSorted, elementIdsUnsorted, elementIdsSorted, numElements);

      if (d_temp_storage != nullptr) cudaFree(d_temp_storage);
      cudaMalloc(&d_temp_storage, temp_storage_bytes);
      oldN = numElements;
    }
    OWL_CUDA_SYNC_CHECK();

    // sort on morton curve
    cub::DeviceRadixSort::SortPairs(d_temp_storage, temp_storage_bytes,
                                    codesUnsorted, codesSorted, elementIdsUnsorted, elementIdsSorted, numElements);

    // cleanup
    cudaFree(codesUnsorted);
    cudaFree(elementIdsUnsorted);
    cudaFree(d_temp_storage);
  }

}
