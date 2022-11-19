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
  T __both__ div_round_up(T a, T b)
  {
    return (a + b - 1) / b;
  }

  template <typename T>
  __both__ T next_multiple(T val, T divisor) {
    return div_round_up(val, divisor) * divisor;
  }

  constexpr uint32_t n_threads_linear = 1024;

  template <typename T>
  constexpr uint32_t n_blocks_linear(T n_elements) {
    return (uint32_t)div_round_up(n_elements, (T)n_threads_linear);
  }

  template <typename K, typename T, typename ... Types>
  inline void linear_kernel(K kernel, T n_elements, Types ... args) {
    if (n_elements <= 0) {
      return;
    }
    kernel<<<n_blocks_linear(n_elements), n_threads_linear, /*shmem_size=*/0, /*stream=*/0>>>(n_elements, args...);
  }

  template <typename F>
  __global__ void parallel_for_kernel(const size_t n_elements, F fun) {
    const size_t i = threadIdx.x + size_t(blockIdx.x) * blockDim.x;
    if (i >= n_elements) return;

    fun(i);
  }

  template <typename F>
  inline void parallel_for_gpu(uint32_t shmem_size, cudaStream_t stream, size_t n_elements, F&& fun) {
    if (n_elements <= 0) {
      return;
    }
    parallel_for_kernel<F><<<n_blocks_linear(n_elements), n_threads_linear, shmem_size, stream>>>(n_elements, fun);
  }

  template <typename F>
  inline void parallel_for_gpu(cudaStream_t stream, size_t n_elements, F&& fun) {
    parallel_for_gpu(0, stream, n_elements, std::forward<F>(fun));
  }

  template <typename F>
  inline void parallel_for_gpu(size_t n_elements, F&& fun) {
    parallel_for_gpu(nullptr, n_elements, std::forward<F>(fun));
  }

  //-------------------------------------------------------------------------------------------------
  // useful stuff
  //-------------------------------------------------------------------------------------------------

  static __global__ void computeUmeshMaxOpacitiesGPU(const size_t numElements,
                                                     float       * __restrict__ maxOpacities,
                                                     const vec4f * __restrict__ vertices,
                                                     const int   * __restrict__ indices,
                                                     const vec4f * __restrict__ colorMap,
                                                     const size_t  numColors,
                                                     const range1f xfRange)
  {
    const size_t threadID = blockIdx.x * size_t(blockDim.x) + threadIdx.x;
    if (threadID >= numElements)
      return;

    range1f valueRange = {1e30f,-1e30f};

    #pragma unroll
    for (int i=0; i<8; ++i) {
      int idx = indices[threadID*8+i];
      if (idx >= 0) {
        const vec4f v = vertices[idx];
        valueRange.lower = min(valueRange.lower,v.w);
        valueRange.upper = max(valueRange.upper,v.w);
      }
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
      linear_kernel(computeUmeshMaxOpacitiesGPU,
        model->indices.size()/8,
        (float       *)owlBufferGetPointer(umeshMaxOpacities,0),
        (const vec4f *)owlBufferGetPointer(vertexBuffer,0),
        (const int   *)owlBufferGetPointer(indexBuffer,0),
        (const vec4f *)owlBufferGetPointer(colorMap,0),
        owlBufferSizeInBytes(colorMap)/sizeof(vec4f), xfRange);

      // const uint32_t numColors = (uint32_t)owlBufferSizeInBytes(colorMap)/sizeof(vec4f);
      // const uint32_t numThreads = 1024; // it seems CUDA kernel launch only accepts uint32_t, so no need for uint64_t
      // 
      // computeUmeshMaxOpacitiesGPU<<<div_round_up((uint32_t)(model->indices.size()/8), numThreads), numThreads>>>(
      //   model->indices.size()/8,
      //   (float *)owlBufferGetPointer(umeshMaxOpacities,0),
      //   (const vec4f *)owlBufferGetPointer(vertexBuffer,0),
      //   (const int *)owlBufferGetPointer(indexBuffer,0),
      //   (const vec4f *)owlBufferGetPointer(colorMap,0),
      //   numColors, xfRange);

      owlGroupBuildAccel(leafGeom.blas);
      owlGroupBuildAccel(tlas);
    }
  }

  __global__ void computeCentroidsAndIndices(const uint64_t numElements,
                                             float4 * __restrict__ primcentroids, 
                                             uint32_t * __restrict__ primIndices,
                                             const int * __restrict__ indexBuffer, 
                                             const vec4f * __restrict__ vertexBuffer)
  {
    const uint64_t threadID = blockIdx.x * (uint64_t)blockDim.x + threadIdx.x;
    const uint64_t primID = threadID;

    if (primID >= numElements) return;

    // vec4f v[8];
    // int numVerts = 0;
    // for (int i=0; i<8; ++i) {
    //   int idx = indexBuffer[primID*8+i];
    //   if (idx >= 0) {
    //     numVerts++;
    //     v[i] = vertexBuffer[idx];
    //   }
    // }

    box4f primBounds4 = box4f();

    #pragma unroll
    for (int i=0; i<8; ++i) {
      const int idx = indexBuffer[primID*8+i];
      if (idx >= 0) {
        const vec4f v = vertexBuffer[idx];
        primBounds4 = primBounds4.extend(v);
      }
    }

    const float4 pt = make_float4(
      (primBounds4.upper.x + primBounds4.lower.x) * .5f,
      (primBounds4.upper.y + primBounds4.lower.y) * .5f,
      (primBounds4.upper.z + primBounds4.lower.z) * .5f,
      (primBounds4.upper.w + primBounds4.lower.w) * .5f);
    
    primcentroids[primID] = pt;
    primIndices[primID] = primID;
  }

  __global__ void computeCentroidBounds(const uint64_t N, const float4* __restrict__ centroids, box4f* __restrict__ centroidBoundsPtr)
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

  __global__ void assignCodes(const uint64_t N, uint64_t* __restrict__ codes, 
                              const float4* __restrict__ centroids, 
                              const box4f*  __restrict__ centroidBoundsPtr)
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

  void sortElements(const size_t numElements, 
                    const vec4f *d_vertices, 
                    const int *d_indices,
                    uint64_t** d_sortedElementCodes, 
                    uint32_t** d_sortedElementIDs)
  {
    uint64_t *&codesSorted = *d_sortedElementCodes;
    uint32_t* &elementIdsSorted = *d_sortedElementIDs;

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
    linear_kernel(computeCentroidsAndIndices, numElements, centroids, elementIdsUnsorted, d_indices, d_vertices);
    OWL_CUDA_SYNC_CHECK();

    // Compute centroid bounds 
    linear_kernel(computeCentroidBounds, numElements, centroids, centroidBounds);
    cudaMemcpy(&emptyBounds, centroidBounds, sizeof(box4f), cudaMemcpyDeviceToHost);
    printf("centroidBounds %f %f %f %f -- %f %f %f %f \n", 
           emptyBounds.lower.x, emptyBounds.lower.y, emptyBounds.lower.z, emptyBounds.lower.w,
           emptyBounds.upper.x, emptyBounds.upper.y, emptyBounds.upper.z, emptyBounds.upper.w);
    OWL_CUDA_SYNC_CHECK();

    // Project on morton curve
    uint64_t* codesUnsorted;
    cudaMalloc((void**)&codesUnsorted, numElements * sizeof(uint64_t));
    cudaMalloc((void**)&codesSorted, numElements * sizeof(uint64_t));     
    linear_kernel(assignCodes, numElements, codesUnsorted, centroids, centroidBounds);
    cudaFree(centroids);
    cudaFree(centroidBounds);
    OWL_CUDA_SYNC_CHECK();

    //
    // Sort code
    //

    // Allocate temporary storage
    static size_t oldN = 0; // Determine temporary device storage requirements
    static void *d_temp_storage = NULL;
    static size_t temp_storage_bytes = 0;
    if (oldN < numElements)
    {
      cub::DeviceRadixSort::SortPairs(d_temp_storage, temp_storage_bytes,
                                      codesUnsorted, codesSorted, 
                                      elementIdsUnsorted, elementIdsSorted, 
                                      (int)numElements);
      if (d_temp_storage != nullptr) cudaFree(d_temp_storage);
      cudaMalloc(&d_temp_storage, temp_storage_bytes);
      oldN = numElements;
    }
    OWL_CUDA_SYNC_CHECK();

    // sort on morton curve
    cub::DeviceRadixSort::SortPairs(d_temp_storage, temp_storage_bytes,
                                    codesUnsorted, codesSorted, 
                                    elementIdsUnsorted, elementIdsSorted, 
                                    (int)numElements);
    OWL_CUDA_SYNC_CHECK();

    // cleanup
    cudaFree(codesUnsorted);
    cudaFree(elementIdsUnsorted);
    cudaFree(d_temp_storage);
  }

  void reorderElements(const size_t numElements, 
                       const uint32_t *d_sortedElementIDs, 
                       const vec4f    *d_vertices, 
                       const int      *d_indices,
                       int **_sortedIndices)
  {
    int *&d_sortedIndices = *_sortedIndices;
    cudaMalloc((void**)&d_sortedIndices, numElements * sizeof(int) * 8);

    parallel_for_gpu(numElements,
    [
      N=numElements,
      sortedElementIDs=d_sortedElementIDs, 
      sortedIndices=(int *)d_sortedIndices,
      indexBuffer=d_indices
    ] 
    __device__ (size_t oldID) {
      const uint32_t newID = sortedElementIDs[oldID];

      #pragma unroll
      for (int i=0; i<8; ++i) {
        const int idx = indexBuffer[oldID*8+i];
        sortedIndices[newID*8+i] = idx;
      }
    });
  }

  __global__ void mergeClusters(uint64_t numElements,
                                uint32_t numClusters,
                                uint32_t* __restrict__ flags,
                                const uint64_t* __restrict__ codes,
                                uint32_t* __restrict__ elementToCluster,
                                uint32_t* __restrict__ elementsInClusters,
                                uint32_t maxElementsPerCluster)
  {
    uint32_t index = blockIdx.x * blockDim.x + threadIdx.x;   
    if (index >= numElements) return;

    // never clear the first cluster (should also always be an even cluster... optional)
    if (index == 0) return; 
    // if not the start of the cluster, return
    if (!flags[index]) return;
    uint32_t clusterID = elementToCluster[index];
    // return if we are not an odd cluster ID (odd's get merged into evens)
    if ((clusterID % 2) != 1) return;
    // figure out how many elements are in this cluster...
    uint32_t count = elementsInClusters[clusterID];
    // ... also how many elements were in the previous cluster...
    uint32_t prevCount = elementsInClusters[clusterID - 1];

    bool tooMany = count + prevCount > maxElementsPerCluster;
    if (tooMany) return;

    // combine this cluster with previous
    flags[index] = 0;
  }

  void buildClusters(const size_t    numElements,
                     const uint64_t* d_sortedCodes, 
                     const uint32_t  maxNumClusters,
                     const uint32_t  maxElementsPerCluster,
                     uint32_t*  _numClusters, 
                     uint32_t** d_sortedIndexToCluster)
  {
    uint32_t*& sortedIndexToCluster = *d_sortedIndexToCluster;
    uint32_t & numClusters = *_numClusters;

    const uint64_t* codesSorted = d_sortedCodes;

    if (maxNumClusters == 0 && maxElementsPerCluster == 0)
      throw std::runtime_error("One of 'maxNumClusters' and 'maxElementsPerCluster' must be non-zero.");
    if (maxNumClusters != 0 && maxElementsPerCluster != 0)
      throw std::runtime_error("Only one of 'maxNumClusters' and 'maxElementsPerCluster' can be non-zero.");

    // Mark cluster starts
    uint32_t* flags = nullptr;
    cudaMalloc((void**)&flags, numElements * sizeof(uint32_t));
    parallel_for_gpu(numElements,  [numbers=flags] __device__ (size_t index) {
      numbers[index] = 1;
    });
    cudaMalloc((void**)&sortedIndexToCluster, numElements * sizeof(uint32_t));

    uint32_t *uniqueSortedIndexToCluster = nullptr;
    uint32_t *uniqueSortedIndexToClusterCount = nullptr;
    uint32_t *numRuns = nullptr;
    cudaMalloc((void**)&uniqueSortedIndexToCluster, numElements * sizeof(uint32_t));
    cudaMalloc((void**)&uniqueSortedIndexToClusterCount, numElements * sizeof(uint32_t));
    cudaMalloc((void**)&numRuns, sizeof(uint32_t));

    // going to try to merge clusters that are too small
    int prevNumClusters = 0;
    uint32_t numElementsPerCluster = 1;
    while (true)
    {
      // Postfix sum the flag and subtract one to compute cluster addresses
      {
        // Declare, allocate, and initialize device-accessible pointers for input and output
        int  num_items = (int)numElements;       // e.g., 7
        uint32_t  *d_in = flags;                 // e.g., [8, 6, 7, 5, 3, 0, 9]
        uint32_t  *d_out = sortedIndexToCluster; // e.g., [ ,  ,  ,  ,  ,  ,  ]
        // Determine temporary device storage requirements
        void  *d_temp_storage = NULL;
        size_t temp_storage_bytes = 0;
        cub::DeviceScan::InclusiveSum(d_temp_storage, temp_storage_bytes, d_in, d_out, num_items);
        // Allocate temporary storage
        cudaMalloc(&d_temp_storage, temp_storage_bytes);
        // Run exclusive prefix sum
        cub::DeviceScan::InclusiveSum(d_temp_storage, temp_storage_bytes, d_in, d_out, num_items);
        // subtract one to get addresses
        parallel_for_gpu(num_items, [numbers=d_out] __device__ (size_t index) {
          numbers[index] = numbers[index] - 1;
        });
        cudaDeviceSynchronize();
        cudaFree(d_temp_storage);
      }

      // Compute run length encoding to determine next cluster from current
      {
        // Declare, allocate, and initialize device-accessible pointers for input and output
        int  num_items = (int)numElements;                         // e.g., 8
        uint32_t  *d_in = sortedIndexToCluster;                    // e.g., [0, 2, 2, 9, 5, 5, 5, 8]
        uint32_t  *d_unique_out = uniqueSortedIndexToCluster;      // e.g., [ ,  ,  ,  ,  ,  ,  ,  ]
        uint32_t  *d_counts_out = uniqueSortedIndexToClusterCount; // e.g., [ ,  ,  ,  ,  ,  ,  ,  ]
        uint32_t  *d_num_runs_out = numRuns;                       // e.g., [ ]
        // Determine temporary device storage requirements
        void     *d_temp_storage = NULL;
        size_t   temp_storage_bytes = 0;
        cub::DeviceRunLengthEncode::Encode(d_temp_storage, temp_storage_bytes, d_in, d_unique_out, d_counts_out, d_num_runs_out, num_items);
        // Allocate temporary storage
        cudaMalloc(&d_temp_storage, temp_storage_bytes);
        // Run encoding
        cub::DeviceRunLengthEncode::Encode(d_temp_storage, temp_storage_bytes, d_in, d_unique_out, d_counts_out, d_num_runs_out, num_items);
        // d_unique_out      <-- [0, 2, 9, 5, 8]
        // d_counts_out      <-- [1, 2, 1, 3, 1]
        // d_num_runs_out    <-- [5]
        cudaFree(d_temp_storage);
      } 

      uint32_t hNumRuns;
      cudaMemcpy(&hNumRuns, numRuns, sizeof(uint32_t), cudaMemcpyDeviceToHost);

      // alternatively...
      numClusters = hNumRuns;
      if (prevNumClusters == numClusters) break; // can't merge any more
      prevNumClusters = numClusters;
      std::cout << "generated " << numClusters << " clusters... merging..." << std::endl;

      // unmark cluster flags where too many elements exist in a cluster
      linear_kernel(mergeClusters, 
        numElements, hNumRuns,
        flags, codesSorted,
        sortedIndexToCluster,
        uniqueSortedIndexToClusterCount,
        maxElementsPerCluster > 0 ? maxElementsPerCluster : (uint32_t)numElements
      );
      OWL_CUDA_SYNC_CHECK();
      numElementsPerCluster *= 2;

      // cudaMemcpy(&numClusters,(sortedIndexToCluster + (numElements - 1)),sizeof(uint32_t),cudaMemcpyDeviceToHost);
      // numClusters += 1; // account for subtract by one.

      if (maxNumClusters > 0 && numClusters < maxNumClusters) break;
    }

    std::cout << "done... numElementsPerCluster = " << numElementsPerCluster << std::endl;

    cudaFree(uniqueSortedIndexToCluster);
    cudaFree(uniqueSortedIndexToClusterCount);

    cudaFree(numRuns);
    cudaFree(flags);
  }

  __global__ void makeClusterBBox( const uint64_t numElements,
                                   const uint32_t* __restrict__ elementIDs,
                                   const vec4f   * __restrict__ vertexBuffer,
                                   const int32_t * __restrict__ indexBuffer,
                                   const uint32_t numClusters,
                                   const uint32_t* __restrict__ clusterIDs,
                                   box4f* clusters)
  {
    const size_t index = size_t(blockIdx.x) * blockDim.x + threadIdx.x;
    if (index >= numElements) return;

    const uint32_t clusterID = clusterIDs[index];
    const uint32_t primID = elementIDs[index];

    box4f primBounds4 = box4f();

    #pragma unroll
    for (int i=0; i<8; ++i) {
      const int idx = indexBuffer[primID*8+i];
      if (idx >= 0) {
        const vec4f v = vertexBuffer[idx];
        primBounds4 = primBounds4.extend(v);
      }
    }

    atomicMin(&clusters[clusterID].lower.x,primBounds4.lower.x);
    atomicMax(&clusters[clusterID].upper.x,primBounds4.upper.x);
    atomicMin(&clusters[clusterID].lower.y,primBounds4.lower.y);
    atomicMax(&clusters[clusterID].upper.y,primBounds4.upper.y);
    atomicMin(&clusters[clusterID].lower.z,primBounds4.lower.z);
    atomicMax(&clusters[clusterID].upper.z,primBounds4.upper.z);
    atomicMin(&clusters[clusterID].lower.w,primBounds4.lower.w);
    atomicMax(&clusters[clusterID].upper.w,primBounds4.upper.w);
  }

  void fillClusterBBoxBuffer(const size_t    numElements,
                             const uint32_t *d_sortedElementIDs,
                             const vec4f    *d_vertices, 
                             const int      *d_indices, 
                             const uint32_t  numClusters,
                             const uint32_t *d_sortedClusterIDs, 
                             box4f*          d_clusters)
  {  
    cudaMemset(d_clusters, sizeof(box4f)*numClusters, 0);
    linear_kernel(makeClusterBBox, 
      numElements, d_sortedElementIDs, d_vertices, d_indices, 
      numClusters, d_sortedClusterIDs, d_clusters
    );
    OWL_CUDA_SYNC_CHECK();
  }
  
  struct VertexValue {
    uint32_t address;
    float3 position;  
    float scalar;  
  };

  void fillLeafClusterBuffer(const size_t    numElements, const uint32_t * d_sortedElementIDs,
                             const size_t    numVertices, const vec4f    * d_vertices, 
                             const size_t    numIndices,  const int      * d_indices, 
                             const uint32_t  numMeshlets, const uint32_t * d_sortednumMeshletIDs)
  {
    // figure out how many elements of each type are in each cluster
    uint32_t* numPrimitivesInCluster;
    cudaMalloc((void**)&numPrimitivesInCluster, numMeshlets * sizeof(uint32_t));
    cudaMemset(numPrimitivesInCluster, 0, numMeshlets * sizeof(uint32_t));

    parallel_for_gpu(numElements, 
    [ 
      sortedIndexToCluster=d_sortednumMeshletIDs, 
      numClusters=numMeshlets, 
      elementIds=d_sortedElementIDs, 
      numPrimitivesInCluster=numPrimitivesInCluster
    ] 
    __device__ (size_t index) {
      const uint32_t clusterIndex = sortedIndexToCluster[index];
      atomicAdd(&numPrimitivesInCluster[clusterIndex], 1);
    });
    OWL_CUDA_SYNC_CHECK();

    // Copy to host ...
    std::vector<uint32_t> hNumPrimitivesInCluster(numMeshlets);
    cudaMemcpy(hNumPrimitivesInCluster.data(), numPrimitivesInCluster, numMeshlets * sizeof(uint32_t), cudaMemcpyDeviceToHost);
    // for (auto& n : hNumPrimitivesInCluster) {
    //   std::cout << n << std::endl;
    // }

    // // Fill vertex values structures
    // VertexValue* vertexValues;
    // cudaMalloc((void**)&vertexValues, numVertices * sizeof(VertexValue));
    // parallel_for_gpu(numVertices, 
    // [
    //   out=vertexValues, 
    //   in=d_vertices
    // ] 
    // __device__ (size_t index) {
    //   out[index] = {
    //     (uint32_t)index,
    //     vec3f(in[index]),
    //     in[index].w
    //   };
    // });
    // OWL_CUDA_SYNC_CHECK();

    // VertexValue *verticesCopy;
    // cudaMalloc((void**)&verticesCopy, numVertices * sizeof(VertexValue)); // freed after sort

    // uint8_t *isUsed;
    // cudaMalloc((void**)&isUsed,  numVertices * sizeof(uint8_t)); // free'd after replace unused step
    
    // uint32_t *newIdx;
    // cudaMalloc((void**)&newIdx, numVertices * sizeof(uint32_t)); // free'd at end

    // uint32_t* perm;
    // cudaMalloc((void**) &perm, numVertices * sizeof(uint32_t)); // free'd at end

    // uint32_t* clusterElementIds;
    // cudaMalloc((void**)&clusterElementIds, totalElements * sizeof(uint32_t));  

    // uint8_t* clusterElementTypes;
    // cudaMalloc((void**)&clusterElementTypes, totalElements * sizeof(uint8_t));  // freed after sort

  }

}
