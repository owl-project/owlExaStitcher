#include "QuickClustersSampler.h"

#include <owl/common/parallel/parallel_for.h>
#include <owl/helper/cuda.h>

#include <cuda_runtime.h>

#include <set>

extern "C" char embedded_QuickClustersSampler[];

namespace exa
{
  template<typename T>
  T div_round_up(T a, T b)
  {
    return (a + b - 1) / b;
  }

  void sortElements(const size_t numElements,
                    const vec4f *d_vertices,
                    const int *d_indices,
                    uint64_t **d_sortedElementCodes,
                    uint32_t **d_sortedElementIDs);

  void reorderElements(const size_t numElements,
                       const uint32_t *d_sortedElementIDs,
                       const vec4f *d_vertices,
                       const int *d_indices,
                       int **d_sortedIndices);

  void buildClusters(const size_t numElements,
                     const uint64_t *d_sortedCodes,
                     const uint32_t maxNumClusters,
                     const uint32_t maxElementsPerCluster,
                     uint32_t *numClusters,
                     uint32_t **d_sortedIndexToCluster);

  void fillClusterBBoxBuffer(const size_t numElements,
                             const uint32_t *d_sortedElementIDs,
                             const vec4f *d_vertices,
                             const int *d_indices,
                             const uint32_t numClusters,
                             const uint32_t *d_sortedClusterIDs,
                             box4f *d_clusters);

  void fillLeafClusterBuffer(const size_t numElements, const uint32_t *d_sortedElementIDs,
                             const size_t numVertices, const vec4f *d_vertices,
                             const size_t numIndices, const int *d_indices,
                             const uint32_t numMeshlets, const uint32_t *d_sortednumMeshletIDs);

  bool QuickClustersSampler::build(OWLContext context, Model::SP mod)
  {
    // process host data
    if (!mod) return false;
    model = std::dynamic_pointer_cast<QuickClustersModel>(mod);
    if (!model) return false;
    std::vector<int> &indices = model->indices;
    std::vector<vec4f> &vertices = model->vertices;

    // ==================================================================
    // setup geometry data
    // ==================================================================
    if (vertices.empty()) return false;

    module = owlModuleCreate(context, embedded_QuickClustersSampler);

    // ==================================================================
    // setup geometry data
    // ==================================================================

    vertexBuffer = owlDeviceBufferCreate(context, OWL_FLOAT4,
                                         vertices.size(),
                                         vertices.data());

    indexBuffer = owlDeviceBufferCreate(context, OWL_INT,
                                        indices.size(),
                                        indices.data());

    umeshMaxOpacities = owlDeviceBufferCreate(context, OWL_FLOAT, indices.size() / 8, nullptr);

    // ==================================================================
    // setup geometry data
    // ==================================================================
    const vec4f *d_vertices = (const vec4f *)owlBufferGetPointer(vertexBuffer, 0);
    int *d_indices          = (int *)owlBufferGetPointer(indexBuffer, 0);

    const size_t numVertices = vertices.size();
    const size_t numIndices = indices.size();
    const size_t numElements = numIndices / 8;
    std::cout << "numElements " << numElements << std::endl;

    printGPUMemory("<<<< before clustering");

    uint64_t *d_sortedCodes = nullptr;
    uint32_t *d_sortedElementIDs = nullptr;
    sortElements(numElements, d_vertices, d_indices, &d_sortedCodes, &d_sortedElementIDs);

    // std::vector<uint64_t> sortedElementCodes(numElements);
    // cudaMemcpy(sortedElementCodes.data(), d_sortedCodes, numElements * sizeof(uint64_t), cudaMemcpyDeviceToHost);
    // std::vector<uint32_t> sortedElementIDs(numElements);
    // cudaMemcpy(sortedElementIDs.data(), d_sortedElementIDs, numElements * sizeof(uint32_t), cudaMemcpyDeviceToHost);

    OWL_CUDA_SYNC_CHECK();

    int *d_sortedIndices = nullptr;
    reorderElements(numElements, d_sortedElementIDs, d_vertices, d_indices, &d_sortedIndices);
    OWL_CUDA_SYNC_CHECK();
    cudaMemcpy(d_indices, d_sortedIndices, numIndices * sizeof(int), cudaMemcpyDeviceToDevice);
    cudaMemcpy(indices.data(), d_sortedIndices, numIndices * sizeof(int), cudaMemcpyDeviceToHost); // just keep host/device consistent
    cudaFree(d_sortedIndices);

    OWL_CUDA_SYNC_CHECK();

#if 0
    // building ... macrocells
    printGPUMemory("<<<< before create macrocells");
    OWLBuffer clusterBBoxBuffer;
    box4f* d_clusterBBoxes = nullptr;
    uint32_t numClusterBBoxes;
    {
      uint32_t* d_sortedClusterIDs = nullptr;
      buildClusters(numElements, d_sortedCodes,  
                    /*maxNumClusters=*/1000000,  // here, we don't care how many elements go 
                    /*maxElementsPerCluster=*/0, // in a cluster. just for adaptive sampling...
                    &numClusterBBoxes, &d_sortedClusterIDs);
      clusterBBoxBuffer = owlDeviceBufferCreate(context, OWL_USER_TYPE(box4f), numClusterBBoxes, nullptr);
      d_clusterBBoxes = (box4f*)owlBufferGetPointer(clusterBBoxBuffer, 0);
      fillClusterBBoxBuffer(numElements, d_sortedElementIDs, d_vertices, d_indices, numClusterBBoxes, d_sortedClusterIDs, d_clusterBBoxes);
      cudaFree(d_sortedClusterIDs);
    }
    owlBufferRelease(clusterBBoxBuffer);
    printGPUMemory(">>>> after create macrocells");
#endif

#if 0
    // building ... leaf-clusters
    printGPUMemory("<<<< before create leaf-clusters");
    {
      uint32_t* d_sortedClusterIDs = nullptr;
      buildClusters(numElements, d_sortedCodes,
                    /*maxNumClusters=*/0,
                    /*maxElementsPerCluster=*/
                    // 50000,  // works for impact
                    // 30000,  // works for chombo, earthquake
                    // 200000, // works for small lander...
                    10000000000000000, // one big meshlet
                    &numClusterBBoxes, &d_sortedClusterIDs);

      fillLeafClusterBuffer(numElements, d_sortedElementIDs, 
                            numVertices, d_vertices, 
                            numIndices, d_indices, 
                            numClusterBBoxes, d_sortedClusterIDs);

      cudaFree(d_sortedClusterIDs);
    }
    printGPUMemory(">>>> after create leaf-clusters");
#endif

    cudaFree(d_sortedElementIDs);
    cudaFree(d_sortedCodes);
    printGPUMemory(">>>> after clustering");

    // ==================================================================
    // setup geometry data
    // ==================================================================

    OWLVarDecl leafGeomVars[] = {
        {"indexBuffer", OWL_BUFPTR, OWL_OFFSETOF(QCLeafGeom, indexBuffer)},
        {"vertexBuffer", OWL_BUFPTR, OWL_OFFSETOF(QCLeafGeom, vertexBuffer)},
        {"maxOpacities", OWL_BUFPTR, OWL_OFFSETOF(QCLeafGeom, maxOpacities)},
        {"numElements", OWL_UINT, OWL_OFFSETOF(QCLeafGeom, numElements)},
        {nullptr /* sentinel to mark end of list */}};

    leafGeom.geomType = owlGeomTypeCreate(context, OWL_GEOM_USER, sizeof(QCLeafGeom), leafGeomVars, -1);
    owlGeomTypeSetBoundsProg(leafGeom.geomType, module, "QCLeafGeomBounds");
    owlGeomTypeSetIntersectProg(leafGeom.geomType, SAMPLING_RAY_TYPE, module, "QCLeafGeomIsect");

    OWLGeom geom = owlGeomCreate(context, leafGeom.geomType);
    owlGeomSetPrimCount(geom, div_round_up(indices.size() / 8, (size_t)QCLeafGeom::ELEMENTS_PER_BOX));

    owlGeomSetBuffer(geom, "vertexBuffer", vertexBuffer);
    owlGeomSetBuffer(geom, "indexBuffer", indexBuffer);
    owlGeomSetBuffer(geom, "maxOpacities", umeshMaxOpacities);
    owlGeomSet1ui(geom, "numElements", indices.size() / 8);

    owlBuildPrograms(context);

    leafGeom.blas = owlUserGeomGroupCreate(context, 1, &geom);
    owlGroupBuildAccel(leafGeom.blas);

    // ==================================================================
    // build accel struct
    // ==================================================================
    if (leafGeom.blas) {
#ifdef EXA_STITCH_MIRROR_EXAJET
      tlas = owlInstanceGroupCreate(context, 2);
#else
      tlas = owlInstanceGroupCreate(context, 1);
#endif
      owlInstanceGroupSetChild(tlas, 0, leafGeom.blas);
#ifdef EXA_STITCH_MIRROR_EXAJET
      owlInstanceGroupSetChild(tlas, 1, leafGeom.blas);
      owlInstanceGroupSetTransform(tlas, 1, &mirrorTransform);
#endif
    } else {
      return false;
    }

    size_t peak = 0;
    size_t final = 0;
    owlGroupGetAccelSize(tlas, &final, &peak);
    std::cout << "Peak element BVH memory consumption: " << std::string(prettyBytes(peak)) << " final " << std::string(prettyBytes(final)) << std::endl;

    owlGroupBuildAccel(tlas);

    // build the grid
    Grid::SP &grid = model->grid;
    if (!grid || grid->dims == vec3i(0)) return false;

    box3f &cellBounds = model->cellBounds;
    grid->build(context, shared_from_this()->as<QuickClustersSampler>(), grid->dims, cellBounds);
#ifdef EXA_STITCH_MIRROR_EXAJET
    grid->deviceTraversable.mirrorInvTransform = rcp((const affine3f &)mirrorTransform);
    grid->deviceTraversable.mirrorPlane.axis = 1;
    grid->deviceTraversable.mirrorPlane.offset = cellBounds.upper.y;
#endif
    Sampler::majorantAccel.grid = model->grid;
    Sampler::maxOpacities = model->grid->maxOpacities;

    // All tests passed => success
    return true;
  }

  std::vector<OWLVarDecl> QuickClustersSampler::getLPVariables()
  {
    std::vector<OWLVarDecl> vars = {
        {"qcs.sampleBVH", OWL_GROUP, OWL_OFFSETOF(LP, sampleBVH)},
#ifdef EXA_STITCH_MIRROR_EXAJET
        {"qcs.mirrorInvTransform", OWL_USER_TYPE(affine3f), OWL_OFFSETOF(LP, mirrorInvTransform)}
#endif
    };
    return vars;
  }

  void QuickClustersSampler::setLPs(OWLParams lp)
  {
    owlParamsSetGroup(lp, "qcs.sampleBVH", tlas);
#ifdef EXA_STITCH_MIRROR_EXAJET
    affine3f mirrorInvTransform = rcp((const affine3f &)model->mirrorTransform);
    owlParamsSetRaw(lp, "qcs.mirrorInvTransform", &mirrorInvTransform);
#endif
  }
} // ::exa
