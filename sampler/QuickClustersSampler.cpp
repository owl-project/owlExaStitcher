#include "QuickClustersSampler.h"

#include <owl/common/parallel/parallel_for.h>

#include <cuda_runtime.h>

extern "C" char embedded_ExaStitchSampler[];

namespace exa {

  void sortLeafPrimitives(uint64_t* &codesSorted, uint32_t* &elementIdsSorted, 
                          const vec4f *d_vertices, const int *d_indices, const size_t numElements);

  void buildClusters(const uint64_t* codesSorted, 
                     const size_t    numElements,
                     const uint32_t  maxNumClusters,
                     const uint32_t  maxElementsPerCluster,
                     uint32_t & numClusters, 
                     uint32_t*& sortedIndexToCluster);

  void fillClusterBBoxBuffer(const size_t    numElements,
                             const uint32_t *d_sortedElementIDs,
                             const vec4f    *d_vertices, 
                             const int      *d_indices, 
                             const uint32_t  numClusters,
                             const uint32_t *d_sortedClusterIDs, 
                             box4f*          d_clusters);
  
  void fillLeafClusterBuffer(const size_t    numElements, const uint32_t * d_sortedElementIDs,
                             const size_t    numVertices, const vec4f    * d_vertices, 
                             const size_t    numIndices,  const int      * d_indices, 
                             const uint32_t  numMeshlets, const uint32_t * d_sortednumMeshletIDs);

  bool QuickClustersSampler::build(OWLContext context, Model::SP mod)
  {
    // process host data
    if (!mod)
      return false;
    model = std::dynamic_pointer_cast<QuickClustersModel>(mod);
    if (!model)
      return false;
    const std::vector<int> &indices = model->indices;
    const std::vector<vec4f> &vertices = model->vertices;

    // ==================================================================
    // setup geometry data
    // ==================================================================

    OWLVarDecl leafGeomVars[]
    = {
       { "indexBuffer",  OWL_BUFPTR, OWL_OFFSETOF(QCLeafGeom,indexBuffer)},
       { "vertexBuffer",  OWL_BUFPTR, OWL_OFFSETOF(QCLeafGeom,vertexBuffer)},
       { "maxOpacities", OWL_BUFPTR, OWL_OFFSETOF(QCLeafGeom,maxOpacities)},
       { nullptr /* sentinel to mark end of list */ }
    };

    module = owlModuleCreate(context, embedded_ExaStitchSampler);

    if (!vertices.empty()) {
      umeshMaxOpacities = owlDeviceBufferCreate(context, OWL_FLOAT, indices.size()/8, nullptr);

      leafGeom.geomType = owlGeomTypeCreate(context, OWL_GEOM_USER, sizeof(QCLeafGeom), leafGeomVars, -1);
      owlGeomTypeSetBoundsProg   (leafGeom.geomType, module, "StitchGeomBounds");
      owlGeomTypeSetIntersectProg(leafGeom.geomType, SAMPLING_RAY_TYPE, module, "StitchGeomIsect");
      owlGeomTypeSetClosestHit   (leafGeom.geomType, SAMPLING_RAY_TYPE, module, "StitchGeomCH");

      OWLGeom geom = owlGeomCreate(context, leafGeom.geomType);
      owlGeomSetPrimCount(geom, indices.size()/8);

      vertexBuffer = owlDeviceBufferCreate(context, OWL_FLOAT4,
                                           vertices.size(),
                                           vertices.data());

      indexBuffer = owlDeviceBufferCreate(context, OWL_INT,
                                          indices.size(),
                                          indices.data());

      owlGeomSetBuffer(geom,"vertexBuffer",vertexBuffer);
      owlGeomSetBuffer(geom,"indexBuffer",indexBuffer);
      owlGeomSetBuffer(geom,"maxOpacities",umeshMaxOpacities);

      owlBuildPrograms(context);

      leafGeom.blas = owlUserGeomGroupCreate(context, 1, &geom);
      owlGroupBuildAccel(leafGeom.blas);
    }

    // ==================================================================
    // setup geometry data
    // ==================================================================
    const int   *d_indices  = (const int*  )owlBufferGetPointer(indexBuffer,0);
    const vec4f *d_vertices = (const vec4f*)owlBufferGetPointer(vertexBuffer,0);

    const size_t numIndices = indices.size();
    const size_t numVertices = vertices.size();
    const size_t numElements = numIndices/8;

    uint64_t* d_sortedCodes = nullptr;
    uint32_t* d_sortedElementIDs = nullptr;

    std::cout << "numElements " << numElements << std::endl;

    printGPUMemory("<<<< before clustering");
    sortLeafPrimitives(d_sortedCodes, d_sortedElementIDs, d_vertices, d_indices, numElements);

    // // building ... macrocells ?!
    // printGPUMemory("<<<< before create macrocells");
    // OWLBuffer clusterBBoxBuffer;
    // box4f* d_clusterBBoxes = nullptr;
    // uint32_t numClusterBBoxes;
    // {
    //   uint32_t* d_sortedClusterIDs = nullptr;
    //   buildClusters(d_sortedCodes, numElements, 
    //                 /*maxNumClusters=*/1000000,  // here, we don't care how many elements go 
    //                 /*maxElementsPerCluster=*/0, // in a cluster. just for adaptive sampling...
    //                 numClusterBBoxes, d_sortedClusterIDs);
    //   clusterBBoxBuffer = owlDeviceBufferCreate(context, OWL_USER_TYPE(box4f), numClusterBBoxes, nullptr);
    //   d_clusterBBoxes = (box4f*)owlBufferGetPointer(clusterBBoxBuffer, 0);
    //   fillClusterBBoxBuffer(numElements, d_sortedElementIDs, d_vertices, d_indices, numClusterBBoxes, d_sortedClusterIDs, d_clusterBBoxes);
    //   cudaFree(d_sortedClusterIDs);
    // }
    // owlBufferRelease(clusterBBoxBuffer);
    // printGPUMemory(">>>> after create macrocells");

    // // building ... leaf-clusters
    // printGPUMemory("<<<< before create leaf-clusters");
    // {
    //   uint32_t* d_sortedClusterIDs = nullptr;
    //   buildClusters(d_sortedCodes, numElements, /*maxNumClusters=*/0,
    //                 /*maxElementsPerCluster=*/
    //                 // 50000,  // works for impact
    //                 30000,     // works for chombo, earthquake
    //                 // 200000, // works for small lander...
    //                 // 10000000000000000, // one big meshlet
    //                 numClusterBBoxes, d_sortedClusterIDs);
    //
    //   fillLeafClusterBuffer(numElements, d_sortedElementIDs, 
    //                         numVertices, d_vertices, 
    //                         numIndices, d_indices, 
    //                         numClusterBBoxes, d_sortedClusterIDs);
    //
    //   cudaFree(d_sortedClusterIDs);
    // }
    // printGPUMemory(">>>> after create leaf-clusters");

    cudaFree(d_sortedElementIDs);
    cudaFree(d_sortedCodes);
    printGPUMemory(">>>> after clustering");

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

    owlGroupBuildAccel(tlas);

    // build the grid
    Grid::SP &grid = model->grid;
    if (!grid || grid->dims==vec3i(0))
      return false;

    box3f &cellBounds = model->cellBounds;
    grid->build(context,shared_from_this()->as<QuickClustersSampler>(),grid->dims,cellBounds);
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
    std::vector<OWLVarDecl> vars
      = {
         { "qcs.sampleBVH", OWL_GROUP, OWL_OFFSETOF(LP,sampleBVH) },
#ifdef EXA_STITCH_MIRROR_EXAJET
         { "qcs.mirrorInvTransform", OWL_USER_TYPE(affine3f), OWL_OFFSETOF(LP,mirrorInvTransform)}
#endif
    };
    return vars;
  }

  void QuickClustersSampler::setLPs(OWLParams lp)
  {
    owlParamsSetGroup(lp,"qcs.sampleBVH",tlas);
#ifdef EXA_STITCH_MIRROR_EXAJET
    affine3f mirrorInvTransform = rcp((const affine3f &)model->mirrorTransform);
    owlParamsSetRaw(lp,"qcs.mirrorInvTransform",&mirrorInvTransform);
#endif
  }
} // ::exa
