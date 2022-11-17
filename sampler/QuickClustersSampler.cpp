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
                     uint32_t & numClusters, 
                     uint32_t*& sortedIndexToCluster);

  void fillClusters(const size_t    numElements,
                    const uint32_t *d_sortedElementIDs,
                    const vec4f    *d_vertices, 
                    const int      *d_indices, 
                    const uint32_t  numClusters,
                    const uint32_t *d_sortedClusterIDs, 
                    box4f*          d_clusters);
  
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
    const size_t numElements = indices.size()/8;
    uint32_t     numClusters;

    uint32_t* d_sortedElementIDs = nullptr;
    uint32_t* d_sortedClusterIDs = nullptr;

    printGPUMemory("<<<< before clustering");

    uint64_t* d_sortedCodes = nullptr;
    sortLeafPrimitives(d_sortedCodes, d_sortedElementIDs, d_vertices, d_indices, numElements);
    buildClusters(d_sortedCodes, numElements, 1000000, numClusters, d_sortedClusterIDs);
    cudaFree(d_sortedCodes);

    printGPUMemory("<<<< before fillClusters");
    clusterBuffer = owlDeviceBufferCreate(context, OWL_USER_TYPE(box4f), numClusters, nullptr);
    box4f* d_clusters = (box4f*)owlBufferGetPointer(clusterBuffer, 0);
    fillClusters(numElements, d_sortedElementIDs, d_vertices, d_indices, numClusters, d_sortedClusterIDs, d_clusters);
    owlBufferRelease(clusterBuffer);
    printGPUMemory(">>>> after fillClusters");

    cudaFree(d_sortedElementIDs);
    cudaFree(d_sortedClusterIDs);
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
