
project(exa-stitch)

cmake_minimum_required(VERSION 3.1.3)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

option(BUILD_SHARED_LIBS "Build all libraries as shared libraries instead of static" ON)

option(EXA_STITCH_CPU "Build the project to run on the CPU" OFF)
if(EXA_STITCH_CPU)
  set(owl_dir ${PROJECT_SOURCE_DIR}/submodules/fakeOwl)
  add_subdirectory(${owl_dir} EXCLUDE_FROM_ALL)
  include_directories(${owl_dir}/include)
  include(${owl_dir}/cmake/configure_fake_owl.cmake)
  fake_owl_compile_and_embed(
    embedded_deviceCode deviceCode.cpp
    )
set(CUTEEOWL_USE_CUDA OFF CACHE BOOL "Build with CUDA frame buffer implementation" FORCE)
else()
  enable_language(CUDA)
  set(owl_dir ${CMAKE_CURRENT_SOURCE_DIR}/submodules/owl)
  add_subdirectory(${owl_dir} EXCLUDE_FROM_ALL)
  include_directories(${OWL_INCLUDES})
  
  set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${owl_dir}/owl/cmake/")
  include(embed_ptx)
  embed_ptx(
    OUTPUT_TARGET
      embedded_deviceCode
    EMBEDDED_SYMBOL_NAMES
      embedded_deviceCode
    PTX_LINK_LIBRARIES
      owl::owl
    SOURCES
      deviceCode.cpp
  )
  set_source_files_properties(deviceCode.cpp PROPERTIES LANGUAGE CUDA)
  set(CUTEEOWL_USE_CUDA ON CACHE BOOL "Build with CUDA frame buffer implementation" FORCE)
endif()
add_subdirectory(${PROJECT_SOURCE_DIR}/submodules/fakeOwl/submodules/cuteeOwl)
include_directories(${PROJECT_SOURCE_DIR}/submodules/fakeOwl/submodules/cuteeOwl)
include_directories(${QT_OWL_INCLUDES})
find_package(Qt5Widgets REQUIRED)
find_package(OpenGL REQUIRED)
set(CMAKE_AUTOMOC ON)

# ------------------------------------------------------------------
# umesh
# ------------------------------------------------------------------

add_subdirectory(submodules/umesh EXCLUDE_FROM_ALL)


# ------------------------------------------------------------------
# witcher lib
# ------------------------------------------------------------------

add_library(witcher STATIC
  Grid.cu
  ABRs.cpp
  AMRCellModel.cpp
  ExaBrickModel.cpp
  ExaBrickModel.cu
  ExaStitchModel.cpp
  KDTree.cpp
  LightInteractor.cpp
  Model.cpp
  OWLRenderer.cpp
  TriangleMesh.cpp
  Witcher.cpp
  )
if(EXA_STITCH_CPU)
  target_sources(witcher PRIVATE ${embedded_deviceCode})
endif()

#--- options ----------------------------------------------
option(EXA_STITCH_WITH_EXA_STITCH_SAMPLER "Build includes exa stitch sampler" ON)
if(EXA_STITCH_WITH_EXA_STITCH_SAMPLER)
  target_compile_definitions(embedded_deviceCode_ptx PUBLIC -DEXA_STITCH_WITH_EXA_STITCH_SAMPLER=1)
  target_compile_definitions(witcher PUBLIC -DEXA_STITCH_WITH_EXA_STITCH_SAMPLER=1)
endif()

option(EXA_STITCH_WITH_EXA_BRICK_SAMPLER "Build includes exa brick sampler" ON)
if(EXA_STITCH_WITH_EXA_BRICK_SAMPLER)
  target_compile_definitions(embedded_deviceCode_ptx PUBLIC -DEXA_STITCH_WITH_EXA_BRICK_SAMPLER=1)
  target_compile_definitions(witcher PUBLIC -DEXA_STITCH_WITH_EXA_BRICK_SAMPLER=1)
endif()

option(EXA_STITCH_WITH_AMR_CELL_SAMPLER "Build includes AMR cell sampler" ON)
if(EXA_STITCH_WITH_AMR_CELL_SAMPLER)
  target_compile_definitions(embedded_deviceCode_ptx PUBLIC -DEXA_STITCH_WITH_AMR_CELL_SAMPLER=1)
  target_compile_definitions(witcher PUBLIC -DEXA_STITCH_WITH_AMR_CELL_SAMPLER=1)
endif()

option(EXA_STITCH_MIRROR_EXAJET "Activate mirroring for exajet in kernels" OFF)
if(EXA_STITCH_MIRROR_EXAJET)
  target_compile_definitions(embedded_deviceCode_ptx PUBLIC -DEXA_STITCH_MIRROR_EXAJET=1)
  target_compile_definitions(witcher PUBLIC -DEXA_STITCH_MIRROR_EXAJET=1)
endif()

set(EXA_STITCH_EXA_BRICK_SAMPLER_MODE 0 CACHE STRING "ABR-BVH: 0, ext.brick-BVH: 1")
target_compile_definitions(embedded_deviceCode_ptx PUBLIC -DEXA_STITCH_EXA_BRICK_SAMPLER_MODE=${EXA_STITCH_EXA_BRICK_SAMPLER_MODE})
target_compile_definitions(witcher PUBLIC -DEXA_STITCH_EXA_BRICK_SAMPLER_MODE=${EXA_STITCH_EXA_BRICK_SAMPLER_MODE})

set(EXA_STITCH_EXA_BRICK_TRAVERSAL_MODE 0 CACHE STRING "ABR+optix: 0, grid+DDA: 1, grid+optix: 2, bricks+kdtree: 3, bricks+optix: 4")
target_compile_definitions(embedded_deviceCode_ptx PUBLIC -DEXA_STITCH_EXA_BRICK_TRAVERSAL_MODE=${EXA_STITCH_EXA_BRICK_TRAVERSAL_MODE})
target_compile_definitions(witcher PUBLIC -DEXA_STITCH_EXA_BRICK_TRAVERSAL_MODE=${EXA_STITCH_EXA_BRICK_TRAVERSAL_MODE})
#--- options ----------------------------------------------

target_link_directories(witcher PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/submodules/fakeOwl/submodules/cuteeOwl/qtOWL)
if(NOT EXA_STITCH_CPU)
  target_link_libraries(witcher embedded_deviceCode)
endif()
target_link_libraries(witcher
      owl::owl
      qtOWL
      umesh
      ${OPENGL_LIBRARIES})

# ------------------------------------------------------------------
# viewer
# ------------------------------------------------------------------

add_executable(exaStitchViewer viewer.cpp)
qt5_use_modules(exaStitchViewer Widgets)
target_link_libraries(exaStitchViewer witcher)
set_target_properties(exaStitchViewer PROPERTIES MACOSX_BUNDLE YES)
if(EXA_STITCH_CPU)
  target_compile_options(exaStitchViewer PUBLIC -DEXA_STITCH_CPU=1)
endif()
# QT_OWL_LINK(exaStitchViewer)



# ------------------------------------------------------------------
# headless viewer
# ------------------------------------------------------------------

add_executable(exaStitchHeadlessViewer viewer.cpp headless.cpp)
#link with QT5 only because that's more convenient than ifdef'ing
#all the relevant parts in the cpp
qt5_use_modules(exaStitchHeadlessViewer Core)
target_link_libraries(exaStitchHeadlessViewer witcher)
target_compile_definitions(exaStitchHeadlessViewer PUBLIC HEADLESS)
if(EXA_STITCH_CPU)
  target_compile_options(exaStitchHeadlessViewer PUBLIC -DEXA_STITCH_CPU=1)
endif()
# QT_OWL_LINK(exaStitchViewer)

