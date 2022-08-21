
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
      deviceCode.cu
  )
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
# exastitch
# ------------------------------------------------------------------

add_executable(exaStitchViewer
  Grid.cu
  ABRs.cpp
  AMRCellModel.cpp
  ExaBrickModel.cpp
  ExaBrickModel.cu
  ExaStitchModel.cpp
  OWLRenderer.cpp
  TriangleMesh.cpp
  viewer.cpp
  )
if(EXA_STITCH_CPU)
  target_sources(exaStitchViewer PRIVATE ${embedded_deviceCode})
endif()

target_link_directories(exaStitchViewer PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/submodules/fakeOwl/submodules/cuteeOwl/qtOWL)
if(NOT EXA_STITCH_CPU)
  target_link_libraries(exaStitchViewer embedded_deviceCode)
endif()
target_link_libraries(exaStitchViewer
      owl::owl
      qtOWL
      umesh
      ${OPENGL_LIBRARIES})
qt5_use_modules(exaStitchViewer Widgets)
set_target_properties(exaStitchViewer PROPERTIES MACOSX_BUNDLE YES)
if(EXA_STITCH_CPU)
  target_compile_options(exaStitchViewer PUBLIC -DEXA_STITCH_CPU=1)
endif()
# QT_OWL_LINK(exaStitchViewer)



