
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

add_library(witcher
  Grid.cu
  ABRs.cpp
  AMRCellModel.cpp
  ExaBrickModel.cpp
  ExaBrickModel.cu
  ExaStitchModel.cpp
  KDTree.cpp
  LightInteractor.cpp
  KDTree.cpp
  OWLRenderer.cpp
  TriangleMesh.cpp
  )
if(EXA_STITCH_CPU)
  target_sources(witcher PRIVATE ${embedded_deviceCode})
endif()

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

