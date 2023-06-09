cmake_minimum_required(VERSION 3.16)

if (POLICY CMP0048)
  cmake_policy(SET CMP0048 NEW)
endif()

project(make-dual-and-bricks VERSION 0.0.1 LANGUAGES C CXX)

# ==================================================================
add_executable(amrMakeDualMesh
  makeDual.cpp
  )

target_link_libraries(amrMakeDualMesh
  PUBLIC
  umesh
  )

# ==================================================================
add_executable(amrMakeGrids
  makeGrids.cpp
  )

target_link_libraries(amrMakeGrids
  PUBLIC
  umesh
  )



