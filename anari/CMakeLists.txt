
find_package(anari REQUIRED)

project(anari_library_exastitch)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Code generation ##

anari_generate_queries(
  NAME exastitch
  PREFIX Device
  CPP_NAMESPACE exa
  JSON_DEFINITIONS_FILE ${CMAKE_CURRENT_SOURCE_DIR}/exastitch_device.json
)

anari_generate_debug_objects(
  NAME exastitch
  PREFIX Device
  CPP_NAMESPACE exa
  JSON_DEFINITIONS_FILE ${CMAKE_CURRENT_SOURCE_DIR}/exastitch_device.json
)

add_library(${PROJECT_NAME} SHARED)

target_sources(${PROJECT_NAME} PRIVATE
  array/Array.cpp
  array/Array1D.cpp
  array/Array2D.cpp
  array/Array3D.cpp
  array/ObjectArray.cpp
  Camera.cpp
  Device.cpp
  DeviceDebugFactory.cpp
  DeviceQueries.cpp
  Frame.cpp
  Light.cpp
  Object.cpp
  Renderer.cpp
  SpatialField.cpp
  Volume.cpp
)

target_include_directories(${PROJECT_NAME}
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
)

target_link_libraries(${PROJECT_NAME}
PUBLIC
  anari::anari
  anari::anari_library_debug
  anari::helium
  owl::owl
  witcher
)
