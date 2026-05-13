cmake_minimum_required(VERSION 3.18)

if(NOT DEFINED CMAKE_CUDA_COMPILER AND EXISTS "/usr/local/cuda/bin/nvcc")
  set(CMAKE_CUDA_COMPILER "/usr/local/cuda/bin/nvcc" CACHE FILEPATH "CUDA compiler")
endif()

project(data_collect_cloud_renderer LANGUAGES CXX CUDA)

if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
  set(CMAKE_CUDA_ARCHITECTURES 75 86 89)
endif()

find_package(ament_cmake REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)

add_library(data_collect_cloud_renderer SHARED
  src/cloud_renderer.cu
)

target_include_directories(data_collect_cloud_renderer
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(data_collect_cloud_renderer PUBLIC cxx_std_17)
target_compile_options(data_collect_cloud_renderer
  PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-Wall -Wextra -Wpedantic>
)
set_target_properties(data_collect_cloud_renderer PROPERTIES
  CUDA_STANDARD 17
  CUDA_STANDARD_REQUIRED ON
  POSITION_INDEPENDENT_CODE ON
)

target_link_libraries(data_collect_cloud_renderer
  PRIVATE
    CUDA::cudart
    OpenGL::GL
    GLEW::GLEW
)

install(
  TARGETS data_collect_cloud_renderer
  EXPORT export_data_collect_cloud_renderer
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_data_collect_cloud_renderer HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_package()
