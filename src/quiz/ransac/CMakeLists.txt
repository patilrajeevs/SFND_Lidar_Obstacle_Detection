set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)
enable_language(CUDA)

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_definitions(-std=c++14)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")
set(CMAKE_BUILD_TYPE Debug)

project(playback)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

# add_executable (quizRansac2d ransac2d.cpp ../../render/render.cpp ../../geometry/line.cpp ransac2d.cu)
# target_link_libraries (quizRansac2d ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_executable (quizRansac3d ransac3d.cpp ../../render/render.cpp ../../geometry/plane.cpp ransac2d.cu)
target_link_libraries (quizRansac3d ${PCL_LIBRARIES} ${Boost_LIBRARIES})

