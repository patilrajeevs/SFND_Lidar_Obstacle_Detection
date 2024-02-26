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

# set_source_files_properties(ransac2d.cpp PROPERTIES LANGUAGE CUDA)
add_executable (quizRansac ransac2d.cpp ../../render/render.cpp ../../geometry/line.cpp ransac2d.cu)
target_link_libraries (quizRansac ${PCL_LIBRARIES} ${Boost_LIBRARIES})

