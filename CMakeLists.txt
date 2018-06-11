cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(pcl_visualizer_viewports)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable (generate_pc src/generate_pc.cpp)
target_link_libraries (generate_pc ${PCL_LIBRARIES})

add_executable (crop src/crop.cpp)
target_link_libraries (crop ${PCL_LIBRARIES})

add_executable (resampling src/resampling.cpp)
target_link_libraries (resampling ${PCL_LIBRARIES})



