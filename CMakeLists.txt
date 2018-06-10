cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(pcl_visualizer_viewports)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable (pole_detection src/pole_detection.cpp)
target_link_libraries (pole_detection ${PCL_LIBRARIES})

add_executable (crop src/crop.cpp)
target_link_libraries (crop ${PCL_LIBRARIES})

add_executable (cy_seg src/cy_seg.cpp)
target_link_libraries (cy_seg ${PCL_LIBRARIES})


