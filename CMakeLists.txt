cmake_minimum_required(VERSION 2.8)
project(point_cloud_plane_detection)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(PCL 1.8)
message(WARNING "PCL 1.8 found")

if (NOT ${PCL_FOUND})
  message(WARNING "PCL 1.8 not found, attempting 1.7...")
  find_package(PCL 1.7 REQUIRED)
  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/pcl_include)
  set(MAYBE_PCL_COMMON_COLORS "${CMAKE_CURRENT_SOURCE_DIR}/pcl_include/colors.cpp")
endif()

set(CMAKE_CXX_STANDARD 11) # C++11...
set(CMAKE_AUTOMOC ON)
add_definitions(-DQT_NO_KEYWORDS)

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

add_executable(main
  src/main.cpp
  )

target_link_libraries(
  main
  point_cloud_plane_detection
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  ${Eigen3_LIBRARIES}
  )


set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_library(point_cloud_plane_detection
  src/point_cloud_plane_curves_extract.cpp
  src/point_cloud_plane_segment.cpp
  src/point_cloud_plane_params.cpp
)

target_link_libraries(point_cloud_plane_detection
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  ${Eigen3_LIBRARIES}
)
