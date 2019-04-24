cmake_minimum_required(VERSION 2.8.3)
project(cloudsegment)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

find_package(PCL 1.7 REQUIRED)
catkin_package()

SET(INCLUDE_DIRS
    ${PCL_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

include_directories(
    include/cloudsegment
    ${INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

SET(SOURCES
    src/main.cpp
    src/cloud_segmenter.cpp
    src/descriptors.cpp
    )
# libraries
SET(LIBRARIES
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
)

add_executable(cloudsegment_node ${SOURCES} ${INCLUDE_DIRECTORIES})
target_link_libraries(cloudsegment_node ${LIBRARIES})
