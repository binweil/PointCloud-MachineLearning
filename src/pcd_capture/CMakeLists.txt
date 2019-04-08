cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_BUILD_TYPE Release)

project(pcd_capture)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
FIND_PACKAGE(PCL 1.7 REQUIRED)
find_package(RTABMap 0.19.1 REQUIRED)
find_package(Qt5 COMPONENTS Widgets Core Gui Svg QUIET)
find_package(VTK REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(

)

# include directories
SET(INCLUDE_DIRS
    ${RTABMap_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

message("include='${RTABMap_INCLUDE_DIRS}'")

add_definitions(${PCL_DEFINITIONS})

include_directories(
    include/pcd_capture
    ${INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

SET(SOURCES
    src/main.cpp
    src/viewer.cpp
    src/viewer.ui
    src/cloudcapture.cpp

    src/mainwindow.ui
    src/mainwindow.cpp
    src/main.cpp
    src/svm_trainer.cpp
    src/classifier.cpp
    src/descriptors.cpp
    src/segmentation.cpp
    src/cloud_segmenter.cpp
    src/obj_detector.cpp
    src/robot_vision_common.cpp
    )
# libraries
SET(LIBRARIES
    ${RTABMap_LIBRARIES}
    ${QT_LIBRARIES}
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    Qt5::Widgets
)

add_executable(pcd_capture_node ${SOURCES} ${INCLUDE_DIRECTORIES})
target_link_libraries(pcd_capture_node ${LIBRARIES})
