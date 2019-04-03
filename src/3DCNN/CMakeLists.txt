cmake_minimum_required(VERSION 2.8.11)
project(3DCNN)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

add_compile_options(-std=c++11)
find_package(VTK REQUIRED)
find_package(Qt5Widgets CONFIG REQUIRED)
#find_package(Qt4 REQUIRED )
find_package(PCL 1.8 REQUIRED)
find_package(OpenCV 2 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  actionlib
  roscpp
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES 3DCNN
#  CATKIN_DEPENDS actionlib libopencv-dev libpcl-dev roscpp rospy std_msgs
#  DEPENDS system_lib
)

INCLUDE(${VTK_USE_FILE})
include_directories(
  header
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_BINARY_DIR}
)
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(SOURCES
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

add_executable(3DCNN_node ${SOURCES} ${INCLUDE_DIRECTORIES})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${QT_LIBRARIES}
  Qt5::Widgets
 )

