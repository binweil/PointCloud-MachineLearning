[TOC]

#### Features

- Real-time point cloud stream from Realsense D435
- 3D CNN model from Keras
- Segmentation node in ROS

##### Dependencies
-OpenCV 2.4
-PCL 1.8
-python-pcl
-Tensorflow
-Keras
-scipy
-vispy
-numpy

#### Installation
            cd catkin_ws/src
            catkin build
            cd build/OpenCV
			make install
            source Venv/bin/activate
            rosrun 3DCNN 3DCNN_node
