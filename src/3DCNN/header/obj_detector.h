#ifndef OBJ_DETECTOR_H
#define OBJ_DETECTOR_H

#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/filters/passthrough.h>

//#include <robot_vision/visualizer.h>
#include <cloud_segmenter.h>
#include <descriptors.h>
#include <classifier.h>
//#include <robot_vision/SVMclassifierAction.h>
#include <robot_vision_common.h>


void cloud_cb (const pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud,
               pcl::visualization::PCLVisualizer::Ptr viewer);


#endif // OBJ_DETECTOR_H
