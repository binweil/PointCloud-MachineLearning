#ifndef SEGMENTATION_H
#define SEGMENTATION_H

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/filters/passthrough.h>

//#include <robot_vision/visualizer.h>
#include <cloud_segmenter.h>
#include <descriptors.h>
//#include <classifier.h>
//#include <robot_vision/SVMclassifierAction.h>
//#include <robot_vision/robot_vision_common.h>
//#include <robot_vision/robot_vision_paramsConfig.h>
//#include <robot_vision/common_constants.h>

//#include <robot_vision/pointcloudVector.h>

void cloud_segment (const pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud);

#endif // SEGMENTATION_H
