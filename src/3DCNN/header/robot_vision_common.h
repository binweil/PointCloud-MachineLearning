#ifndef _ROBOT_VISION_COMMON_H_
#define _ROBOT_VISION_COMMON_H_


#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

//#ifndef _COMMON_CONSTANTS_H_
//#include <robot_vision/common_constants.h>
//#endif
//#include <robot_vision/visualizer.h>

#define RED 0xFF0000
#define GREEN 0x00FF00
#define BLUE 0x0000FF
#define YELLOW 0xFFFF00
#define CYAN 0x00FFFF
#define MAGENTA 0xFF00FF

const double bowlLength=0.1282528;
const double bowlWidth=0.121;
const double bowlHeight=0.1157708;

const double mugLength=0.0943486;
const double mugWidth=0.127;
const double mugHeight=0.1155481;

static const int color_array[] = {RED,GREEN,BLUE,YELLOW,CYAN,MAGENTA};

extern double voxelLeafSize;
extern int removePlaneIterations;
extern double distanceThreshold;
extern double PercentageCloud;
extern double tolerance;
extern int minClustersSize ;
extern int maxClustersSize ;

namespace robot_vision_common{

    template < typename T > std::string to_string( const T& n );

    void AssembleCloud(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &finalClusters,
                       sensor_msgs::PointCloud2 &output);

    bool inDimensions(pcl::PointXYZ minPT,pcl::PointXYZ maxPT,
                      double length,double width,double height,double scale);

    void detectObject(
                      pcl::PointXYZ &minPT,pcl::PointXYZ &maxPT,
                      std::vector<float> result,
                      double threshold, double scale,
                      pcl::visualization::PCLVisualizer::Ptr viewer,
                      int _id); // updates Marker vector
}

#endif
