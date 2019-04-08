#ifndef CLOUDCAPTURE_H
#define CLOUDCAPTURE_H

#include "viewer.h"

#include "rtabmap/core/Odometry.h"
#include "rtabmap/core/Rtabmap.h"
#include "rtabmap/core/RtabmapThread.h"
#include "rtabmap/core/CameraRGBD.h"
#include "rtabmap/core/CameraStereo.h"
#include "rtabmap/core/CameraThread.h"
#include "rtabmap/core/OdometryThread.h"
#include "rtabmap/core/Graph.h"
#include "rtabmap/core/Signature.h"

#include "rtabmap/utilite/UEventsManager.h"
#include "pcl/io/pcd_io.h"
#include "pcl/io/ply_io.h"
#include "pcl/filters/filter.h"
#include "rtabmap/core/OctoMap.h"

#include "pcl/common/common_headers.h"
#include "pcl/features/normal_3d.h"
#include "pcl/io/vtk_io.h"
#include "pcl/io/io.h"
#include "pcl/io/vtk_lib_io.h"
#include "pcl/io/file_io.h"
#include "pcl/io/ply/ply_parser.h"
#include "pcl/io/ply/ply.h"
#include "pcl/visualization/pcl_visualizer.h"
#include "pcl/console/print.h"
#include "pcl/console/parse.h"
#include "pcl/console/time.h"
#include "pcl/range_image/range_image.h"
#include "pcl/visualization/pcl_visualizer.h"
#include "pcl/common/transforms.h"
#include "pcl/common/geometry.h"
#include "pcl/common/common.h"
#include "pcl/surface/vtk_smoothing/vtk_utils.h"
#include "pcl/surface/gp3.h"
#include "pcl/ModelCoefficients.h"
#include "pcl/point_types.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/project_inliers.h"
#include "pcl/filters/crop_box.h"
#include "pcl/surface/convex_hull.h"
#include "pcl/point_types.h"
//#include "pcl/features/gasd.h"
#include "pcl/features/normal_3d_omp.h"
#include "pcl/surface/poisson.h"
#include "pcl/surface/mls.h"
#include "pcl/surface/simplification_remove_unused_vertices.h"
#include "pcl/filters/crop_hull.h"
#include "pcl/search/search.h"
#include "pcl/search/kdtree.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/radius_outlier_removal.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/segmentation/extract_clusters.h"

#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <pcl/point_types.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "octomap/octomap.h"
#include "octomap/ColorOcTree.h"
#include "octomap/OcTree.h"
#include "octomap_msgs/Octomap.h"
#include "octomap_msgs/conversions.h"
#include <octomap/AbstractOcTree.h>

#include <QApplication>
#include <stdio.h>
#include <assert.h>
#include <iostream>
#include <fstream>
#include <string>
#include <stdlib.h>
#include <list>

#include "sqlite3.h"

#include <Eigen/Eigen>
#include "pcl/PolygonMesh.h"
#include "pcl/common/centroid.h"
#include "pcl/common/transforms.h"
#include "pcl/ModelCoefficients.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/surface/convex_hull.h"
#include "pcl/surface/mls.h"

using namespace std;
using namespace rtabmap;
class CloudCapture
{
public:
    std::string prefix = "/home/lamy/Documents/GitKraken/pointcloud_perception";

    void create_mesh(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,pcl::PolygonMesh &mesh);
    bool lamy_savePolygonFileSTL (const std::string &file_name,
                                 const pcl::PolygonMesh& mesh,
                                 const bool binary_format);
    void createMeshFromCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,pcl::PolygonMesh& triangles);
    void vizualizeMesh(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,pcl::PolygonMesh &mesh);
    void cloudPointFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr& filterCloud);
    void createOctomap(std::string pcdfilename);
    void Create_Cubes(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transform,float upperbound);
    void save_cloud(std::map<int, Signature> nodes, std::map<int, Transform> optimizedPoses, std::multimap<int, Link> links);
    void toMesh(pcl::PointCloud<pcl::PointXYZRGB> cloud);

private:

};

#endif // CLOUDCAPTURE_H
