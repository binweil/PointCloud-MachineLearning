#include "segmentation.h"

void cloud_segment (const pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud)
{
    double voxelLeafSize =0.005;
    int removePlaneIterations = 100;
    double distanceThreshold = 0.02;
    double PercentageCloud =40;
    double tolerance =0.03 ;
    int minClustersSize =100;
    int maxClustersSize =25000;
    std::string segmented_file_path = "/home/lamy/Desktop/PCD_MachineLearning/Segmented_Cloud/";

    printf("Received Cloud");
    pcl::PointCloud<pcl::PointXYZ>::Ptr newCloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> finalClusters;
    cloud_segmenter cs;
    processPointCloud p;

    pcl::PointXYZ minPT, maxPT;

    cs.load_cloud(inputCloud);
    cs.getVoxelFiltered(voxelLeafSize,voxelLeafSize,voxelLeafSize);
    cs.removePlanes(removePlaneIterations,distanceThreshold,PercentageCloud);
    finalClusters = cs.getClusters(tolerance,minClustersSize,maxClustersSize);
    //ROS_INFO("Got Final clusters %d ", (int)finalClusters.size());
    printf("Got Final clusters %d ", (int)finalClusters.size());

    // Publish Segmented Cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr someCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2 pc;
    for(size_t i=0;i<finalClusters.size();i++){
        pcl::io::savePCDFileASCII(segmented_file_path+"segmented_cloud"+std::to_string(i)+ ".pcd", *finalClusters[i]);
        std::cout << "Segmented Cloud Saved" << std::endl;
        for(size_t j=0;j<finalClusters[i]->points.size();j++){
            someCloud->points.push_back(finalClusters[i]->points.at(j));
        }
    }
    someCloud->width=1;
    someCloud->height=someCloud->points.size();
    //pcl::io::savePCDFileASCII(segmented_file_path+"segmented_cloud.pcd",*someCloud);
} //END cloud_cb

int main(int argc, char** argv)
{
    std::string fileName = "/home/lamy/Desktop/PCD_MachineLearning/data/kinect_original.pcd";
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputcloud;
    inputcloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ> (fileName,*inputcloud);
    cloud_segment(inputcloud);
}
