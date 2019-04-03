#include <robot_vision_common.h>


namespace robot_vision{
template < typename T >
std::string to_string( const T& n )
{
    std::ostringstream stm ;
    stm << n ;
    return stm.str() ;
}
}

void
robot_vision_common::AssembleCloud(
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &finalClusters,
        sensor_msgs::PointCloud2 &output)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr someCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2 pc;
    for(size_t i=0;i<finalClusters.size();i++){
        for(size_t j=0;j<finalClusters[i]->points.size();j++){
            someCloud->points.push_back(finalClusters[i]->points.at(j));
        }
    }

    pcl::toPCLPointCloud2(*someCloud,pc);
    pcl_conversions::fromPCL(pc, output);
    output.header.frame_id=std::string("world");
}


bool
robot_vision_common::inDimensions(pcl::PointXYZ minPT,pcl::PointXYZ maxPT,
                  double length,double width,double height,double scale)
{
    std::cout << std::abs(maxPT.x - minPT.x) <<"   "<< scale*length <<std::endl;
    std::cout << std::abs(maxPT.y - minPT.y) <<"   "<< scale*width <<std::endl;
    std::cout << std::abs(maxPT.z - minPT.z) <<"   "<< scale*height <<std::endl;

    if((std::abs(maxPT.y - minPT.y) < scale*width) &&
            (std::abs(maxPT.x - minPT.x) < scale*length) &&
            (std::abs(maxPT.z - minPT.z) < scale*height)){
        return true;
    }
    else{
        return false;
    }
}


void
robot_vision_common::detectObject(
                pcl::PointXYZ &minPT,pcl::PointXYZ &maxPT,
                std::vector<float> result,
                double threshold, double scale,
                pcl::visualization::PCLVisualizer::Ptr viewer,
                int _id) // updates Marker vector
{
    int no_of_classes = result.size();

    for (size_t clf_idx=0; clf_idx<no_of_classes;clf_idx+=2)
    {
        if (result.at(clf_idx)==1.0){// && result.at(clf_idx+1)>threshold){
//            pcl::getMinMax3D(*cloudSegment,minPT,maxPT);
            if(inDimensions(minPT,maxPT,bowlLength,bowlWidth,bowlHeight,scale))
          {
                //Marker_vector.push_back(bb.getBoundingBox(minPT.x,minPT.y,minPT.z,
                //                                          maxPT.x,maxPT.y,maxPT.z,// x,y,z
                //                                          color_array[clf_idx/2],_id));
                printf("min location: %f %f %f\n,",minPT.x,minPT.y,minPT.z);
                viewer->addCube(minPT.x,maxPT.x,
                                minPT.y,maxPT.y,
                                minPT.z,maxPT.z,
                                1,1,1,"marker"+std::to_string(_id));
                _id++;
           }

        }
    }

}
