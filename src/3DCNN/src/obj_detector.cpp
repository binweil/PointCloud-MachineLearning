#include "obj_detector.h"


template <class ContainerAllocator>
struct SVMclassifierGoal_
{
    typedef SVMclassifierGoal_<ContainerAllocator> Type;
    SVMclassifierGoal_()
        :order(){}
    SVMclassifierGoal_(const ContainerAllocator& _alloc)
        :order(_alloc){
        (void)_alloc;
    }
    typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other > _order_type;
    _order_type order;
    typedef boost::shared_ptr< ::SVMclassifierGoal_<ContainerAllocator> >Ptr;
    typedef boost::shared_ptr< ::SVMclassifierGoal_<ContainerAllocator> const> ConstPtr;
};

typedef SVMclassifierGoal_<std::allocator<void> > SVMclassifierGoal;


void cloud_cb (const pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud,
               pcl::visualization::PCLVisualizer::Ptr viewer)
{
    double voxelLeafSize =0.005;
    int removePlaneIterations = 100;
    double distanceThreshold = 0.02;
    double PercentageCloud =40;
    double tolerance =0.03 ;
    int minClustersSize =100;
    int maxClustersSize =25000;

    printf("Received cloud");
    pcl::PointCloud<pcl::PointXYZ>::Ptr newCloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> finalClusters;
    cloud_segmenter cs;
    processPointCloud p;

    pcl::PointXYZ minPT, maxPT;

    cs.load_cloud(inputCloud);
    cs.getVoxelFiltered(voxelLeafSize,voxelLeafSize,voxelLeafSize);
    cs.removePlanes(removePlaneIterations,distanceThreshold,PercentageCloud);
    finalClusters = cs.getClusters(tolerance,minClustersSize,maxClustersSize);
    ROS_INFO("Got Final clusters %d ", (int)finalClusters.size());

    // Publish Segmented Cloud
    sensor_msgs::PointCloud2 output;
    robot_vision_common::AssembleCloud(finalClusters,output);
    // Publish Segmented Cloud end

    //ros::Rate r(1);

    for(size_t i=0;i<finalClusters.size();i++)
    {
      p.cloudinput(finalClusters[i]);

      // send a goal to the action
      SVMclassifierGoal goal;
<<<<<<< Updated upstream
      //pcl::PointCloud<pcl::VFHSignature308>::Ptr inputDescriptor = p.getDescriptor();  //Causing error
      for(int idx=0;idx<306;idx++)
=======
      pcl::PointCloud<pcl::VFHSignature308>::Ptr inputDescriptor = p.getDescriptor();
      for(size_t idx=0;idx<306;idx++)
>>>>>>> Stashed changes
      {
          //goal.order.push_back((float)inputDescriptor->points[0].histogram[idx]);
      }
      /*// load classifiers
      std::string pkgPath = "/home/lamy/Desktop/PCD_MachineLearning/src/3DCNN";
      classifier bowlClf(pkgPath+"/bin/Classifier/bowl_1.xml");
      //classifier MugClf(pkgPath+"/bin/Classifier/coffee_mug_1.xml");
      cv::Mat testArray = cv::Mat::zeros(1, 306, CV_32FC1);
      for(size_t j=0; j<306;j++){
          testArray.at<float>(0,j)= (float)goal.order[j];
      }

      std::vector<float> predBowl = bowlClf.predict(testArray);
      //std::vector<float> predMug = MugClf.predict(testArray);
      std::vector<float> result_;

      result_.clear();
      result_.push_back(predBowl[0]);
      result_.push_back(predBowl[1]);
      //result_.push_back(predMug[0]);
      //result_.push_back(predMug[1]);
      std::cout << result_.at(0) << "    "<< result_.at(1)<<std::endl;
      pcl::getMinMax3D(*finalClusters[i],minPT,maxPT);
      robot_vision_common::detectObject(minPT,maxPT,result_,0.3,1.8,viewer,
                                        (int)i);*/
    } // END FOR LOOP

} //END cloud_cb
