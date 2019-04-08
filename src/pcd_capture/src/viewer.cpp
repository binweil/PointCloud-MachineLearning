#include "viewer.h"
#include "ui_viewer.h"



using namespace rtabmap;

Viewer::Viewer(CameraThread * camera, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::Viewer),
    camera_(camera),
    odometryCorrection_(Transform::getIdentity()),
    processingStatistics_(false),
    lastOdometryProcessed_(true)
{
    ui->setupUi(this);
    viewer = new CloudViewer(this);
    ui->verticalLayout->addWidget(viewer);
    qRegisterMetaType<rtabmap::OdometryEvent>("rtabmap::OdometryEvent");
    qRegisterMetaType<rtabmap::Statistics>("rtabmap::Statistics");

    QAction * pause = new QAction(this);
    this->addAction(pause);
    pause->setShortcut(Qt::Key_Space);
    connect(pause, SIGNAL(triggered()), this, SLOT(pauseDetection()));
}

Viewer::~Viewer()
{
    this->unregisterFromEventsManager();
    delete ui;
}

void Viewer::pauseDetection()
{
 UWARN("");
 if(camera_)
 {
   if(camera_->isCapturing())
   {
     camera_->join(true);
   }
   else
   {
     camera_->start();
   }
 }
}

void Viewer::processOdometry(const rtabmap::OdometryEvent & odom)
{
 if(!this->isVisible())
 {
   return;
 }

 Transform pose = odom.pose();
 if(pose.isNull())
 {
   //Odometry lost
   viewer->setBackgroundColor(Qt::darkRed);

   pose = lastOdomPose_;
 }
 else
 {
   viewer->setBackgroundColor(viewer->getDefaultBackgroundColor());
 }
 if(!pose.isNull())
 {
   lastOdomPose_ = pose;

   // 3d cloud
   if(odom.data().depthOrRightRaw().cols == odom.data().imageRaw().cols &&
      odom.data().depthOrRightRaw().rows == odom.data().imageRaw().rows &&
      !odom.data().depthOrRightRaw().empty() &&
      (odom.data().stereoCameraModel().isValidForProjection() || odom.data().cameraModels().size()))
   {
     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = util3d::cloudRGBFromSensorData(
       odom.data(),
       2,     // decimation
       4.0f); // max depth
     if(cloud->size())
     {
       if(!viewer->addCloud("cloudOdom", cloud, odometryCorrection_*pose))
       {
         UERROR("Adding cloudOdom to viewer failed!");
       }
     }
     else
     {
       viewer->setCloudVisibility("cloudOdom", false);
       UWARN("Empty cloudOdom!");
     }
   }

   if(!odom.pose().isNull())
   {
     // update camera position
     viewer->updateCameraTargetPosition(odometryCorrection_*odom.pose());
   }
 }
 viewer->update();

 lastOdometryProcessed_ = true;
}


void Viewer::processStatistics(const rtabmap::Statistics & stats)
{
 processingStatistics_ = true;

 //============================
 // Add RGB-D clouds
 //============================
 const std::map<int, Transform> & poses = stats.poses();
 QMap<std::string, Transform> clouds = viewer->getAddedClouds();
 for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
 {
   if(!iter->second.isNull())
   {
     std::string cloudName = uFormat("cloud%d", iter->first);

     // 3d point cloud
     if(clouds.contains(cloudName))
     {
       // Update only if the pose has changed
       Transform tCloud;
       viewer->getPose(cloudName, tCloud);
       if(tCloud.isNull() || iter->second != tCloud)
       {
         if(!viewer->updateCloudPose(cloudName, iter->second))
         {
           UERROR("Updating pose cloud %d failed!", iter->first);
         }
       }
       viewer->setCloudVisibility(cloudName, true);
     }
     else if(0)//uContains(stats.getSignatures(), iter->first))
     {
       /*Signature s = stats.getSignatures().at(iter->first);
       s.sensorData().uncompressData(); // make sure data is uncompressed
       // Add the new cloud
       pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = util3d::cloudRGBFromSensorData(
           s.sensorData(),
             4,     // decimation
             4.0f); // max depth
       if(cloud->size())
       {
         if(!cloudViewer_->addCloud(cloudName, cloud, iter->second))
         {
           UERROR("Adding cloud %d to viewer failed!", iter->first);
         }
       }
       else
       {
         UWARN("Empty cloud %d!", iter->first);
       }*/
     }
   }
   else
   {
     UWARN("Null pose for %d ?!?", iter->first);
   }
 }

 //============================
 // Add 3D graph (show all poses)
 //============================
 viewer->removeAllGraphs();
 viewer->removeCloud("graph_nodes");
 if(poses.size())
 {
   // Set graph
   pcl::PointCloud<pcl::PointXYZ>::Ptr graph(new pcl::PointCloud<pcl::PointXYZ>);
   pcl::PointCloud<pcl::PointXYZ>::Ptr graphNodes(new pcl::PointCloud<pcl::PointXYZ>);
   for(std::map<int, Transform>::const_iterator iter=poses.begin(); iter!=poses.end(); ++iter)
   {
     graph->push_back(pcl::PointXYZ(iter->second.x(), iter->second.y(), iter->second.z()));
   }
   *graphNodes = *graph;


   // add graph
   viewer->addOrUpdateGraph("graph", graph, Qt::gray);
   viewer->addCloud("graph_nodes", graphNodes, Transform::getIdentity(), Qt::green);
   viewer->setCloudPointSize("graph_nodes", 5);
 }

 odometryCorrection_ = stats.mapCorrection();

 viewer->update();

 processingStatistics_ = false;
}

bool Viewer::handleEvent(UEvent * event)
{
 if(event->getClassName().compare("RtabmapEvent") == 0)
 {
   RtabmapEvent * rtabmapEvent = (RtabmapEvent *)event;
   const Statistics & stats = rtabmapEvent->getStats();
   // Statistics must be processed in the Qt thread
   if(this->isVisible())
   {
     QMetaObject::invokeMethod(this, "processStatistics", Q_ARG(rtabmap::Statistics, stats));
   }
 }
 else if(event->getClassName().compare("OdometryEvent") == 0)
 {
   OdometryEvent * odomEvent = (OdometryEvent *)event;
   // Odometry must be processed in the Qt thread
   if(this->isVisible() &&
      lastOdometryProcessed_ &&
      !processingStatistics_)
   {
     lastOdometryProcessed_ = false; // if we receive too many odometry events!
     QMetaObject::invokeMethod(this, "processOdometry", Q_ARG(rtabmap::OdometryEvent, *odomEvent));
   }
 }
 return false;
}

void Viewer::on_MainWindowButton_clicked()
{
    //MainWindow *w;
    //w = new MainWindow(this);
    //w->show();
//    Dialog *secdialog;
//    secdialog = new Dialog(this);
//    secdialog->show();
}
