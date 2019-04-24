#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <sstream>
#include "sqlite3.h"
using namespace std;
float minmaxPT[10][9];
int j;
MainWindow::MainWindow(CameraThread * camera,  Rtabmap * rtabmap, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    camera_(camera),
    rtabmap_(rtabmap),
    odometryCorrection_(Transform::getIdentity()),
    processingStatistics_(false),
    lastOdometryProcessed_(true)
{
    ui->setupUi(this);
    this->setWindowTitle("Point Cloud Capturing and Predict Tool");
    // Set up the QVTK window
    viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
    ui->widget->SetRenderWindow (viewer->getRenderWindow ());
    viewer->setupInteractor (ui->widget->GetInteractor (), ui->widget->GetRenderWindow ());
    ui->widget->update();

    if (camera!= nullptr){
    cloud_viewer = new CloudViewer(this);
    ui->verticalLayout_camera->addWidget(cloud_viewer);
    qRegisterMetaType<rtabmap::OdometryEvent>("rtabmap::OdometryEvent");
    qRegisterMetaType<rtabmap::Statistics>("rtabmap::Statistics");

    QAction * pause = new QAction(this);
    this->addAction(pause);
    pause->setShortcut(Qt::Key_Space);
    connect(pause, SIGNAL(triggered()), this, SLOT(pauseDetection()));
    }

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_actionOpen_triggered()
{
    fileName = QFileDialog::getOpenFileName(this, tr("Open File"),"/path/to/file/",tr("pcd Files (*.pcd)"));
    cloud.reset (new PointCloudT);
    pcl::io::loadPCDFile<PointT> (fileName.toStdString(), *cloud);
    viewer->removeAllPointClouds();
    viewer->addPointCloud (cloud, "cloud");
    viewer->addCoordinateSystem(1);
    viewer->resetCamera ();
    ui->widget->update ();
    pcl::io::savePCDFile("/home/lamy/Desktop/PCD_MachineLearning/data/kinect_original.pcd",*cloud);
}

//void MainWindow::on_PredictButton_clicked()
//{
//    pcl::PointXYZ minPt, maxPt;
//    fileName = QFileDialog::getOpenFileName(this, tr("Open File"),"/path/to/file/",tr("pcd Files (*.pcd)"));
//    inputcloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::io::loadPCDFile<pcl::PointXYZ> (fileName.toStdString(),*inputcloud);
//    viewer->removeAllShapes();
//    cloud_cb(inputcloud,viewer);
//    viewer->removeAllPointClouds();
//    viewer->addPointCloud(inputcloud,"inputcloud");
//    pcl::getMinMax3D(*inputcloud,minPt,maxPt);
//    scale = std::max({std::abs(maxPt._PointXYZ::x-minPt._PointXYZ::x),std::abs(maxPt._PointXYZ::y-minPt._PointXYZ::y),std::abs(maxPt._PointXYZ::z-minPt._PointXYZ::z)});
//    viewer->addCoordinateSystem(scale);
//    viewer->resetCamera ();
//    ui->widget->update();
//}

//void MainWindow::on_SegButton_clicked()
//{
//    fileName = QFileDialog::getOpenFileName(this, tr("Open File"),"/path/to/file/",tr("pcd Files (*.pcd)"));
//    inputcloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::io::loadPCDFile<pcl::PointXYZ> (fileName.toStdString(),*inputcloud);
//    //cloud_segment_f.waitForFinished();
//    //cloud_segment(inputcloud);
//    cloud_segment_f = QtConcurrent::run(cloud_segment,inputcloud);
//    std::string fileName = "/home/lamy/Desktop/PCD_MachineLearning/data/kinect_original.pcd";

      //comment above
//    pcl::PointCloud<pcl::PointXYZ>::Ptr inputcloud1;
//    inputcloud1.reset(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::io::loadPCDFile<pcl::PointXYZ> (fileName,*inputcloud1);
//    cloud_segment(inputcloud1);
//}

//void MainWindow::on_TrainButton_clicked()
//{
//    std::vector<std::string> possitive_class[4];
//    possitive_class->push_back("apple_1");
//    //possitive_class->push_back("banana_1");// = {,"banana_1","calculator_1","coffe_mug_1"};
//    possitive_class->push_back("bowl_1");
//    //possitive_class->push_back("calculator_1");
//    //possitive_class->push_back("coffe_mug_1");
//    for(int i=0; i<possitive_class->size();i++)
//    {
//    //svm_trainer_f =QtConcurrent::run(svm_trainer,possitive_class->at(i));
//    //svm_trainer(possitive_class->at(i));
//    }
//}


void MainWindow::pauseDetection()
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

void MainWindow::processOdometry(const rtabmap::OdometryEvent & odom)
{
 if(!this->isVisible())
 {
   return;
 }

 Transform pose = odom.pose();
 if(pose.isNull())
 {
   //Odometry lost
   cloud_viewer->setBackgroundColor(Qt::darkRed);
   pose = lastOdomPose_;
 }
 else
 {
   cloud_viewer->setBackgroundColor(cloud_viewer->getDefaultBackgroundColor());
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
       if(!cloud_viewer->addCloud("cloudOdom", cloud, odometryCorrection_*pose))
       {
         UERROR("Adding cloudOdom to viewer failed!");
       }
     }
     else
     {
       cloud_viewer->setCloudVisibility("cloudOdom", false);
       UWARN("Empty cloudOdom!");
     }
   }

   if(!odom.pose().isNull())
   {
     // update camera position
     cloud_viewer->updateCameraTargetPosition(odometryCorrection_*odom.pose());
   }
 }
 cloud_viewer->update();

 lastOdometryProcessed_ = true;
}


void MainWindow::processStatistics(const rtabmap::Statistics & stats)
{
 processingStatistics_ = true;

 //============================
 // Add RGB-D clouds
 //============================
 const std::map<int, Transform> & poses = stats.poses();
 QMap<std::string, Transform> clouds = cloud_viewer->getAddedClouds();
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
       cloud_viewer->getPose(cloudName, tCloud);
       if(tCloud.isNull() || iter->second != tCloud)
       {
         if(!cloud_viewer->updateCloudPose(cloudName, iter->second))
         {
           UERROR("Updating pose cloud %d failed!", iter->first);
         }
       }
       cloud_viewer->setCloudVisibility(cloudName, true);
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
 cloud_viewer->removeAllGraphs();
 cloud_viewer->removeCloud("graph_nodes");
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
   cloud_viewer->addOrUpdateGraph("graph", graph, Qt::gray);
   cloud_viewer->addCloud("graph_nodes", graphNodes, Transform::getIdentity(), Qt::green);
   cloud_viewer->setCloudPointSize("graph_nodes", 5);
 }

 odometryCorrection_ = stats.mapCorrection();

 cloud_viewer->update();

 processingStatistics_ = false;
}

bool MainWindow::handleEvent(UEvent * event)
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

void MainWindow::on_saveButton_clicked()
{
    // Save 3D map
    //printf("Saving rtabmap_cloud.pcd...\n");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr saved_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::map<int, Signature> nodes;
    std::map<int, Transform> optimizedPoses;
    std::multimap<int, Link> links;
    rtabmap_->get3DMap(nodes, optimizedPoses, links, false, true);
    CloudCapture capture;
    saved_cloud = capture.save_cloud(nodes,optimizedPoses,links);

    viewer->removeAllPointClouds();
    viewer->addPointCloud (saved_cloud, "saved_cloud");
    viewer->addCoordinateSystem(1);
    viewer->resetCamera ();
    ui->widget->update ();

    system("rm /home/lamy/Desktop/PCD_MachineLearning/Segmented_Cloud/*.pcd");
    system("rosrun cloudsegment cloudsegment_node");
    // remove handlers
    //cloud_viewer.unregisterFromEventsManager();
    //rtabmapThread.unregisterFromEventsManager();
    //odomThread.unregisterFromEventsManager();

    // Kill all threads
    //cameraThread.kill();
    //odomThread.join(true);
    //rtabmapThread.join(true);
    //rtabmap->close(false);

    //pcl::PointCloud<pcl::PointXYZRGB> cloud;
    //pcl::io::loadPCDFile<pcl::PointXYZRGB>(capture.prefix+"/data/kinect_original.pcd", cloud );
    //capture.toMesh(cloud);
}

//void MainWindow::on_radioButton_clicked(bool checked)
//{
//    showcoord = !showcoord;
//    std::cout << checked <<std::endl;
//    std::cout << showcoord <<std::endl;
//    if (showcoord){
//        viewer->removeCoordinateSystem();
//        viewer->resetCamera ();
//        ui->widget->update();
//    }else {
//        viewer->addCoordinateSystem(scale);
//        viewer->resetCamera();
//        ui->widget->update();
//    }

//}

void MainWindow::on_resetButton_clicked()
{
    //this->post(new OdometryResetEvent());
    UEventsManager::post(new OdometryResetEvent());
}


static int sql_callback(void* data, int argc, char** argv, char** azColName)
{
    int i;
    for (i = 0; i < argc; i++) {
        minmaxPT[j][i] = strtof(argv[i],nullptr);
    }
    j++;
    return 0;
};

void MainWindow::on_actionload_triggered()
{
   j=0;
   sqlite3 *database;
   std::string prefix = "/home/lamy/Desktop/PCD_MachineLearning";
   std::string database_path = "/data/result";
   sqlite3_open((prefix+database_path).c_str(),&database);
   sqlite3_exec(database,"SELECT * FROM minmax",sql_callback,NULL,NULL);
   sqlite3_close(database);
   for (int id = 0; id <= j; id++){
        viewer->addCube(minmaxPT[id][1],minmaxPT[id][4],minmaxPT[id][2],minmaxPT[id][5],minmaxPT[id][3],minmaxPT[id][6],1,1,1,"cube"+to_string(id));
   }
   ui->widget->update();
}

void MainWindow::on_predictButton_clicked()
{
    system("rm /home/lamy/Desktop/PCD_MachineLearning/Segmented_Cloud/*.pcd");
    system("rosrun cloudsegment cloudsegment_node");
}

void MainWindow::on_pushButton_clicked()
{
    viewer->removeAllShapes();
    j=0;
    sqlite3 *database;
    std::string prefix = "/home/lamy/Desktop/PCD_MachineLearning";
    std::string database_path = "/data/result";
    sqlite3_open((prefix+database_path).c_str(),&database);
    sqlite3_exec(database,"SELECT * FROM minmax",sql_callback,NULL,NULL);
    sqlite3_close(database);
    for (int id = 0; id < j; id++){
         viewer->addCube(minmaxPT[id][1],minmaxPT[id][4],minmaxPT[id][2],minmaxPT[id][5],minmaxPT[id][3],minmaxPT[id][6],1,1,1,"cube"+to_string(id));
         PointT position(minmaxPT[id][1],minmaxPT[id][2],minmaxPT[id][3]);
         cout << position << endl;
         string name;
         if (minmaxPT[id][7] == 1){
             name = "CAR";
         }else{
             name = "Obstacle";
         }
         bool res = viewer->addText3D(name,position,0.3,1,1,1,"text"+to_string(id));
    }
    ui->widget->update();
}

void MainWindow::on_segButton_2_clicked()
{
    system("rm /home/lamy/Desktop/PCD_MachineLearning/Segmented_Cloud/*.pcd");
    system("rosrun cloudsegment cloudsegment_node");
}
