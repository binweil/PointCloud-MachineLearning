#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <iostream>
#include <QFileDialog>
#include <QString>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <vtkRenderWindow.h>

#include <QMainWindow>
#include <QThread>
#include <qthread.h>
#include <QtDebug>
#include <QtConcurrent/qtconcurrentrun.h>
#include <vtkRenderWindow.h>
#include "svm_trainer.h"
#include "segmentation.h"
#include "obj_detector.h"



#include <QVBoxLayout>
#include <QVTKWidget.h>
#include <QtCore/QMetaType>
#include <QAction>

#ifndef Q_MOC_RUN // Mac OS X issue
#include "rtabmap/gui/CloudViewer.h"
#include "rtabmap/core/util3d.h"
#include "rtabmap/core/util3d_filtering.h"
#include "rtabmap/core/util3d_transforms.h"
#include "rtabmap/core/RtabmapEvent.h"
#endif
#include "rtabmap/utilite/UStl.h"
#include "rtabmap/utilite/UConversion.h"
#include "rtabmap/utilite/UEventsHandler.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/core/OdometryEvent.h"
#include "rtabmap/core/CameraThread.h"

#include "rtabmap/core/camera/CameraRealSense2.h"
#include "rtabmap/core/Rtabmap.h"
#include "rtabmap/core/Odometry.h"
#include "rtabmap/core/OdometryThread.h"
#include "rtabmap/core/RtabmapThread.h"
#include "rtabmap/utilite/UEventsManager.h"

#include "viewer.h"
#include "cloudcapture.h"

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

using namespace rtabmap;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow, public UEventsHandler
{
    Q_OBJECT

public:
    explicit MainWindow(CameraThread * camera, Rtabmap * rtabmap, QWidget *parent = nullptr);
    ~MainWindow();
    pcl::visualization::PCLVisualizer::Ptr viewer;

private:
    Ui::MainWindow *ui;
    CloudViewer *cloud_viewer;
    CameraThread * camera_;
    Rtabmap * rtabmap_;
    Transform lastOdomPose_;
    Transform odometryCorrection_;
    bool processingStatistics_;
    bool lastOdometryProcessed_;

    float scale;
    bool showcoord = true;


protected:
  PointCloudT::Ptr cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr inputcloud;
  QString fileName;
  //QFuture<void> cloud_segment_f;// = QtConcurrent::run(cloud_segment,inputcloud);
 // QFuture<void>svm_trainer_f;

private slots:
  void on_actionOpen_triggered();
  //void on_PredictButton_clicked();
  //void on_SegButton_clicked();
  //void on_TrainButton_clicked();

  virtual void pauseDetection();
  virtual void processOdometry(const rtabmap::OdometryEvent & odom);
  virtual void processStatistics(const rtabmap::Statistics & stats);
  virtual bool handleEvent(UEvent * event);

  void on_saveButton_clicked();
  //void on_radioButton_clicked(bool checked);
  void on_resetButton_clicked();
  void on_actionload_triggered();
  void on_predictButton_clicked();
  void on_pushButton_clicked();
  void on_segButton_2_clicked();
};


#endif // MAINWINDOW_H
