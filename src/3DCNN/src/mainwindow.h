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

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    pcl::visualization::PCLVisualizer::Ptr viewer;

private:
    Ui::MainWindow *ui;

protected:
  PointCloudT::Ptr cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr inputcloud;
  QString fileName;
  //QFuture<void> cloud_segment_f = QtConcurrent::run(cloud_segment,inputcloud);

  QFuture<void>svm_trainer_f;

private slots:
  void on_actionOpen_triggered();
  void on_PredictButton_clicked();
  void on_SegButton_clicked();
  void on_TrainButton_clicked();
};


#endif // MAINWINDOW_H
