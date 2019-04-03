#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->setWindowTitle("PCD Viewer");

    // Set up the QVTK window
    viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
    ui->widget->SetRenderWindow (viewer->getRenderWindow ());
    viewer->setupInteractor (ui->widget->GetInteractor (), ui->widget->GetRenderWindow ());
    ui->widget->update ();

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
}

void MainWindow::on_PredictButton_clicked()
{
    fileName = QFileDialog::getOpenFileName(this, tr("Open File"),"/path/to/file/",tr("pcd Files (*.pcd)"));
    inputcloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ> (fileName.toStdString(),*inputcloud);
    viewer->removeAllShapes();
    cloud_cb(inputcloud,viewer);
    viewer->removeAllPointClouds();
    viewer->addPointCloud(inputcloud,"inputcloud");
    viewer->addCoordinateSystem(1);
    viewer->resetCamera ();
    ui->widget->update();

}

void MainWindow::on_SegButton_clicked()
{
    fileName = QFileDialog::getOpenFileName(this, tr("Open File"),"/path/to/file/",tr("pcd Files (*.pcd)"));
    inputcloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ> (fileName.toStdString(),*inputcloud);
    //cloud_segment_f.waitForFinished();
    cloud_segment(inputcloud);
}

void MainWindow::on_TrainButton_clicked()
{
    std::vector<std::string> possitive_class[4];
   // possitive_class->push_back("apple_1");
    //possitive_class->push_back("banana_1");// = {,"banana_1","calculator_1","coffe_mug_1"};
    possitive_class->push_back("bowl_1");
    //possitive_class->push_back("calculator_1");
    //possitive_class->push_back("coffe_mug_1");
    for(int i=0; i<possitive_class->size();i++)
    {
    svm_trainer_f =QtConcurrent::run(svm_trainer,possitive_class->at(i));
    //svm_trainer(possitive_class->at(i));
    }
}
