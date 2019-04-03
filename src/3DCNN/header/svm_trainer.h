#ifndef SVM_TRAINER_H
#define SVM_TRAINER_H

#include <ros/ros.h>
#include <ros/package.h>
#include <stdio.h>
#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/vfh.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>

#include <fstream>
#include <sys/types.h>
#include <sys/stat.h>

#include "header/descriptors.h"
#include <header/classifier.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml/ml.hpp>

void svm_trainer(std::string POSITIVE_CLASS);
void readFromFile(std::string DataFilePath,std::string Fname ,cv::Mat &Data);

#endif // SVM_TRAINER_H
