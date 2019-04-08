#include "svm_trainer.h"

class getPointCloud{
    private :
        struct stat info;
    public :
        std::string PCD_BASE_PATH;
        std::string PCD_CLASS_PATH;
        std::string PCD_FILE_PATH;
        std::string PCD_FILE_EXT;
        std::string FULL_FILE_PATH;
        std::string CLASSIFIER_NAME;
        std::vector<std::string> DataFiles;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
        pcl::PointCloud<pcl::VFHSignature308>::Ptr VFH_descriptor;

        ofstream outputFile;

        processPointCloud pc;
        classifier clf;

        getPointCloud(std::string _PCD_BASE_PATH,
                      std::vector<std::string> _DataFiles,
                      std::string _CLASSIFIER_NAME);

        pcl::PointCloud<pcl::PointXYZ>::Ptr loadCloud(std::string filePath);

        void getTrainingData(std::vector <cv::Mat>& _trainingData,
                             std::vector <cv::Mat>& _trainingLabels);
        int getdir (std::string dir, std::vector<std::string> &files);

};

/*
 * Constructor
 */
getPointCloud::getPointCloud(std::string _PCD_BASE_PATH,
              std::vector<std::string> _DataFiles,
              std::string _CLASSIFIER_NAME): cloud(new pcl::PointCloud<pcl::PointXYZ>),
                     VFH_descriptor(new pcl::PointCloud<pcl::VFHSignature308>),
                     clf()
{


    PCD_BASE_PATH = _PCD_BASE_PATH;
    CLASSIFIER_NAME = _CLASSIFIER_NAME;
    for(size_t i=0;i<_DataFiles.size();i++){
        DataFiles.push_back(PCD_BASE_PATH +_DataFiles[i]);
    }

}

pcl::PointCloud<pcl::PointXYZ>::Ptr
getPointCloud::loadCloud(std::string filePath){
    pcl::PointCloud<pcl::PointXYZ>::Ptr someCloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (filePath, *someCloud) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    }

    return someCloud;
}


void
getPointCloud::getTrainingData(std::vector <cv::Mat>& _trainingData,
                               std::vector <cv::Mat>& _trainingLabels){
    float minX,minY,minZ=1000.0;
    float maxX,maxY,maxZ=0.0;

    pcl::VoxelGrid<pcl::PointXYZ> vg;

    cv::Mat label = cv::Mat::zeros(1,1,CV_32FC1);

    // Loop over each Directory
    for(size_t fileIdx=0; fileIdx<DataFiles.size();fileIdx++){
        std::vector<std::string> files = std::vector<std::string>();
        pcl::PointXYZ minPt, maxPt;
        int skip=0;

        getdir(DataFiles[fileIdx]+"/",files);
        printf("No of Files: %d\n",(int)files.size());

            cv::Mat trainingData;
            cv::Mat trainingLabels;
        // Loop over each file
        for (unsigned int i = 0;i < files.size();i++) {
//                 remove this after testing classifier training
//            if(skip > 5){break;}
//            skip++;

            if(files[i] == "."  || files[i] == ".."){
                //ROS_INFO(" . or .. files ignored");
                printf(". or .. files ignored");
            }
            else{
                std::cout << " [ processing ] " << files[i];

                cloud = loadCloud(DataFiles[fileIdx]+"/"+files[i]);

                //-- voxelization of the cloud --------//
                vg.setInputCloud (cloud);
                vg.setLeafSize (0.005f, 0.005f, 0.005f);
                vg.filter (*cloud);
                //-- Getting the descriptors ----------//
                pc.cloudinput(cloud);
                VFH_descriptor = pc.getDescriptor();
                cv::Mat _descriptor = cv::Mat::zeros(1,306,CV_32FC1);

                for(size_t i=0;i<306;i++){
                    _descriptor.at<float>(0,i)=(float)VFH_descriptor->points[0].histogram[i];
                }
                //-------------------------------------//
                trainingData.push_back(_descriptor);
                if(DataFiles[fileIdx] == (PCD_BASE_PATH+CLASSIFIER_NAME)){
                    label.at<float>(0,0)= 1;
                    trainingLabels.push_back(label);
                    std::cout << " [label] 1";
                }
                else{
                    label.at<float>(0,0)=-1;
                    trainingLabels.push_back(label);
                    std::cout << " [label] -1";
                }
                std::cout << std::endl;

            }
        }
        _trainingData.push_back(trainingData);
        _trainingLabels.push_back(trainingLabels);
    }
}


int
getPointCloud::getdir (std::string dir, std::vector<std::string> &files)
{
    DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(dir.c_str())) == NULL) {
        std::cout << "Error(" << errno << ") opening " << dir << endl;
        return errno;
    }

    while ((dirp = readdir(dp)) != NULL) {
        files.push_back(std::string(dirp->d_name));
    }
    closedir(dp);
    return 0;
}

void write2file(std::string DataFilePath,std::string Fname ,cv::Mat Data){
        cv::FileStorage file(DataFilePath, cv::FileStorage::WRITE);
        file << Fname << Data;
        file.release();
}

void readFromFile(std::string DataFilePath,std::string Fname ,cv::Mat &Data){
    cv::FileStorage file(DataFilePath, cv::FileStorage::READ);
    file[Fname] >> Data;
    file.release();
}

void svm_trainer(std::string POSITIVE_CLASS){
    std::string pkgPath = "/home/lamy/Desktop/PCD_MachineLearning/src/3DCNN";
    printf("Trainning SVM\n");
    std::string PCD_BASE_PATH = "/home/lamy/Downloads/Dataset_RGBD/";
    bool parsePCD =true,trainSVM=true;
    std::vector<std::string> PCD_CLASS_PATHS = {"apple_1","banana_1","calculator_1", "bowl_1","coffee_mug_1"};

    if (parsePCD) {
        printf("Parameters %s %s\n",PCD_BASE_PATH.c_str(),"true");
    }else{
        printf("Parameters %s %s\n",PCD_BASE_PATH.c_str(),"false");
    }
    for(size_t i=0;i<PCD_CLASS_PATHS.size();i++){
        printf("class: %s\n",PCD_CLASS_PATHS[i].c_str());
    }
    printf("Done\n");
    std::string trainDataFile = pkgPath+"/bin/Dataset/";
    std::string trainLabelsFile = pkgPath+"/bin/Dataset/trainingLabels";
    std::string classifierPath = pkgPath+"/bin/Classifier/"+POSITIVE_CLASS;
    printf("trainData: %s\n",trainDataFile.c_str());
    printf("trainLabels: %s\n",trainLabelsFile.c_str());

    classifier clf_bowl;
    if(parsePCD){
        std::vector< cv::Mat > _trainingData;
        std::vector< cv::Mat > _trainingLabels;

        getPointCloud p(PCD_BASE_PATH,PCD_CLASS_PATHS,POSITIVE_CLASS);
        p.getTrainingData(_trainingData,_trainingLabels);

        mkdir((pkgPath+"/bin/Dataset").c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        for (size_t f=0;f<_trainingData.size();f++){
            ROS_INFO("file: %s",PCD_CLASS_PATHS[f].c_str());
            ROS_INFO("Training Data dimensions: rows: %d cols: %d",
                     (int)_trainingData[f].rows,
                     (int)_trainingData[f].cols);
            ROS_INFO("Training Labels dimensions: rows: %d cols: %d",
                     (int)_trainingLabels[f].rows,
                     (int)_trainingLabels[f].cols);
            std::stringstream idx;
            idx << f;
            write2file(trainDataFile+PCD_CLASS_PATHS[f]+".xml",
                       PCD_CLASS_PATHS[f],
                       _trainingData[f]);
        }


        ROS_INFO("Training Data saved.");
    }

    if(trainSVM){
        cv::Mat trainingData;
        cv::Mat trainingLabels;


        cv::Mat tempLabels = cv::Mat::zeros(1,1,CV_32FC1);
        float labelval=-1.0;

        for(size_t f=0;f< PCD_CLASS_PATHS.size();f++){
            cv::Mat temptrainingData;
            readFromFile(trainDataFile+PCD_CLASS_PATHS[f]+".xml",
                         PCD_CLASS_PATHS[f],
                         temptrainingData);
            trainingData.push_back(temptrainingData);

            if (PCD_CLASS_PATHS[f] == POSITIVE_CLASS){
                labelval=1.0;
            }
            else{
                labelval= -1.0;
            }

            for(size_t d=0;d<temptrainingData.rows;d++){
                tempLabels.at<float>(0,0)=labelval;
                trainingLabels.push_back(tempLabels);
            }
            ROS_INFO("[ CLASS] %s [ LABEL] %f",PCD_CLASS_PATHS[f].c_str(),labelval);
        }
        printf("Files Loaded\n");
        printf("training SVM...\n");
        clf_bowl.trainSVM(trainingData,trainingLabels,classifierPath);
        printf("SVM trained\n");
    }
}
