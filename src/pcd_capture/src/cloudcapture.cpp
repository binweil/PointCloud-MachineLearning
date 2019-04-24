#include "cloudcapture.h"
using namespace std;
using namespace octomap;

using namespace std;
float transformation[7];

void CloudCapture::create_mesh(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,pcl::PolygonMesh &mesh){
  int nThreads=8;
  int setKsearch=10;
  int depth=10;
  float pointWeight=4.0;
  float samplePNode=1.5;
  float scale=1.0;
  int isoDivide=5;
  bool confidence=true;
  bool outputPolygons=true;
  bool manifold=true;
  int solverDivide=5;
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud, centroid);
  std::cout << centroid << std::endl;

  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() << -centroid[0], -centroid[1], -centroid[2];

  std::cout << transform.matrix() << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTranslated(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud(*cloud, *cloudTranslated, transform);


  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);


    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (10);

    seg.setInputCloud (cloudTranslated);
    seg.segment (*inliers, *coefficients);

    // Project the model inliers
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (cloudTranslated);
    proj.setIndices (inliers);
    proj.setModelCoefficients (coefficients);
    proj.filter (*cloud_projected);

    // Create a Convex Hull representation of the projected inliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConvexHull<pcl::PointXYZ> chull;
    chull.setInputCloud (cloud_projected);
    chull.reconstruct (*cloud_hull);

    std::cout << "Convex hull has: " << cloud_hull->points.size () << " data points." << std::endl;
    std::cout << "Convex hull volume:" << chull.getTotalVolume() << std::endl;

    pcl::PolygonMesh ms2;

    pcl::PCDWriter writer;
    writer.write ("/home/lamy/Desktop/PCD_MachineLearning/cloud_convex_hull.pcd", *cloud_hull, false);
   // pcl::io::savePolygonFilePLY("cloud_convex_hull2.ply", ms2, false);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());

  // Output has the PointNormal type in order to store the normals calculated by MLS
  pcl::PointCloud<pcl::PointNormal>::Ptr mls_points (new pcl::PointCloud<pcl::PointNormal>());

  // Init object (second point type is for the normals, even if unused)
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

  mls.setComputeNormals (true);

  // Set parameters
  mls.setInputCloud (cloudTranslated);
  mls.setDilationIterations(10);
  mls.setPointDensity(30);
  mls.setSearchMethod(tree);
  mls.setSearchRadius(5);

  // Reconstruct
  mls.process (*mls_points);

  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_smoothed_normals (new pcl::PointCloud<pcl::PointNormal> ());
  pcl::concatenateFields (*cloudTranslated, *mls_points, *cloud_smoothed_normals);//x

  pcl::Poisson<pcl::PointNormal> poisson;

  poisson.setDepth(depth);//9
  poisson.setInputCloud(cloud_smoothed_normals);
  poisson.setPointWeight(pointWeight);//4
  poisson.setDegree(2);
  poisson.setSamplesPerNode(samplePNode);//1.5
  poisson.setScale(scale);//1.1
  poisson.setIsoDivide(isoDivide);//8
  poisson.setConfidence(confidence);
  poisson.setOutputPolygons(outputPolygons);
  poisson.setManifold(manifold);
  poisson.setSolverDivide(solverDivide);//8

  pcl::PolygonMesh mesh2;
  poisson.reconstruct(mesh2);

  pcl::surface::SimplificationRemoveUnusedVertices rem;
  rem.simplify(mesh2,mesh);

}

bool CloudCapture::lamy_savePolygonFileSTL (const std::string &file_name,
                             const pcl::PolygonMesh& mesh,
                             const bool binary_format)
{
  vtkSmartPointer<vtkPolyData> poly_data = vtkSmartPointer<vtkPolyData>::New ();

  pcl::io::mesh2vtk (mesh, poly_data);
  vtkSmartPointer<vtkSTLWriter> poly_writer = vtkSmartPointer<vtkSTLWriter>::New ();
  poly_writer->SetInputData (poly_data);

  if (binary_format)
    poly_writer->SetFileTypeToBinary ();
  else
    poly_writer->SetFileTypeToASCII ();

  poly_writer->SetFileName (file_name.c_str ());
  return (poly_writer->Write ());
}

 void CloudCapture::createMeshFromCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,pcl::PolygonMesh& triangles){

     pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
     pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

     pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

     Eigen::Vector4f centroid;

     //pcl::compute3DCentroid(*cloud, centroid);
     //std::cout << centroid << std::endl;

     Eigen::Affine3f transform = Eigen::Affine3f::Identity();
     transform.translation() << 0,0,0;

     //std::cout << transform.matrix() << std::endl;

     pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTranslated(new pcl::PointCloud<pcl::PointXYZ>());
     pcl::transformPointCloud(*cloud, *cloudTranslated, transform);

     tree->setInputCloud (cloudTranslated);
     n.setInputCloud (cloudTranslated);
     n.setSearchMethod (tree);
     n.setKSearch (20);        //It was 20
     n.compute (*normals);                //Normals are estimated using standard method.

     pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal> ());
     pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);//x

     // Create search tree*
     pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
     tree2->setInputCloud (cloud_with_normals);

     // Initialize objects
     pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;

     std::cout << "Applying surface meshing..." <<std::endl;

     // Set the maximum distance between connected points (maximum edge length)
     gp3.setSearchRadius(0.025);           //It was 0.025

     // Set typical values for the parameters
     gp3.setMu (2.5); //It was 2.5
     gp3.setMaximumNearestNeighbors (100);    //It was 100
     gp3.setNormalConsistency(false); //It was false

     // Get result
     gp3.setInputCloud (cloud_with_normals);
     gp3.setSearchMethod (tree2);
     gp3.reconstruct (triangles);

     vtkSmartPointer<vtkPolyData> polydata= vtkSmartPointer<vtkPolyData>::New();

     pcl::PolygonMesh mms2;

     pcl::VTKUtils::convertToVTK(triangles,polydata);
     pcl::VTKUtils::convertToPCL(polydata,mms2);

     std::cout << "Saving Mesh to stl" <<std::endl;
     //pcl::io::savePolygonFilePLY("output_mesh.ply", mms2);
     //pcl::io::savePolygonFileSTL("output_mesh.stl", mms2);
     lamy_savePolygonFileSTL(prefix+"/data/map_test.stl",mms2,true);
     std::cout << "Mesh Saved" <<std::endl;

 }

void CloudCapture::vizualizeMesh(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,pcl::PolygonMesh &mesh){

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("MAP3D MESH"));

int PORT1 = 0;
viewer->createViewPort(0.0, 0.0, 0.5, 1.0, PORT1);
viewer->setBackgroundColor (0, 0, 0, PORT1);
viewer->addText("ORIGINAL", 10, 10, "PORT1", PORT1);
viewer->addPointCloud(cloud,"original_cloud",PORT1);

int PORT2 = 0;
viewer->createViewPort(0.5, 0.0, 1.0, 1.0, PORT2);
viewer->setBackgroundColor (0, 0, 0, PORT2);
viewer->addText("MESH", 10, 10, "PORT2", PORT2);
viewer->addPolygonMesh(mesh,"mesh",PORT2);

viewer->setBackgroundColor (0, 0, 0);
viewer->addCoordinateSystem (1.0);
viewer->initCameraParameters ();
viewer->resetCamera();

while (!viewer->wasStopped ()){
    viewer->spin();
}
}

void CloudCapture::cloudPointFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr& filterCloud){

  std::cout << "Filtering point cloud..." << std::endl;
  std::cout << "Point cloud before filter:" << cloud->points.size()<< std::endl;

  pcl::RadiusOutlierRemoval<pcl::PointXYZ> radius_outlier_removal;
  radius_outlier_removal.setInputCloud(cloud);
  radius_outlier_removal.setRadiusSearch(0.01);
  radius_outlier_removal.setMinNeighborsInRadius(1);
  radius_outlier_removal.filter(*filterCloud);

  std::cout << "Point cloud after filter:" << filterCloud->points.size() << std::endl;
}


void CloudCapture::createOctomap(std::string pcdfilename)
{
  pcl::PointCloud<pcl::PointXYZRGBA> pointcloud;
  pcl::io::loadPCDFile<pcl::PointXYZRGBA> (pcdfilename, pointcloud );
  octomap::ColorOcTree tree(0.01);
  for (auto p:pointcloud.points)
  {
    tree.updateNode(octomap::point3d(p.x,p.y,p.z),true);
  }
  for (auto p:pointcloud.points)
  {
      tree.integrateNodeColor( p.x, p.y, p.z, p.r, p.g, p.b );
  }
  tree.updateInnerOccupancy();
  std::cout << "Saving Octomap to bt & ot" << std::endl;
  tree.write(prefix+"/data/map_test.ot");
  tree.writeBinary(prefix+"/data/map_test.bt");
  std::cout << "Octomap Saved" <<std::endl;
}

void CloudCapture::Create_Cubes(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transform,float upperbound)
{
  for (size_t i = 0; i < 100; ++i)
  {
    cloud_transform->points[i].x = 1.2+0.1*rand()/(RAND_MAX + 1.0f);
    cloud_transform->points[i].y = -1.2+0.1*rand()/(RAND_MAX + 1.0f);
    cloud_transform->points[i].z = -0.2+0.1*rand()/(RAND_MAX + 1.0f);
  }
  for (size_t i = 100; i < 200; ++i)
  {
    cloud_transform->points[i].x = -1.2+0.1*rand()/(RAND_MAX + 1.0f);
    cloud_transform->points[i].y = 1.2+0.1*rand()/(RAND_MAX + 1.0f);
    cloud_transform->points[i].z = -0.2+0.1*rand()/(RAND_MAX + 1.0f);
  }
  for (size_t i = 200; i < 300; ++i)
  {
    cloud_transform->points[i].x = 1.2+0.1*rand()/(RAND_MAX + 1.0f);
    cloud_transform->points[i].y = 1.2+0.1*rand()/(RAND_MAX + 1.0f);
    cloud_transform->points[i].z = -0.2+0.1*rand()/(RAND_MAX + 1.0f);
  }
  for (size_t i = 300; i < 400; ++i)
  {
    cloud_transform->points[i].x = -1.2+0.1*rand()/(RAND_MAX + 1.0f);
    cloud_transform->points[i].y = -1.2+0.1*rand()/(RAND_MAX + 1.0f);
    cloud_transform->points[i].z = -0.2+0.1*rand()/(RAND_MAX + 1.0f);
  }
  for (size_t i = 400; i < 500; ++i)
  {
    cloud_transform->points[i].x = 1.2+0.1*rand()/(RAND_MAX + 1.0f);
    cloud_transform->points[i].y = -1.2+0.1*rand()/(RAND_MAX + 1.0f);
    cloud_transform->points[i].z = upperbound+0.1*rand()/(RAND_MAX + 1.0f);
  }
  for (size_t i = 500; i < 600; ++i)
  {
    cloud_transform->points[i].x = -1.2+0.1*rand()/(RAND_MAX + 1.0f);
    cloud_transform->points[i].y = 1.2+0.1*rand()/(RAND_MAX + 1.0f);
    cloud_transform->points[i].z = upperbound+0.1*rand()/(RAND_MAX + 1.0f);
  }
  for (size_t i = 600; i < 700; ++i)
  {
    cloud_transform->points[i].x = 1.2+0.1*rand()/(RAND_MAX + 1.0f);
    cloud_transform->points[i].y = 1.2+0.1*rand()/(RAND_MAX + 1.0f);
    cloud_transform->points[i].z = upperbound+0.1*rand()/(RAND_MAX + 1.0f);
  }
  for (size_t i = 700; i < 800; ++i)
  {
    cloud_transform->points[i].x = -1.2+0.1*rand()/(RAND_MAX + 1.0f);
    cloud_transform->points[i].y = -1.2+0.1*rand()/(RAND_MAX + 1.0f);
    cloud_transform->points[i].z = upperbound+0.1*rand()/(RAND_MAX + 1.0f);
  }
  pcl::io::savePCDFile(prefix+"/data/Cloud_with_cube.pcd",*cloud_transform);
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr CloudCapture::save_cloud(std::map<int, Signature> nodes, std::map<int, Transform> optimizedPoses, std::multimap<int, Link> links){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr saved_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for(std::map<int, Transform>::iterator iter=optimizedPoses.begin(); iter!=optimizedPoses.end(); ++iter)
    {
      Signature node = nodes.find(iter->first)->second;
     // uncompress data
     node.sensorData().uncompressData();
     pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp = util3d::cloudRGBFromSensorData(
         node.sensorData(),
         4,           // image decimation before creating the clouds
         4.0f,        // maximum depth of the cloud
         0.0f);
     pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpNoNaN(new pcl::PointCloud<pcl::PointXYZRGB>);
     std::vector<int> index;
     pcl::removeNaNFromPointCloud(*tmp, *tmpNoNaN, index);



     if(!tmpNoNaN->empty())
     {
       *cloud += *util3d::transformPointCloud(tmpNoNaN, iter->second); // transform the point cloud to its pose
     }
    }
    if(cloud->size())
    {
     printf("Voxel grid filtering of the assembled cloud (voxel=%f, %d points)\n", 0.01f, (int)cloud->size());
     cloud = util3d::voxelize(cloud, 0.01f);
     copyPointCloud(*cloud,*saved_cloud);
     std::cout << "Saving Path: " << prefix << "/data/kinect_original.pcd" << std::endl;
     pcl::io::savePCDFile(prefix+"/data/kinect_original.pcd", *cloud);
     printf("Saving kinect_original.pcd... done! (%d points)\n", (int)cloud->size());

    }
    else
    {
     printf("Saving kinect_original.pcd... failed! The cloud is empty.\n");
    }

    // Save trajectory
    printf("Saving rtabmap_trajectory.txt ...\n");
    if(optimizedPoses.size() && graph::exportPoses(prefix+"/data/rtabmap_trajectory.txt", 0, optimizedPoses, links))
    {
     printf("Saving rtabmap_trajectory.txt... done!\n");
    }
    else
    {
     printf("Saving rtabmap_trajectory.txt... failed!\n");
    }
    return saved_cloud;
}


static int sql_callback(void* data, int argc, char** argv, char** azColName)
{
    int i;
    for (i = 0; i < argc; i++) {
        transformation[i] = strtof(argv[i],nullptr);
    }
    return 0;
};

void CloudCapture::toMesh(pcl::PointCloud<pcl::PointXYZRGB> cloud){
    /**********************************************************************************************************/
    // Convert Pointcloud:cloud to mesh and save as stl
    pcl::console::print_color(stdout,10,pcl::console::TT_YELLOW,"Processing Point Cloud\n");

    cloud.width = (int) cloud.points.size ();
    cloud.height = 1;
    cloud.is_dense = true;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(cloud,*cloud_xyz);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_filtered (new pcl::PointCloud<pcl::PointXYZ>());
    cloudPointFilter(cloud_xyz,cloud_xyz_filtered);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transform (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(*cloud_xyz_filtered,*cloud_transform);

    //Apply Transformation
    Eigen::Vector3f kinect_self_translate(0,0,0);
    Eigen::Quaternionf kinect_self_rotate1(0.5,-0.5,-0.5,-0.5);
    pcl::transformPointCloud(*cloud_transform,*cloud_transform,kinect_self_translate,kinect_self_rotate1);
    Eigen::Quaternionf kinect_self_rotate2(0,0,0,1);
    pcl::transformPointCloud(*cloud_transform,*cloud_transform,kinect_self_translate,kinect_self_rotate2);
    sqlite3 *database;

    std::string database_path = "/data/transform_matrix.db";
    sqlite3_open((prefix+database_path).c_str(),&database);
    sqlite3_exec(database,"SELECT * FROM translation",sql_callback,NULL,NULL);
    sqlite3_close(database);
    Eigen::Vector3f translation(transformation[0],transformation[1],transformation[2]);
    Eigen::Quaternionf quat(transformation[3],transformation[4],transformation[5],transformation[6]);
    pcl::transformPointCloud(*cloud_transform,*cloud_transform,translation,quat);

    const float xmin = 0.2;
    const float ymin = 0.2;
    //Remove robot
    /*for (size_t j=0; j<cloud_transform->points.size();++j)
    {
      if ((cloud_transform->points[j].x<xmin) && (cloud_transform->points[j].y<ymin))
      {
        cloud_transform->points[j].x = 0;
        cloud_transform->points[j].y = 0;
        cloud_transform->points[j].z = -1;
      }
    }*/

    //Create_Cubes(cloud_transform,2.0);
    pcl::PolygonMesh cloud_mesh;
    createMeshFromCloud(cloud_transform,cloud_mesh); //create mesh and save
    createOctomap(prefix+"/data/Cloud_with_cube.pcd");
    vizualizeMesh(cloud_transform,cloud_mesh);
}

