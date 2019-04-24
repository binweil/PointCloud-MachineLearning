import numpy as np
import pcl
from robot_vision_helper.GASD import GASD
from robot_vision_helper.CNNModel import CNNModel
import os

def is_in_dimension(cloud_array):
    BOX_SIZE = (0.3, 0.3, 0.3)
    """ check if the cloud segment is of required dimension i.e mxmxm"""
    max_dim = np.amax(cloud_array, axis=0)
    min_dim = np.amin(cloud_array, axis=0)
    return all((max_dim-min_dim) < BOX_SIZE), (max_dim, min_dim)

def cloud_cluster(fname):
    # int main (int argc, char** argv)
    # {
    #   // Read in the cloud data
    #   pcl::PCDReader reader;
    #   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
    #   reader.read ("table_scene_lms400.pcd", *cloud);
    #   std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*
    cloud = pcl.load(fname)

    #   // Create the filtering object: downsample the dataset using a leaf size of 1cm
    #   pcl::VoxelGrid<pcl::PointXYZ> vg;
    #   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    #   vg.setInputCloud (cloud);
    #   vg.setLeafSize (0.01f, 0.01f, 0.01f);
    #   vg.filter (*cloud_filtered);
    #   std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*
    vg = cloud.make_voxel_grid_filter()
    vg.set_leaf_size(0.005, 0.005, 0.005)
    cloud_filtered = vg.filter()

    #   // Create the segmentation object for the planar model and set all the parameters
    #   pcl::SACSegmentation<pcl::PointXYZ> seg;
    #   pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    #   pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    #   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
    #   pcl::PCDWriter writer;
    #   seg.setOptimizeCoefficients (true);
    #   seg.setModelType (pcl::SACMODEL_PLANE);
    #   seg.setMethodType (pcl::SAC_RANSAC);
    #   seg.setMaxIterations (100);
    #   seg.setDistanceThreshold (0.02);
    seg = cloud.make_segmenter()
    seg.set_optimize_coefficients(True)
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    seg.set_MaxIterations(100)
    seg.set_distance_threshold(0.02)

    #   int i=0, nr_points = (int) cloud_filtered->points.size ();
    #   while (cloud_filtered->points.size () > 0.3 * nr_points)
    #   {
    #     // Segment the largest planar component from the remaining cloud
    #     seg.setInputCloud (cloud_filtered);
    #     seg.segment (*inliers, *coefficients);
    #     if (inliers->indices.size () == 0)
    #     {
    #       std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    #       break;
    #     }
    #     // Extract the planar inliers from the input cloud
    #     pcl::ExtractIndices<pcl::PointXYZ> extract;
    #     extract.setInputCloud (cloud_filtered);
    #     extract.setIndices (inliers);
    #     extract.setNegative (false);
    #
    #     // Get the points associated with the planar surface
    #     extract.filter (*cloud_plane);
    #     std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
    #
    #     // Remove the planar inliers, extract the rest
    #     extract.setNegative (true);
    #     extract.filter (*cloud_f);
    #     *cloud_filtered = *cloud_f;
    #   }

    i = 0
    nr_points = cloud_filtered.size
    # while nr_points > 0.3 * nr_points:
    #     # Segment the largest planar component from the remaining cloud
    #     [inliers, coefficients] = seg.segment()
    #     # extract = cloud_filtered.extract()
    #     # extract = pcl.PointIndices()
    #     cloud_filtered.extract(extract)
    #     extract.set_Indices (inliers)
    #     extract.set_Negative (false)
    #     cloud_plane = extract.filter ()
    #
    #     extract.set_Negative (True)
    #     cloud_f = extract.filter ()
    #     cloud_filtered = cloud_f


    # Creating the KdTree object for the search method of the extraction
    # pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    # tree->setInputCloud (cloud_filtered);
    tree = cloud_filtered.make_kdtree()
    # tree = cloud_filtered.make_kdtree_flann()


    # std::vector<pcl::PointIndices> cluster_indices;
    # pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    # ec.setClusterTolerance (0.02); // 2cm
    # ec.setMinClusterSize (100);
    # ec.setMaxClusterSize (25000);
    # ec.setSearchMethod (tree);
    # ec.setInputCloud (cloud_filtered);
    # ec.extract (cluster_indices);
    ec = cloud_filtered.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.03)
    ec.set_MinClusterSize(100)
    ec.set_MaxClusterSize(25000)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()

    print('cluster_indices : ' + str(cluster_indices.count) + " count.")
    # print('cluster_indices : ' + str(cluster_indices.indices.max_size) + " count.")

    #   int j = 0;
    #   for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    #   {
    #     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    #     for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    #       cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    #     cloud_cluster->width = cloud_cluster->points.size ();
    #     cloud_cluster->height = 1;
    #     cloud_cluster->is_dense = true;
    #
    #     std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    #     std::stringstream ss;
    #     ss << "cloud_cluster_" << j << ".pcd";
    #     writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
    #     j++;
    #   }
    #

    cloud_cluster = pcl.PointCloud()

    """ 3D-CNN is initailized to have grid size 40x40x40 """
    PACKAGE_PATH = "/home/lamy/Desktop/scripts"
    cnn = CNNModel(ip_shape=(1, 40, 40, 40))
    cnn.load_model(base_path=os.path.join(PACKAGE_PATH, "bin/3DCNN_model"))
    for j, indices in enumerate(cluster_indices):
        # cloudsize = indices
        print('indices = ' + str(len(indices)))
        # cloudsize = len(indices)
        points = np.zeros((len(indices), 3), dtype=np.float32)
        # points = np.zeros((cloudsize, 3), dtype=np.float32)

        # for indice in range(len(indices)):
        for i, indice in enumerate(indices):
            # print('dataNum = ' + str(i) + ', data point[x y z]: ' + str(cloud_filtered[indice][0]) + ' ' + str(cloud_filtered[indice][1]) + ' ' + str(cloud_filtered[indice][2]))
            # print('PointCloud representing the Cluster: ' + str(cloud_cluster.size) + " data points.")
            points[i][0] = cloud_filtered[indice][0]
            points[i][1] = cloud_filtered[indice][1]
            points[i][2] = cloud_filtered[indice][2]

        cloud_cluster.from_array(points)

        # self.pc_to_numpy(cloud)
        g = GASD()
        CLASS_PATH = ["apple_1", "banana_1",
                           "bowl_1", "calculator_1", "car_1"]
        #d = DataHandler()
        dim_check, minmaxpt = is_in_dimension(points)
        #print(dim_check)
        final_clusters = []
        if True:
            volume_data, quaternion = g.get_volumetric_data(
                points)
            label, proba = cnn.predict(
                np.array([[volume_data]]),CLASS_PATH)
            print(label, proba)

            # check if prediction probability is above required threshold
            # and suppress unnecessary labels
            if (proba >= 0.99):  # and (label in ["coffee_mug_1",
                #               "bowl_1"]):
                final_clusters.append((minmaxpt[0],
                                       minmaxpt[1], label,
                                       proba, quaternion))
                print(minmaxpt[0], minmaxpt[1])
                # return minmaxpt, label, proba
        ss = "cloud_cluster_" + str(j) + ".pcd";
        pcl.save(cloud_cluster, ss)
        return final_clusters