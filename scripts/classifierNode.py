#! /usr/bin/env python
from std_msgs.msg import String
import pcl
import numpy as np
import os
from robot_vision_helper.GASD import GASD
from robot_vision_helper.CNNModel import CNNModel
from robot_vision_helper.DataHandler import DataHandler
import tensorflow as tf
from robot_vision_helper.common_constants import *
import glob
from PyQt5.QtCore import *

HEADER = '\033[95m'
OKBLUE = '\033[94m'
OKGREEN = '\033[92m'
WARNING = '\033[93m'
FAIL = '\033[91m'
ENDC = '\033[0m'
BOLD = '\033[1m'
UNDERLINE = '\033[4m'

#rospack = rospkg.RosPack()
PACKAGE_PATH = "/home/lamy/Desktop/scripts"
print(OKBLUE+PACKAGE_PATH+ENDC)

""" 3D-CNN is initailized to have grid size 40x40x40 """
cnn = CNNModel(ip_shape=(1, 40, 40, 40))
cnn.load_model(base_path=os.path.join(PACKAGE_PATH, "bin/3DCNN_model"))


# Call back runs on a different thread, thus we have to make graph global
# ref : https://www.tensorflow.org/api_docs/python/tf/get_default_graph
graph = tf.get_default_graph()


def is_in_dimension(cloud_array):
    """ check if the cloud segment is of required dimension i.e mxmxm"""
    max_dim = np.amax(cloud_array, axis=0)
    min_dim = np.amin(cloud_array, axis=0)
    return all((max_dim-min_dim) < BOX_SIZE), (max_dim, min_dim)


class NodeHandler(QThread):
    """ Handler for classification node"""

    def __init__(self):
        QThread.__init__(self)
        self.g = GASD()
        self.CLASS_PATH = ["apple_1", "banana_1",
                           "bowl_1", "calculator_1", "car_1"]
        self.d = DataHandler()

    def process_segment(self, cloud_list):
        """ Listern to topic /pcVector which is an array of 
            segmented pointclouds of type PointCloud2. 
            Classify each PointCloud and publish bounding boxes 
            to topic /visualization_marker """

        global graph
        #print("\n")
        #print("Received segments: " + str(len(cloud_list)))
        final_clusters = []
        with graph.as_default():
            # iterate over every segment of PointClouds
            for cloud_file in cloud_list:
                cloud = pcl.PointCloud()
                cloud.from_file(str.encode(cloud_file))
                cloud_array = cloud.to_array()
                #self.pc_to_numpy(cloud)
                dim_check, minmaxpt = is_in_dimension(cloud_array)
                # print(dim_check)
                if True:
                    volume_data, quaternion = self.g.get_volumetric_data(
                        cloud_array)
                    label, proba = cnn.predict(
                        np.array([[volume_data]]), self.CLASS_PATH)
                    #print(label, proba)
                
                    # check if prediction probability is above required threshold
                    # and suppress unnecessary labels
                    if (proba >= PRED_THRESHOLD) and (label in ["car_1"]):
                        final_clusters.append((minmaxpt[0],
                                               minmaxpt[1], label,
                                               proba, quaternion))
        #drawMarkers(final_clusters)
        return final_clusters

    def segmentedfile_loader(self,active_file):
        """ Initialize topics and start the node """
        #cloud_list = [f for f in glob.glob("/home/lamy/Desktop/scripts/Segmented/*.pcd")]
        cloud_list = [f for f in glob.glob(active_file)]
        final_cluster = self.process_segment(cloud_list)
        return final_cluster

#
# if __name__ == '__main__':
#     nh = NodeHandler()
#     nh.segmentedfile_loader()
#     os.system("./3DCNN_node")
