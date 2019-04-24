import sys
import os
from VTKPointcloud import VtkPointCloud
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtWidgets import QApplication, QDialog, QMainWindow
from PyQt5.uic import loadUi
from PyQt5 import Qt
import vtk
from trainer import trainModel
from numpy import random
from classifierNode import NodeHandler
import pcl as pcl
from Cluster_extract import cloud_cluster
import sqlite3
import time

def boxCallback(obj, event):
    t = vtk.vtkTransform()
    obj.GetTransform(t)
    obj.GetProp3D().SetUserTransform(t)


class cloud_viewer(Qt.QMainWindow, QThread):
    active_file = "";
    def __init__(self,parent = None):
        Qt.QMainWindow.__init__(self,parent)
        QThread.__init__(self)
        super(cloud_viewer, self).__init__()
        loadUi("mainwindow.ui", self)
        self.setWindowTitle('Point Cloud Training Tool')
        self.frame = Qt.QFrame()
        pointCloud = VtkPointCloud()
        for k in range(1000):
            point = 20*(random.rand(3)-0.5)
            pointCloud.addPoint(point)

        renderer = vtk.vtkRenderer()
        renderer.AddActor(pointCloud.vtkActor)
        renderer.SetBackground(0, 0, 0)
        renderer.ResetCamera()
        self.iren = self.widget.GetRenderWindow().GetInteractor()
        self.widget.GetRenderWindow().AddRenderer(renderer)

        self.iren.Initialize()
        self.iren.Start()

        self.git_thread = trainModel()

    @pyqtSlot()
    def on_TrainButton_clicked(self):
        #self.git_thread.start()
        h = trainModel()
        h.run()

    @pyqtSlot()
    def on_actionOpen_triggered(self):
        fname = QFileDialog.getOpenFileName(self, 'QFileDialog.getOpenFileName()', '/home/lamy', "PCD Files (*.pcd)")
        pointcloud = VtkPointCloud()
        p = pcl.PointCloud()
        p.from_file(str.encode(fname[0]))
        self.active_file = fname[0]
        cloud_array = p.to_array()
        for k in range(0,len(cloud_array)):
            pointcloud.addPoint(cloud_array[k])

        # Renderer
        renderer = vtk.vtkRenderer()
        renderer.AddActor(pointcloud.vtkActor)
        renderer.SetBackground(0, 0, 0)
        renderer.ResetCamera()

        self.widget.GetRenderWindow().AddRenderer(renderer)
        self.iren = self.widget.GetRenderWindow().GetInteractor()
        self.iren.Initialize()
        self.iren.Start()
        print("Opening pcd file: "+fname[0])

    @pyqtSlot()
    def on_PredictButton_clicked(self):
        start_time = time.time()
        path = "/home/lamy/Desktop/PCD_MachineLearning/data/result"
        conn = sqlite3.connect(path)
        c = conn.cursor()
        nh = NodeHandler()
        print(self.active_file)
        final_clusters = nh.segmentedfile_loader("/home/lamy/Desktop/PCD_MachineLearning/Segmented_Cloud/*.pcd")
        id=0
        c.execute("DELETE FROM minmax")
        for clusters in final_clusters:
            minPT = clusters[1]
            maxPT = clusters[0]
            label = clusters[2]
            prob = clusters[3]
            print(label, prob)
            #print(minPT, maxPT, label)
            try:
                c.execute("INSERT INTO minmax (id, minx, miny, minz, maxx,maxy,maxz,name) \
                VALUES ({index},{mx},{my},{mz},{max},{may},{maz},{n})". \
                          format(index=id, mx=minPT[0], my=minPT[1], mz=minPT[2], max=maxPT[0], may=maxPT[1], maz=maxPT[2], n=1))
            except sqlite3.IntegrityError:
                print('ERROR: ID already exists in PRIMARY KEY column {}'.format(id_column))
            id = id + 1
        conn.commit()
        conn.close()
        print("Prediction Completed")
        print("Time: ", time.time()-start_time)

    @pyqtSlot()
    def on_SegButton_clicked(self):
        print("Clustering Cloud")
        start_time = time.time()
        path = "/home/lamy/Desktop/PCD_MachineLearning/data/result"
        conn = sqlite3.connect(path)
        c = conn.cursor()
        final_cluster = cloud_cluster(self.active_file)
        id=0
        c.execute("DELETE FROM minmax")
        for clusters in final_cluster:
            minPT = clusters[1]
            maxPT = clusters[0]
            label = clusters[2]
            prob = clusters[4]
            print(prob)
            #print(minPT, maxPT, label)
            try:
                c.execute("INSERT INTO minmax (id, minx, miny, minz, maxx,maxy,maxz,name) \
                VALUES ({index},{mx},{my},{mz},{max},{may},{maz},{n})". \
                          format(index=id, mx=minPT[0], my=minPT[1], mz=minPT[2], max=maxPT[0], may=maxPT[1], maz=maxPT[2], n=1))
            except sqlite3.IntegrityError:
                print('ERROR: ID already exists in PRIMARY KEY column {}'.format(id_column))
            id = id + 1
        conn.commit()
        conn.close()
        print("Prediction Completed")
        print("Time: ", time.time()-start_time)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    mainwindow = cloud_viewer()
    mainwindow.show()
    sys.exit(app.exec_())
