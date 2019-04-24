#include <QApplication>
#include "viewer.h"
#include "cloudcapture.h"
#include "mainwindow.h"
#include "rtabmap/core/odometry/OdometryF2M.h"

int main(int argc, char** argv)
{
    ULogger::setType(ULogger::kTypeConsole);
    ULogger::setLevel(ULogger::kWarning);

    Camera *camera = 0;
    Transform opticalRotation(0,0,1,0, -1,0,0,0, 0,-1,0,0);
    camera = new CameraFreenect(0, CameraFreenect::kTypeColorDepth, 0, opticalRotation);
    //camera = new CameraRealSense2("",0,opticalRotation);
    //camera = new CameraFreenect2(0, CameraFreenect2::kTypeColor2DepthSD, 0, opticalRotation);
    if(!camera->init())
    {
     UERROR("Camera init failed!");
     //camera = new CameraFreenect(0, CameraFreenect::kTypeColorDepth, 0, opticalRotation);
     //camera = new CameraFreenect2(0, CameraFreenect2::kTypeColor2DepthSD, 0, opticalRotation);
     //camera = new CameraRealSense2("",0,opticalRotation);
     QApplication app(argc,argv);
     MainWindow win(nullptr,nullptr);
     win.show();
     app.exec();
    }else{
    CameraThread cameraThread(camera);
    Rtabmap * rtabmap = new Rtabmap();
    QApplication app(argc,argv);
    //Viewer cloud_viewer(&cameraThread);
    MainWindow win(&cameraThread,rtabmap);
    //OdometryThread odomThread(Odometry::create());
    OdometryThread odomThread(OdometryF2M::create());

    rtabmap->init();
    RtabmapThread rtabmapThread(rtabmap); // ownership is transfered

    // Setup handlers
    odomThread.registerToEventsManager();
    rtabmapThread.registerToEventsManager();
    //cloud_viewer.registerToEventsManager();
    win.registerToEventsManager();

    UEventsManager::createPipe(&cameraThread, &odomThread, "CameraEvent");
    // Let's start the threads
    rtabmapThread.start();
    odomThread.start();
    cameraThread.start();
    //cloud_viewer.show();
    win.show();
    app.exec();
    }
    return 0;
}
