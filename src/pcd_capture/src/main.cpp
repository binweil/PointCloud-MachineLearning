#include <QApplication>
#include "viewer.h"
#include "cloudcapture.h"
#include "mainwindow.h"

int main(int argc, char** argv)
{
    ULogger::setType(ULogger::kTypeConsole);
    ULogger::setLevel(ULogger::kWarning);

    Camera *camera = nullptr;
    Transform opticalRotation(0,0,1,0, -1,0,0,0, 0,-1,0,0);
    //camera = new CameraFreenect(0, CameraFreenect::kTypeColorDepth, 0, opticalRotation);
    camera = new CameraRealSense2("",0,opticalRotation);
    if(!camera->init())
    {
     UERROR("Camera init failed!");
     camera = new CameraRealSense2("",0,opticalRotation);
     sleep(1);
    }
    CameraThread cameraThread(camera);
    Rtabmap * rtabmap = new Rtabmap();
    QApplication app(argc,argv);
    //Viewer cloud_viewer(&cameraThread);
    MainWindow win(&cameraThread,rtabmap);
    OdometryThread odomThread(Odometry::create());
    //OdometryThread odomThread(Odometry::create());


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

    return 0;
}
