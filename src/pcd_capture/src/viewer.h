#ifndef VIEWER_H
#define VIEWER_H

#include <QMainWindow>
#include <QVBoxLayout>
#include <QVTKWidget.h>
#include <QtCore/QMetaType>
#include <QAction>

#ifndef Q_MOC_RUN // Mac OS X issue
#include "rtabmap/gui/CloudViewer.h"
#include "rtabmap/core/util3d.h"
#include "rtabmap/core/util3d_filtering.h"
#include "rtabmap/core/util3d_transforms.h"
#include "rtabmap/core/RtabmapEvent.h"
#endif
#include "rtabmap/utilite/UStl.h"
#include "rtabmap/utilite/UConversion.h"
#include "rtabmap/utilite/UEventsHandler.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/core/OdometryEvent.h"
#include "rtabmap/core/CameraThread.h"

#include "mainwindow.h"

using namespace rtabmap;

namespace Ui {
class Viewer;
}

class Viewer : public QMainWindow, public UEventsHandler
{
    Q_OBJECT

public:
    Viewer(CameraThread * camera, QWidget *parent = nullptr);
    ~Viewer();

private:
    Ui::Viewer *ui;
    CloudViewer *viewer;
    CameraThread * camera_;
    Transform lastOdomPose_;
    Transform odometryCorrection_;
    bool processingStatistics_;
    bool lastOdometryProcessed_;
private slots:
    virtual void pauseDetection();
    virtual void processOdometry(const rtabmap::OdometryEvent & odom);
    virtual void processStatistics(const rtabmap::Statistics & stats);
    virtual bool handleEvent(UEvent * event);
    void on_MainWindowButton_clicked();
};

#endif // VIEWER_H
