#ifndef VTKPOINTCLOUDWIDGET_H
#define VTKPOINTCLOUDWIDGET_H

#include <QObject>
#include <QWidget>
#include "vtkRenderWindow.h"
#include <QVTKWidget.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "logwindow.h"
#include <pcl/visualization/pcl_visualizer.h>

class VTKPointCloudWidget : public QVTKWidget
{
    Q_OBJECT

public slots:
    void representWithSurface();
    void representWithWire();


public:
    VTKPointCloudWidget(QWidget *parent);
    void loadPolygon(QString path);
    void showPointCloud();


private:
    std::string filePath;
    pcl::PolygonMesh mesh_of_triangles;

};

#endif // VTKPOINTCLOUDWIDGET_H
