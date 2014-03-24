#include "vtkpointcloudwidget.h"
#include "vtkRenderWindow.h"
#include <QVTKWidget.h>


pcl::visualization::PCLVisualizer vis ("vis", false);

VTKPointCloudWidget::VTKPointCloudWidget(QWidget *parent) : QVTKWidget(parent)
{
}

void VTKPointCloudWidget::loadPolygon(QString path)
{
    this->resize(750, 750);

    filePath = path.toStdString();
    pcl::io::loadPolygonFile(filePath, mesh_of_triangles);
    vis.addPolygonMesh(mesh_of_triangles);

    vtkSmartPointer<vtkRenderWindow> renderWindow = vis.getRenderWindow();
    this->SetRenderWindow(renderWindow);
    this->show();
}


void VTKPointCloudWidget::showPointCloud()
{
    //vis.addPolygonMesh()

    this->update();
}
