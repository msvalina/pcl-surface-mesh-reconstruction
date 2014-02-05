#include "meshreconstruction.h"
#include "qdebug.h"

MeshReconstruction::MeshReconstruction(QObject *parent) :
    QObject(parent)
{
}

void MeshReconstruction::setFilePath(QString path)
{
    filePath = path.toStdString();
    if(path.isEmpty())
        filePath = "/home/maki/geek/source/masters-thesis/mesh-reconstruction-build-desktop-Qt_4_8_1_in_PATH__System__Debug/ts-mesh-depth-4.vtk";
}

void MeshReconstruction::showMesh()
{
    pcl::PolygonMesh mesh_of_triangles;
    pcl::io::loadPolygonFile(filePath, mesh_of_triangles);
    qDebug() << "Started - show_mesh() with PCLVisualizer\n";
    // Create viewer object and show mesh
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new
          pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPolygonMesh (mesh_of_triangles, "sample mesh");
    viewer->initCameraParameters ();
    // Setting type of mesh representation
    // Wireframe = standard "mesh" representation
    viewer->setRepresentationToWireframeForAllActors ();
    // viewer->setRepresentationToSurfaceForAllActors ();
    // viewer->setRepresentationToPointsForAllActors ();
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100); boost::this_thread::sleep
            (boost::posix_time::microseconds (100000));
    }
    qDebug() << "Finshed - show_mesh() with PCLVisualizer\n";
}

