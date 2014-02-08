#include "meshreconstruction.h"
#include "qdebug.h"

MeshReconstruction::MeshReconstruction(QObject *parent) :
    QObject(parent)
{
}

void MeshReconstruction::setFilePath(QString path)
{
    filePath = path.toStdString();
}

void MeshReconstruction::downsample(LogWindow *logWin)
{
    logWin->appendMessage("Started - downsample() with VoxelGrid");

    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2());

    // Fill in the cloud data
    pcl::PCDReader reader;
    reader.read (filePath, *cloud);

    logWin->appendMessage("PointCloud before filtering: ");
    QString before;
    before = QString::number(cloud->width * cloud->height) + " data points (" +
            QString::fromStdString(pcl::getFieldsList (*cloud)) + ").";
    logWin->appendMessage(before);

    // Create the filtering object
    pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
    vg.setInputCloud (cloud);
    // voxel size to be 1cm^3
    vg.setLeafSize (0.01f, 0.01f, 0.01f);
    vg.filter (*cloud_filtered);

    logWin->appendMessage("PointCloud after filtering:");
    QString after;
    after = QString::number(cloud_filtered->width * cloud_filtered->height) + " data points (" +
            QString::fromStdString(pcl::getFieldsList (*cloud_filtered)) + ").";
    logWin->appendMessage(after);

    pcl::PCDWriter writer;
    std::string str;
    str.append(filePath.append("-downsampled.pcd"));
    writer.write (str, *cloud_filtered, Eigen::Vector4f::Zero(),
                Eigen::Quaternionf::Identity(), false);

    logWin->appendMessage("Finished - downsample() with VoxelGrid");
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

