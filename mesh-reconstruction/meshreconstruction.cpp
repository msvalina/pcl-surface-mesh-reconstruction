#include "meshreconstruction.h"
#include "qdebug.h"

typedef pcl::Normal Normal;
//typedef pcl::PointXYZ PointType;
//typedef pcl::PointNormal PointTypeN;
typedef pcl::PointXYZRGB PointType;
typedef pcl::PointXYZRGBNormal PointTypeN;

MeshReconstruction::MeshReconstruction(QObject *parent) :
    QObject(parent)
{
    setPoissonParams();
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
    after = QString::number(cloud_filtered->width * cloud_filtered->height) +
            " data points (" + QString::fromStdString(pcl::getFieldsList
            (*cloud_filtered)) + ").";
    logWin->appendMessage(after);

    pcl::PCDWriter writer;
    std::string str;
    str.append(filePath.append("-downsampled.pcd"));
    writer.write (str, *cloud_filtered, Eigen::Vector4f::Zero(),
                Eigen::Quaternionf::Identity(), false);

    logWin->appendMessage("Finished - downsample() with VoxelGrid \n");
}

void MeshReconstruction::removeOutliers(LogWindow *logWin)
{
    logWin->appendMessage("Started - remove_outliers() with "
                          "StatisticalOutlierRemoval");

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
    pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloud);
    // Set number of neighbors to analyze
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud_filtered);

    logWin->appendMessage("PointCloud after filtering: ");
    QString after;
    after = QString::number(cloud_filtered->width * cloud_filtered->height) +
            " data points (" + QString::fromStdString(pcl::getFieldsList
            (*cloud_filtered)) + ").";
    logWin->appendMessage(after);

    pcl::PCDWriter writer;

    logWin->appendMessage("Writing both inliers and outliers");
    std::string str1;
    str1.append(filePath.append("-inliers.pcd"));
    std::string str2;
    str2.append(filePath.append("-outliers.pcd"));
    writer.write (str1, *cloud_filtered, Eigen::Vector4f::Zero(),
            Eigen::Quaternionf::Identity(), false);

    sor.setNegative (true);
    sor.filter (*cloud_filtered);
    writer.write (str2, *cloud_filtered, Eigen::Vector4f::Zero(),
            Eigen::Quaternionf::Identity(), false);

    logWin->appendMessage("Finished - remove_outliers() with "
                          "StatisticalOutlierRemoval\n");
}

void MeshReconstruction::setPoissonParams(int depth, int solverDivide,
                                          int isoDivide, int samplesPerNode,
                                          float scale, bool confidence)
{
    psn_depth = depth;
    psn_solverDivide = solverDivide;
    psn_isoDivide = isoDivide;
    psn_samplesPerNode = samplesPerNode;
    psn_scale = scale;
    psn_confidence = confidence;
}

void MeshReconstruction::meshReconstruction(LogWindow *logWin)
{
    logWin->appendMessage("Started - reconstruct_mesh() with Poisson");

    pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2());
    pcl::PointCloud<PointType>::Ptr cloud (new
            pcl::PointCloud<PointType>);
    pcl::PolygonMesh triangles;

    pcl::PCDReader reader;
    reader.read (filePath, *cloud_blob);

    pcl::fromPCLPointCloud2 (*cloud_blob, *cloud);
    // the data should be available in cloud
    logWin->appendMessage("PointCloud loaded: " + QString::number(cloud->size())
                          + " data points\n");

    // Normal estimation
    pcl::NormalEstimation<PointType, Normal> normEst;
    pcl::PointCloud<Normal>::Ptr normals (new pcl::PointCloud<Normal>);

    // Create kdtree representation of cloud,
    // and pass it to the normal estimation object.
    pcl::search::KdTree<PointType>::Ptr tree (new
            pcl::search::KdTree<PointType>);
    tree->setInputCloud (cloud);
    normEst.setInputCloud (cloud);
    normEst.setSearchMethod (tree);
    // Use 20 neighbor points for estimating normal
    normEst.setKSearch (20);
    normEst.compute (*normals);
    // normals should not contain the point normals + surface
    // curvatures

    // Concatenate the XYZ and normal fields
    pcl::PointCloud<PointTypeN>::Ptr cloud_with_normals (new
            pcl::PointCloud<PointTypeN>);
    pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
    // cloud_with_normals = cloud + normals

    // Create search tree
    pcl::search::KdTree<PointTypeN>::Ptr tree2 (new
            pcl::search::KdTree<PointTypeN>);
    tree2->setInputCloud (cloud_with_normals);

    // Initialize objects
    // psn - for surface reconstruction algorithm
    // triangles - for storage of reconstructed triangles
    pcl::Poisson<PointTypeN> psn;
    //pcl::PolygonMesh triangles;

    psn.setInputCloud(cloud_with_normals);
    psn.setSearchMethod(tree2);
    psn.setDepth(psn_depth);
    psn.setSolverDivide(psn_solverDivide);
    psn.setIsoDivide(psn_isoDivide);
    psn.setSamplesPerNode(psn_samplesPerNode);
    psn.setScale(psn_scale);
    psn.setConfidence(psn_confidence);
    psn.reconstruct (triangles);
    //psn.setOutputPolygons(false);

    pcl::PCDWriter writer;
    pcl::PCLPointCloud2::Ptr cwn (new pcl::PCLPointCloud2());
    pcl::toPCLPointCloud2 (*cloud_with_normals, *cwn);

    logWin->appendMessage("Writing reconstructed mesh and cloud with normals");
    std::string str, str2;
    str.append(filePath).append("-mesh.vtk");
    pcl::io::saveVTKFile (str, triangles);
    str2.append(filePath).append("-cloud_with_normals.pcd");
    writer.write (str2, *cwn, Eigen::Vector4f::Zero(),
            Eigen::Quaternionf::Identity(), false);
    logWin->appendMessage("Finshed - reconstruct_mesh() with Poisson");

    logWin->appendMessage
            ("Using these parameters: \n"
            "Depth: " + QString::number(psn.getDepth()) + "\n"
            "Min Depth: " + QString::number(psn.getMinDepth()) + "\n"
            "Solver Divde: " + QString::number(psn.getSolverDivide()) + "\n"
            "Iso Divide: " + QString::number(psn.getIsoDivide()) + "\n"
            "Samples per Node: " + QString::number(psn.getSamplesPerNode()) +
            "\nScale: " + QString::number(psn.getScale()) + "\n"
            "Confidence: " + QString::number(psn.getConfidence()) + "\n"
            "Degree: " + QString::number(psn.getDegree()) + "\n"
            "Mainfold: " + QString::number(psn.getManifold()) + "\n"
            "Output Polygons: " + QString::number(psn.getOutputPolygons()) +
            "\n" );
}

void MeshReconstruction::showMesh(LogWindow *logWin)
{
    pcl::PolygonMesh mesh_of_triangles;
    pcl::io::loadPolygonFile(filePath, mesh_of_triangles);
    logWin->appendMessage ("Started - show_mesh() with PCLVisualizer\n");
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
    logWin->appendMessage("Finshed - show_mesh() with PCLVisualizer\n");
}

