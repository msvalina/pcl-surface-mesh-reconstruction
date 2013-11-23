#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/poisson.h>
#include <pcl/io/vtk_io.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::Normal Normal;
//typedef pcl::PointXYZ PointType;
//typedef pcl::PointNormal PointTypeN;
typedef pcl::PointXYZRGB PointType;
typedef pcl::PointXYZRGBNormal PointTypeN;

void downsample (int, char*[]);
void remove_outliers (int, char* []);
void reconstruct_mesh (int, char* [], pcl::PolygonMesh&);
boost::shared_ptr<pcl::visualization::PCLVisualizer> simple_visualiser
(pcl::PolygonMesh);
void show_mesh (const pcl::PolygonMesh&);

int main (int argc, char *argv[])
{
    downsample (argc, argv);
    remove_outliers (argc, argv);
    pcl::PolygonMesh mesh_of_triangles;
    reconstruct_mesh (argc, argv, mesh_of_triangles);
    show_mesh (mesh_of_triangles);

    return 0;
}

/* Downsampling with VoxelGrid class.
 * Voxel grid is a set of tiny 3D boxes in space.
 * In each voxel (i.e. 3D box) all points present will be approximated
 * (i.e. downsampled) with their centeroid.
 */

void downsample (int argc, char* argv[])
{
    std::cout << "Started - downsample() with VoxelGrid" << std::endl;

    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2());

    // Fill in the cloud data
    pcl::PCDReader reader;
    if (argc < 2){
        reader.read ("pointcloud.pcd", *cloud);
    }
    else {
        reader.read (argv[1], *cloud);
    }

    std::cout << "PointCloud before filtering: "; 
    std::cout << cloud->width * cloud->height << " data points (" ;
    std::cout << pcl::getFieldsList (*cloud) << ")." << std::endl;

    // Create the filtering object
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloud);
    // voxel size to be 1cm^3
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*cloud_filtered);

    std::cout << "PointCloud after filtering: "; 
    std::cout << cloud_filtered->width * cloud_filtered->height;
    std::cout << " data points ("  << pcl::getFieldsList (*cloud) << ").\n";

    pcl::PCDWriter writer;
    if (argc < 2){
        writer.write ("pointcloud-downsampled.pcd",
                *cloud_filtered, Eigen::Vector4f::Zero(),
                Eigen::Quaternionf::Identity(), false);
    }
    else {
        std::string str;
        str.append(argv[1]).append("-downsampled.pcd");
        writer.write (str, *cloud_filtered, Eigen::Vector4f::Zero(),
                Eigen::Quaternionf::Identity(), false);
    }

    std::cout << "Finished - downsample() with VoxelGrid\n" << std::endl;

}

/* Removing noisy data (i.e. outliers) from measurements using statistical
 * analysis technique
 */

void remove_outliers (int argc, char* argv[])
{
    std::cout << "Started - remove_outliers() with " << 
        "StatisticalOutlierRemoval" << std::endl;

    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2());

    // Fill in the cloud data
    pcl::PCDReader reader;
    if (argc < 2){
        reader.read ("pointcloud-downsampled.pcd", *cloud);
    }
    else {
        std::string str;
        str.append(argv[1]).append("-downsampled.pcd");
        reader.read (str, *cloud);
    }

    std::cout << "PointCloud before filtering: "; 
    std::cout << cloud->width * cloud->height << " data points (" ;
    std::cout << pcl::getFieldsList (*cloud) << ")." << std::endl;

    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor2;
    sor2.setInputCloud (cloud);
    // Set number of neighbors to analyze
    sor2.setMeanK (50);
    sor2.setStddevMulThresh (1.0);
    sor2.filter (*cloud_filtered);

    std::cout << "PointCloud after filtering: "; 
    std::cout << cloud_filtered->width * cloud_filtered->height;
    std::cout << " data points ("  << pcl::getFieldsList (*cloud) << ").\n";

    pcl::PCDWriter writer;

    // Write both inliers and outliers 
    if (argc < 2){
        writer.write ("pointcloud-downsampled-inliers.pcd",
                *cloud_filtered, Eigen::Vector4f::Zero(),
                Eigen::Quaternionf::Identity(), false);

        sor2.setNegative (true);
        sor2.filter (*cloud_filtered);
        writer.write ("pointcloud-downsampled-outliers.pcd",
                *cloud_filtered, Eigen::Vector4f::Zero(),
                Eigen::Quaternionf::Identity(), false);
    }
    else {
        std::string str1;
        str1.append(argv[1]).append("-downsampled.pcd-inliers.pcd");
        std::string str2;
        str2.append(argv[1]).append("-downsampled.pcd-outliers.pcd");
        writer.write (str1, *cloud_filtered, Eigen::Vector4f::Zero(),
                Eigen::Quaternionf::Identity(), false);

        sor2.setNegative (true);
        sor2.filter (*cloud_filtered);
        writer.write (str2, *cloud_filtered, Eigen::Vector4f::Zero(),
                Eigen::Quaternionf::Identity(), false);
    }

    std::cout << "Finished - remove_outliers() with " << 
        "StatisticalOutlierRemoval\n" << std::endl;

}

/* Construct mesh of triangles from input pointcloud using Poisson's
 * surface reconstruction algorithm
 */

void reconstruct_mesh (int argc, char* argv[], pcl::PolygonMesh& triangles)
{
    std::cout << "Started - reconstruct_mesh() with " << 
        "Poisson" << std::endl;

    pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2());
    pcl::PointCloud<PointType>::Ptr cloud (new
            pcl::PointCloud<PointType>);

    pcl::PCDReader reader;
    if (argc < 2) {
        reader.read ("pointcloud-downsampled-inliers.pcd", *cloud_blob);
    }
    else {
        std::string str;
        str.append(argv[1]).append("-downsampled.pcd-inliers.pcd");
        reader.read (str, *cloud_blob);
    }

    pcl::fromPCLPointCloud2 (*cloud_blob, *cloud);
    // the data should be available in cloud
    std::cout << "PointCloud loaded: " << cloud->size() << " data points\n";

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
    psn.reconstruct (triangles);
    psn.setOutputPolygons(false);

    // Write reconstructed mesh
    if (argc < 2){
        pcl::io::saveVTKFile
            ("pointcloud-downsampled-outliers-mesh.vtk",
             triangles);
    }
    else {
        std::string str;
        str.append(argv[1]).append("-mesh.vtk");
        pcl::io::saveVTKFile (str, triangles);
    }
    std::cout << "Finshed - reconstruct_mesh() with " << 
        "Poisson" << std::endl;

}

/* Simple use of PCLVisualizer class (same class is used for pcl_viewer)
 * for displaying constructed mesh
 */

boost::shared_ptr<pcl::visualization::PCLVisualizer> simple_visualiser
(pcl::PolygonMesh mesh)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new
          pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPolygonMesh (mesh, "sample mesh");
    viewer->initCameraParameters (); 
    return (viewer); 
}

/* Function for creating viewer object and making sure it runs */

void show_mesh (const pcl::PolygonMesh& mesh_of_triangles)
{
    // Create viewer object and show mesh
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer = simple_visualiser (mesh_of_triangles);
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100); boost::this_thread::sleep
            (boost::posix_time::microseconds (100000));
    }

}
