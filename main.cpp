#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/impl/organized_fast_mesh.hpp>
#include <pcl/io/vtk_io.h>

typedef pcl::PointXYZ PointType;
typedef pcl::Normal Normal;
typedef pcl::PointNormal PointTypeN;
//typedef pcl::PointXYZRGB PointType;
//typedef pcl::PointXYZRGBNormal PointTypeN;

int main(int argc, char *argv[])
{

    // Load input file into a PointCloud<T> with an appropriate type
    pcl::PointCloud<PointType>::Ptr cloud (new pcl::PointCloud<PointType>);
    pcl::PCLPointCloud2 cloud_blob;
    //pcl::io::loadPCDFile ("pointcloud-ascii-downsampled-inliers.pcd", cloud_blob);
    pcl::io::loadPCDFile ("bun0.pcd", cloud_blob);
    pcl::fromPCLPointCloud2 (cloud_blob, *cloud);
    //* the data should be available in cloud
    std::cout << "bun0 loaded " << std::endl;
    std::cout << cloud->size() << std::endl;

    // Normal estimation*
    pcl::NormalEstimation<PointType, Normal> n;
    pcl::PointCloud<Normal>::Ptr normals (new pcl::PointCloud<Normal>);
    pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
    tree->setInputCloud (cloud);
    n.setInputCloud (cloud);
    n.setSearchMethod (tree);
    n.setKSearch (20);
    n.compute (*normals);
    //* normals should not contain the point normals + surface curvatures

    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<PointTypeN>::Ptr cloud_with_normals (new pcl::PointCloud<PointTypeN>);
    pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals

    // Create search tree*
    pcl::search::KdTree<PointTypeN>::Ptr tree2 (new pcl::search::KdTree<PointTypeN>);
    tree2->setInputCloud (cloud_with_normals);

    // Initialize objects
    // pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;
    pcl::OrganizedFastMesh<PointTypeN> ofm;
    pcl::PolygonMesh triangles;

    std::cout << cloud_with_normals->size() << std::endl;

    ofm.setSearchMethod(tree2);
    ofm.setInputCloud(cloud_with_normals);
    ofm.reconstruct (triangles);
    // Set the maximum distance between connected points (maximum edge length)
    // gp3.setSearchRadius (0.125);

    // // Set typical values for the parameters
    // gp3.setMu (2.5);
    // gp3.setMaximumNearestNeighbors (50);
    // gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    // gp3.setMinimumAngle(M_PI/18); // 10 degrees
    // gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    // gp3.setNormalConsistency(true);

    // // Get result
    // gp3.setInputCloud (cloud_with_normals);
    // gp3.setSearchMethod (tree2);
    // gp3.reconstruct (triangles);

    // // Additional vertex information
    // std::vector<int> parts = gp3.getPartIDs();
    // std::vector<int> states = gp3.getPointStates();

    pcl::io::saveVTKFile ("mesh-ofm.vtk", triangles);
    return 0;
}
