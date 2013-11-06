#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <pcl/io/vtk_io.h>

//typedef pcl::PointXYZ PointType;
typedef pcl::Normal Normal;
//typedef pcl::PointNormal PointTypeN;
typedef pcl::PointXYZRGB PointType;
typedef pcl::PointXYZRGBNormal PointTypeN;

int main(int argc, char *argv[])
{

    // Load input file into a PointCloud<T> with an appropriate type
    pcl::PointCloud<PointType>::Ptr cloud (new pcl::PointCloud<PointType>);
    pcl::PCLPointCloud2 cloud_blob;
    pcl::io::loadPCDFile ("pointcloud-ascii-downsampled-inliers.pcd", cloud_blob);
    //pcl::io::loadPCDFile ("bun0.pcd", cloud_blob);
    pcl::fromPCLPointCloud2 (cloud_blob, *cloud);
    //* the data should be available in cloud
    std::cout << "cloud loaded " << std::endl;
    std::cout << cloud->size() << std::endl;

    // Normal estimation*
    pcl::NormalEstimation<PointType, Normal> normEst;
    pcl::PointCloud<Normal>::Ptr normals (new pcl::PointCloud<Normal>);
    pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
    tree->setInputCloud (cloud);
    normEst.setInputCloud (cloud);
    normEst.setSearchMethod (tree);
    normEst.setKSearch (20);
    normEst.compute (*normals);
    //* normals should not contain the point normals + surface curvatures

    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<PointTypeN>::Ptr cloud_with_normals (new pcl::PointCloud<PointTypeN>);
    pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals

    // Create search tree*
    pcl::search::KdTree<PointTypeN>::Ptr tree2 (new pcl::search::KdTree<PointTypeN>);
    tree2->setInputCloud (cloud_with_normals);

    // Initialize objects
    pcl::Poisson<PointTypeN> poissn;
    pcl::PolygonMesh triangles;

    std::cout << cloud_with_normals->size() << std::endl;

    poissn.setInputCloud(cloud_with_normals);
    poissn.setSearchMethod(tree2);
    poissn.reconstruct (triangles);
    poissn.setOutputPolygons(false);

    pcl::io::saveVTKFile ("mesh-poissn.vtk", triangles);
    return 0;
}
