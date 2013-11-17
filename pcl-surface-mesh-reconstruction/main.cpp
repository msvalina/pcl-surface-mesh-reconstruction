#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/poisson.h>
#include <pcl/io/vtk_io.h>

typedef pcl::Normal Normal;
//typedef pcl::PointXYZ PointType;
//typedef pcl::PointNormal PointTypeN;
typedef pcl::PointXYZRGB PointType;
typedef pcl::PointXYZRGBNormal PointTypeN;

int main(int argc, char *argv[])
{
    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
    pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

    // Fill in the cloud data
    pcl::PCDReader reader;
    if (argc < 1){
    reader.read ("pointcloud-ascii.pcd", *cloud);
    }
    else reader.read(argv[1], *cloud);

    std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
       << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;

    // Create the filtering object
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*cloud_filtered);

    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;

    pcl::PCDWriter writer;
    writer.write ("pointcloud-ascii-downsampled.pcd", *cloud_filtered,
         Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZRGB>);

    // Fill in the cloud data
    // Replace the path below with the path where you saved your file
    reader.read<pcl::PointXYZRGB> ("pointcloud-ascii-downsampled.pcd", *cloud2);

    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << *cloud2 << std::endl;

    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor2;
    sor2.setInputCloud (cloud2);
    sor2.setMeanK (50);
    sor2.setStddevMulThresh (1.0);
    sor2.filter (*cloud_filtered2);

    std::cerr << "Cloud after filtering: " << std::endl;
    std::cerr << *cloud_filtered2 << std::endl;

    writer.write<pcl::PointXYZRGB> ("pointcloud-ascii-downsampled-inliers.pcd", *cloud_filtered2, false);

    sor2.setNegative (true);
    sor2.filter (*cloud_filtered2);
    writer.write<pcl::PointXYZRGB> ("pointcloud-ascii-downsampled-outliers.pcd", *cloud_filtered2, false);

    // Load input file into a PointCloud<T> with an appropriate type
    pcl::PointCloud<PointType>::Ptr cloud3 (new pcl::PointCloud<PointType>);
    pcl::PCLPointCloud2 cloud_blob;
    pcl::io::loadPCDFile ("pointcloud-ascii-downsampled-inliers.pcd", cloud_blob);
    //pcl::io::loadPCDFile ("bun0.pcd", cloud_blob);
    pcl::fromPCLPointCloud2 (cloud_blob, *cloud3);
    //* the data should be available in cloud
    std::cout << "cloud loaded " << std::endl;
    std::cout << cloud3->size() << std::endl;

    // Normal estimation*
    pcl::NormalEstimation<PointType, Normal> normEst;
    pcl::PointCloud<Normal>::Ptr normals (new pcl::PointCloud<Normal>);
    pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
    tree->setInputCloud (cloud3);
    normEst.setInputCloud (cloud3);
    normEst.setSearchMethod (tree);
    normEst.setKSearch (20);
    normEst.compute (*normals);
    //* normals should not contain the point normals + surface curvatures

    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<PointTypeN>::Ptr cloud_with_normals (new pcl::PointCloud<PointTypeN>);
    pcl::concatenateFields (*cloud3, *normals, *cloud_with_normals);
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

    pcl::io::saveVTKFile ("mesh-dnevna.vtk", triangles);
    return 0;
}
