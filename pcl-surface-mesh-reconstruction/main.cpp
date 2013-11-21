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

void downsample (int argc, char* argv[]);
void remove_outliers (int argc, char* argv[]);
void reconstruct_mesh (int argc, char* argv[]);

int main (int argc, char *argv[])
{
    downsample (argc, argv);
    remove_outliers (argc, argv);
    reconstruct_mesh (argc, argv);

    return 0;
}

void downsample (int argc, char* argv[])
{
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

    std::cout << "PointCloud before filtering: " << cloud->width *
        cloud->height << " data points (" << pcl::getFieldsList (*cloud)
        << ")." << std::endl;

    // Create the filtering object
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*cloud_filtered);

    std::cout << "PointCloud after filtering: " << 
            cloud_filtered->width * cloud_filtered->height 
            << " data points (" << pcl::getFieldsList (*cloud_filtered)
            << ")." << std::endl;

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

}

void remove_outliers (int argc, char* argv[])
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (new
            pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered2 (new
            pcl::PointCloud<pcl::PointXYZRGB>);

    // Fill in the cloud data
    pcl::PCDReader reader;
    if (argc < 2){
        reader.read<pcl::PointXYZRGB> ("pointcloud-downsampled.pcd",
                *cloud2);
    }
    else {
        std::string str;
        str.append(argv[1]).append("-downsampled.pcd");
        reader.read (str, *cloud2);
    }


    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << *cloud2 << std::endl;

    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor2;
    sor2.setInputCloud (cloud2);
    sor2.setMeanK (50);
    sor2.setStddevMulThresh (1.0);
    sor2.filter (*cloud_filtered2);

    std::cout << "Cloud after filtering: " << std::endl;
    std::cout << *cloud_filtered2 << std::endl;

    pcl::PCDWriter writer;

    if (argc < 2){
        writer.write<pcl::PointXYZRGB>
            ("pointcloud-downsampled-inliers.pcd",
             *cloud_filtered2, false);

        sor2.setNegative (true);
        sor2.filter (*cloud_filtered2);
        writer.write<pcl::PointXYZRGB>
            ("pointcloud-downsampled-outliers.pcd",
             *cloud_filtered2, false);
    }
    else {
        std::string str1;
        str1.append(argv[1]).append("-downsampled.pcd-inliers.pcd");
        std::string str2;
        str2.append(argv[1]).append("-downsampled.pcd-outliers.pcd");
        writer.write<pcl::PointXYZRGB> (str1, *cloud_filtered2, false);

        sor2.setNegative (true);
        sor2.filter (*cloud_filtered2);
        writer.write<pcl::PointXYZRGB> (str2, *cloud_filtered2, false);
    }

}

void reconstruct_mesh (int argc, char* argv[])
{
    // Load input file into a PointCloud<T> with an appropriate type
    pcl::PointCloud<PointType>::Ptr cloud3 (new
            pcl::PointCloud<PointType>);
    pcl::PCLPointCloud2 cloud_blob;

    if (argc < 2) {
        pcl::io::loadPCDFile
            ("pointcloud-downsampled-inliers.pcd", cloud_blob);
    }
    else {
        std::string str;
        str.append(argv[1]).append("-downsampled.pcd-inliers.pcd");
        pcl::io::loadPCDFile (str, cloud_blob);
    }

    pcl::fromPCLPointCloud2 (cloud_blob, *cloud3);
    //* the data should be available in cloud
    std::cout << "cloud loaded " << std::endl;
    std::cout << cloud3->size() << std::endl;

    // Normal estimation*
    pcl::NormalEstimation<PointType, Normal> normEst;
    pcl::PointCloud<Normal>::Ptr normals (new pcl::PointCloud<Normal>);
    pcl::search::KdTree<PointType>::Ptr tree (new
            pcl::search::KdTree<PointType>);
    tree->setInputCloud (cloud3);
    normEst.setInputCloud (cloud3);
    normEst.setSearchMethod (tree);
    normEst.setKSearch (20);
    normEst.compute (*normals);
    //* normals should not contain the point normals + surface
    //curvatures

    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<PointTypeN>::Ptr cloud_with_normals (new
            pcl::PointCloud<PointTypeN>);
    pcl::concatenateFields (*cloud3, *normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals

    // Create search tree*
    pcl::search::KdTree<PointTypeN>::Ptr tree2 (new
            pcl::search::KdTree<PointTypeN>);
    tree2->setInputCloud (cloud_with_normals);

    // Initialize objects
    pcl::Poisson<PointTypeN> poissn;
    pcl::PolygonMesh triangles;

    std::cout << cloud_with_normals->size() << std::endl;

    poissn.setInputCloud(cloud_with_normals);
    poissn.setSearchMethod(tree2);
    poissn.reconstruct (triangles);
    poissn.setOutputPolygons(false);

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
}

