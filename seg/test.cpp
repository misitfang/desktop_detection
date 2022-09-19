#include <iostream>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <vector>

#include <pcl/common/transforms.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointcloudT;

int main(int argc, char **argv)
{
    PointcloudT::Ptr cloud_in(new PointcloudT);
    if (pcl::io::loadPCDFile<PointT>("/home/flu/XY/pcl/pcl_desktop_detection/seg/no_area_place.pcd", *cloud_in) == -1) {
        std::cout << "cloud_in reading failed." << std::endl;
        return (-1);
    }
    std::cout << "cloud_in has " << cloud_in->points.size() << " points." << endl;
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud_in);
    vg.setLeafSize(0.01, 0.01, 0.01);
    vg.filter(*cloud_in);
    std::cout << "cloud_in after filter has " << cloud_in->points.size() << " points." << endl;

    //---------------旋转点云---------------------------------------------
    float theta = 5 * M_PI / 6;
    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    transform_2.translation() << 0.0, 0.0, 0.0;                             // 三个数分别对应X轴、Y轴、Z轴方向上的平移
    transform_2.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitX())); // UnitY(),绕Y轴；UnitZ(),绕Z轴.
    std::cout << transform_2.matrix() << std::endl;                         //
    PointcloudT::Ptr transformed_cloud(new PointcloudT);
    pcl::transformPointCloud(*cloud_in, *transformed_cloud, transform_2);
    //---------------------------------------------

    pcl::search::Search<PointT>::Ptr tree = boost::shared_ptr<pcl::search::Search<PointT>>(new pcl::search::KdTree<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<PointT, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(transformed_cloud);
    normal_estimator.setKSearch(50);
    normal_estimator.compute(*normals);

    pcl::IndicesPtr indices(new std::vector<int>);
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(transformed_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 1.0);
    pass.filter(*indices);

    pcl::RegionGrowing<PointT, pcl::Normal> reg;
    reg.setMinClusterSize(100);
    reg.setMaxClusterSize(100000);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(30);
    reg.setInputCloud(transformed_cloud);
    // reg.setIndices (indices);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(4.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold(1);

    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);

    std::vector<PointcloudT::Ptr> clusters___;
    // pcl::visualization::PCLVisualizer vis;
    for (size_t i = 0; i < clusters.size(); ++i) {
        std::cout << "Number of clusters is equal to " << clusters.size() << std::endl;
        std::cout << "First cluster has " << clusters[i].indices.size() << " points." << endl;
        // Extract the i_th cluster into a new cloud_in
        PointcloudT::Ptr cluster_i(new PointcloudT);
        pcl::copyPointCloud(*cloud_in, clusters[i], *cluster_i);
        clusters___.push_back(cluster_i);
        std::cout << "cloud_bvefore seg size is :" << cluster_i->points.size() << endl;
        std::stringstream str_clusters;
        str_clusters << "cloud_cluster_before_Plane_segmentation" << i << ".pcd";
        pcl::io::savePCDFile(str_clusters.str(), *clusters___[i]);
        std::cout << "cloud_bvefore seg size is :" << clusters___[i]->points.size() << endl;
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        pcl::SACSegmentation<PointT> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(1000);
        seg.setDistanceThreshold(0.01);

        seg.setInputCloud(cluster_i);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0) {
            PCL_ERROR("Could not estimate a planar model for the given dataset.");
            return (-1);
        }
        pcl::ExtractIndices<PointT> extract;
        int j = 0, nr_points = (int)cluster_i->points.size();
        // While 30% of the original cloud is still there
        while (cluster_i->points.size() > 0.3 * nr_points) {
            // Segment the largest planar component from the remaining cloud
            seg.setInputCloud(cluster_i);
            seg.segment(*inliers, *coefficients);
            if (inliers->indices.size() == 0) {
                std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
                break;
            }
            PointcloudT::Ptr cloud_p(new PointcloudT);

            // Extract the inliers
            extract.setInputCloud(cluster_i);
            extract.setIndices(inliers);
            extract.setNegative(false);
            extract.filter(*cloud_p);

            PointcloudT::Ptr cloud_f(new PointcloudT);

            // Create the filtering object
            extract.setNegative(true);
            extract.filter(*cloud_f);
            cluster_i.swap(cloud_f);
            j++;
            std::stringstream ss;
            ss << "cloud_cluster_after_Plane_segmentation" << i << ".pcd";
            pcl::io::savePCDFile(ss.str(), *cloud_p);
            std::cout << "cloud_after seg size is :" << cloud_p->points.size() << endl;
            std::cout << "clusters " << i << " saved" << std::endl;
        }

        // // Create a random color
        // pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> random_color(cluster_i);

        // // Create a unique identifier
        // std::stringstream cluster_id("cluster");
        // cluster_id << i;

        // Add the i_th cluster to the visualizer with a random color and a unique identifier
        // vis.addPointCloud<pcl::PointXYZ>(cluster_i, random_color, cluster_id.str());
    }
    // vis.addPointCloud(cloud_in);
    // vis.spin();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
    pcl::visualization::CloudViewer viewer("Cluster viewer");

    viewer.showCloud(colored_cloud);
    while (!viewer.wasStopped()) {
    }

    return (0);
}