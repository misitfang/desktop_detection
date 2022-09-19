#include "omp.h"
#include <boost/thread/thread.hpp>
#include <chrono>
#include <iostream>
#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vector>

using namespace std;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointcloudT;
int main()
{
    auto startTime = std::chrono::steady_clock::now();

    PointcloudT::Ptr cloud(new PointcloudT);
    PointcloudT::Ptr cloud_out(new PointcloudT);
    /* 桌面pcd文件路径 */
    // pcl::io::loadPCDFile("/home/lu/XY/pcl/pcl_desktop_detection/data/cloud_cluster_after_Plane_segmentation0.pcd", *cloud);
    pcl::io::loadPCDFile("/home/lu/Download/pcl/pcl_desktop_detection/data/cloud_cluster_after_Plane_segmentation0.pcd", *cloud);
    *cloud_out = *cloud;
    cout << "加载点云" << cloud->points.size() << "个" << endl;

    //------------------------计算法向量---------------------------
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    n.setInputCloud(cloud);
    n.setNumberOfThreads(16);
    n.setSearchMethod(tree);
    // n.setKSearch(20);
    n.setRadiusSearch(0.03); //半径搜索0.03cm
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    n.compute(*normals);
    //-----------------------边界特征估计--------------------------
    pcl::BoundaryEstimation<PointT, pcl::Normal, pcl::Boundary> boundEst;
    boundEst.setInputCloud(cloud);
    boundEst.setInputNormals(normals);
    boundEst.setRadiusSearch(0.1); //设置边界估计所需要的半径
    boundEst.setAngleThreshold(M_PI / 4);
    boundEst.setSearchMethod(tree);
    pcl::PointCloud<pcl::Boundary> boundaries;
    boundEst.compute(boundaries);
    PointcloudT::Ptr cloud_boundary(new PointcloudT);
    PointcloudT::Ptr cloud_without_boundary(new PointcloudT);

    pcl::ExtractIndices<PointT> extract;
    for (size_t i = 0; i < cloud->points.size(); i++)
    {
        if (boundaries[i].boundary_point > 0)
        {
            inliers->indices.push_back(i);
        }
    }
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_boundary);
    extract.setNegative(true);
    extract.filter(*cloud_without_boundary);
    auto end_Time = std::chrono::steady_clock::now();
    auto boundary_estimation_Time = std::chrono::duration_cast<std::chrono::milliseconds>(end_Time - startTime);
    std::cout << "Cal took " << boundary_estimation_Time.count() << " milliseconds" << std::endl;
    cout << "finish boundary estimation" << endl;
    pcl::ExtractIndices<PointT> extract_r;
    pcl::PointIndices::Ptr inliers_r(new pcl::PointIndices());
    // PointcloudT::Ptr cloud_without_boundary_r(new PointcloudT);
    PointcloudT::Ptr cloud_without_boundary_r1(new PointcloudT);

    // kdtree search
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(cloud_without_boundary);
    std::vector<int> point_Index_Radius_Search;
    std::vector<float> point_Radius_Squared_Distance;
    float radius = 0.03;
    PointT search_point;
    int aaaa = 0;
    for (size_t i = 0; i < cloud_without_boundary->points.size(); i++)
    {
        PointcloudT::Ptr cloud_without_boundary_temp(new PointcloudT);
        search_point.x = cloud_without_boundary->points[i].x;
        search_point.y = cloud_without_boundary->points[i].y;
        search_point.z = cloud_without_boundary->points[i].z;
        if (kdtree.radiusSearch(search_point, radius, point_Index_Radius_Search, point_Radius_Squared_Distance) > 0 && point_Index_Radius_Search.size() > 9)
        {
            // for(size_t j=0;j<point_Index_Radius_Search.size();j++){
            // 	cloud_without_boundary_temp->points.push_back(cloud_without_boundary->points[point_Index_Radius_Search[j]]);

            if (aaaa < point_Index_Radius_Search.size())
            {
                aaaa = point_Index_Radius_Search.size();
                if (inliers_r->indices.size() > 0)
                {
                    inliers_r->indices.pop_back();
                }
                inliers_r->indices.push_back(i);
                cout << "point: " << i << "point_Index_Radius_Search.size(): " << point_Index_Radius_Search.size() << endl;
                cout << "The Coordinates of the output point " << i << " cloud are: " << cloud_without_boundary->points[i].x << "   "
                     << cloud_without_boundary->points[i].y << "   " << cloud_without_boundary->points[i].z << endl;
            }
            // point_Index_Radius_Search.size()<10
            // }
        }
        // *cloud_without_boundary_r1 += *cloud_without_boundary_temp;
    }
    extract_r.setInputCloud(cloud_without_boundary);
    extract_r.setIndices(inliers_r);
    extract_r.setNegative(false);
    extract_r.filter(*cloud_without_boundary_r1);

    //遍历搜索，计算与边缘点的距离
    /* 	for (size_t i=0;i<cloud_boundary->points.size(); i++)
        {
            PointcloudT::Ptr cloud_without_boundary_r(new PointcloudT);

            for (size_t j = 0; j < cloud_without_boundary->points.size(); j++)
            {
                if (sqrt(pow(cloud_boundary->points[i].x - cloud_without_boundary->points[j].x, 2) +
                         pow(cloud_boundary->points[i].y - cloud_without_boundary->points[j].y, 2) +
                         pow(cloud_boundary->points[i].z - cloud_without_boundary->points[j].z, 2)) < 0.015)
                {
                    inliers_r->indices.push_back(j);
                }
            }
            extract_r.setInputCloud(cloud_without_boundary);
            extract_r.setIndices(inliers_r);
            extract_r.setNegative(false);
            extract_r.filter(*cloud_without_boundary_r);
            *cloud_without_boundary_r1 += *cloud_without_boundary_r;

        } */

    for (size_t i = 0; i < cloud_without_boundary_r1->size(); i++)
    {
        cout << "The Coordinates of the output point " << i << " cloud are: " << cloud_without_boundary_r1->points[i].x << "   " << cloud_without_boundary_r1->points[i].y << "   " << cloud_without_boundary_r1->points[i].z << endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Cal took " << elapsedTime.count() << " milliseconds" << std::endl;
    cout << "The number of boundary  points cloud is :" << cloud_boundary->points.size() << endl;
    cout << "The number of final points cloud is :" << cloud_without_boundary_r1->points.size() << endl;

    //-------------------------可视化-----------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("ShowCloud"));
    int v1(0);
    viewer->setWindowName("boundary_place");
    viewer->createViewPort(0.0, 0.0, 1, 1.0, v1);
    viewer->setBackgroundColor(0, 0, 0, v1);
    viewer->addText("Raw point clouds", 20, 10, 10, 255, 0, 0, "v1_text", v1);

    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h(cloud_boundary, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_b(cloud_without_boundary_r1, 0, 255, 0);

    viewer->addPointCloud<pcl::PointXYZ>(cloud_boundary, cloud_tr_color_h, "sample cloud", v1);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_without_boundary_r1, cloud_tr_color_b, "cloud_boundary", v1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "sample cloud", v1);

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    return 0;
}
