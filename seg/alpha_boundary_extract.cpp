#include <pcl/io/pcd_io.h>                    
#include <pcl/point_types.h>                   
#include <pcl/surface/concave_hull.h>          
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/time.h> 

using namespace std;

int
main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>),
        cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    //-----------------加载点云----------------------
    pcl::PCDReader reader;
    reader.read("/home/lu/Download/pcl/pcl_desktop_detection/data/cloud_cluster_after_Plane_segmentation0.pcd", *cloud);
    // cout<<"argv: "<<argv[1]<<endl;
    pcl::console::TicToc time;
    time.tic();
    // double alpha = argv[1];
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConcaveHull<pcl::PointXYZ> chull;
    chull.setInputCloud(cloud); // 输入点云为投影后的点云
    chull.setAlpha(std::stod(argv[1]));        // 设置alpha值为0.1
    chull.reconstruct(*cloud_hull);      

    cout << "提取边界点个数为: " << cloud_hull->points.size() << endl;
    cout << "提取边界点用时:" << time.toc() / 1000 << " 秒" << endl;
    pcl::PCDWriter writer;
    // writer.write("/home/tianbot/pcl_desktop_detection/data/3-1/alpha_boundary.pcd", *cloud_hull, false);
    //-----------------结果显示---------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("显示点云"));

    int v1(0), v2(0);
    viewer->setWindowName("alpha_shapes");
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud(cloud, "cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");

    viewer->addPointCloud(cloud_hull, "cloud_boundary");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cloud_boundary");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_boundary");

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(1000);
    }

    return 0;
}


