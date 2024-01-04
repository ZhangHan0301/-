#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/filter.h> 
#include <pcl/common/common.h> 
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>

// #include <ros/ros.h>
// #include <tf/tf.h>
// #include <sensor_msgs/PointCloud2.h>

typedef pcl::PointXYZRGB PointRGB;
typedef pcl::PointCloud<PointRGB> PointCloudRGB;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);

// void LidarCB(const sensor_msgs::PointCloud2ConstPtr &input)
// {
//     pcl::fromROSMsg(*input, *cloud_in);

// }
void euclideanClusterSegmentation(PointCloudT::Ptr cloud, PointCloudRGB::Ptr& colored_clusters)
{
    // 去除无效点
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

    pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
    // pcl::IndicesPtr indices(new std::vector<int>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filter_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    //  pass.setFilterLimits (point.z-0.8, point.z+1.5);// 0 1
    pass.setFilterLimits(0.0, 2.0);// 0 1
    pass.filter(*cloud);

    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    //  pass.setFilterLimits (point.z-0.8, point.z+1.5);// 0 1
    pass.setFilterLimits(-2.5, 2.5);// 0 1
    pass.filter(*cloud);

    pass.setInputCloud(cloud);
    pass.setFilterFieldName("y");
    //  pass.setFilterLimits (point.z-0.8, point.z+1.5);// 0 1
    pass.setFilterLimits(-3, 1.0);// 0 1
    pass.filter(*filter_cloud);


    kdtree->setInputCloud(filter_cloud);


    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(0.12);
    ec.setMinClusterSize(200);
    ec.setMaxClusterSize(100000);
    ec.setSearchMethod(kdtree);
    ec.setInputCloud(filter_cloud);
    ec.extract(cluster_indices);

    // 存储所有簇点云的容器
    std::vector<PointCloudT::Ptr> clusters;
    printf("num of clusters:%d\n",cluster_indices.size());

    int cluster_number = 0;
    double sum_point_x,sum_point_y =0.0;
    double point_avge_x, point_avge_y =0.0;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("PointCloud Viewer"));
    std::vector<pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>> color_handlers;
    //viewer->addPointCloud(filter_cloud, "cloud");
    for (const pcl::PointIndices& indices : cluster_indices)
    {
        PointCloudT::Ptr cluster(new PointCloudT);
        for (std::vector<int>::const_iterator it = indices.indices.begin(); it != indices.indices.end(); ++it)
        {
            sum_point_x += filter_cloud->points[*it].x;
            sum_point_y += filter_cloud->points[*it].y;
            cluster->points.push_back(filter_cloud->points[*it]);
        }

        point_avge_x = sum_point_x/cluster->points.size();
        point_avge_y = sum_point_y/cluster->points.size();
        printf("point avger x:%f,point avger y:%f\n",point_avge_x,point_avge_y);
        

        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;

        clusters.push_back(cluster); // 将簇点云添加到容器
        

        std::string cloud_id = "cluster_" + std::to_string(cluster_number);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(cluster, rand() % 255, rand() % 255, rand() % 255);
        color_handlers.push_back(color_handler);
        viewer->addPointCloud(cluster, color_handlers[cluster_number], cloud_id);

        // 计算包围盒并绘制
        cluster_number++;

        Eigen::Vector4f min_pt, max_pt;
        pcl::getMinMax3D(*cluster, min_pt, max_pt);
        viewer->addCube(min_pt[0], max_pt[0], min_pt[1], max_pt[1], min_pt[2], max_pt[2], 1.0, 1.0, 1.0, cloud_id + "_bbox");
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, cloud_id + "_bbox");
    }
    viewer->spinOnce();

    // while (!viewer->wasStopped())
    // {
    //     viewer->spinOnce();
    // }

    // // 合并所有簇点云
    // colored_clusters = PointCloudRGB::Ptr(new PointCloudRGB);
    // for (std::size_t i = 0; i < clusters.size(); ++i)
    // {
    //     PointRGB color;
    //     color.r = static_cast<uint8_t>(rand() % 255);
    //     color.g = static_cast<uint8_t>(rand() % 255);
    //     color.b = static_cast<uint8_t>(rand() % 255);

    //     for (const PointT& pt : clusters[i]->points)
    //     {
    //         PointRGB colored_pt;
    //         colored_pt.x = pt.x;
    //         colored_pt.y = pt.y;
    //         colored_pt.z = pt.z;
    //         colored_pt.rgb = color.rgb;
    //         colored_clusters->points.push_back(colored_pt);
    //     }
    // }
}

int main(int argc, char * argv[])
{
    //初始化ros节点
    // ros::init(argc,argv,"seg_node")
    // ros::NodeHandle nh;

    // ros::Subscriber lidarSub = nh.subscribe(/rslidar_points, 1, LidarCB);
    
    std::string file_path;
    file_path = argv[1];
    PointCloudT::Ptr cloud(new PointCloudT);
    if (pcl::io::loadPCDFile<PointT>(file_path, *cloud) == -1)
    {
        PCL_ERROR("Couldn't read the file.\n");
        return -1;
    }

    // while(ros::ok()){
    //     ros::spinOnce();
    //     PointCloudRGB::Ptr colored_clusters;
    //     euclideanClusterSegmentation(cloud_in, colored_clusters);
    // }
    PointCloudRGB::Ptr colored_clusters;
    euclideanClusterSegmentation(cloud, colored_clusters);

    pcl::io::savePCDFileBinary("segmented_clusters.pcd", *colored_clusters);

    return 0;
}
