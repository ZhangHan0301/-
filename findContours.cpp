#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>
#include <opencv2/opencv.hpp>

typedef pcl::PointXYZRGB PointRGB;
typedef pcl::PointCloud<PointRGB> PointCloudRGB;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);

void Findcountours(PointCloudT::Ptr cloud) {
  // 初始化最大和最小值
  float min_x = std::numeric_limits<float>::max();
  float max_x = -std::numeric_limits<float>::max();
  float min_y = std::numeric_limits<float>::max();
  float max_y = -std::numeric_limits<float>::max();
  float min_z = std::numeric_limits<float>::max();
  float max_z = -std::numeric_limits<float>::max();

  // 遍历点云数据，找到最大和最小值
  for (const auto& point : cloud->points) {
    if (point.x < min_x) min_x = point.x;
    if (point.x > max_x) max_x = point.x;

    if (point.y < min_y) min_y = point.y;
    if (point.y > max_y) max_y = point.y;

    if (point.z < min_z) min_z = point.z;
    if (point.z > max_z) max_z = point.z;
  }

  std::cout << "Min X: " << min_x << ", Max X: " << max_x << std::endl;
  std::cout << "Min Y: " << min_y << ", Max Y: " << max_y << std::endl;
  std::cout << "Min Z: " << min_z << ", Max Z: " << max_z << std::endl;

  // 创建灰度图
  cv::Mat grayscaleImage(480, 640, CV_8UC1, cv::Scalar(0));

  // 将点云垂直投影到图像上
  for (const auto& point : cloud->points) {
    // 映射点云坐标到图像范围
    int pixelX = static_cast<int>((point.x - min_x) / (max_x - min_x) *
                                  grayscaleImage.cols);
    int pixelY = static_cast<int>((point.y - min_y) / (max_y - min_y) *
                                  grayscaleImage.rows);

    // 在灰度图上设置像素值（这里设置为255，你可以根据需要调整）
    if (pixelX >= 0 && pixelX < grayscaleImage.cols && pixelY >= 0 &&
        pixelY < grayscaleImage.rows) {
      grayscaleImage.at<uchar>(pixelY, pixelX) = 255;
    }
  }

  cv::imshow("0 Original Image", grayscaleImage);
  // 闭运算使像素值连接起来
  // cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
  // cv::morphologyEx(grayscaleImage, grayscaleImage, cv::MORPH_CLOSE, kernel);
  cv::boxFilter(grayscaleImage, grayscaleImage, -1,
                cv::Size(3, 3),
                cv::Point2i(-1, -1), false);

  cv::imshow("1 boxFilter", grayscaleImage);

  // 使用Canny边缘检测
  cv::Mat edges;
  cv::Canny(grayscaleImage, edges, 50, 150);
  cv::imshow("3. Edges", edges);

  //   对边缘进行膨胀
  int dilationSize = 5;  // 膨胀的内核大小，可以根据实际情况调整
  cv::Mat dilatedEdges;
  cv::dilate(edges, dilatedEdges,
             cv::getStructuringElement(
                 cv::MORPH_RECT,
                 cv::Size(2 * dilationSize + 1, 2 * dilationSize + 1)));
  cv::imshow("4. dilatedEdges", dilatedEdges);

  //  寻找边缘轮廓
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(dilatedEdges, contours, hierarchy, cv::RETR_TREE,
                   cv::CHAIN_APPROX_SIMPLE);

  // 打印层级信息
  
  for (int i = 0; i < contours.size(); ++i) {
    std::cout << "Contour " << i << ": "
              << " Parent: " << hierarchy[i][3]  // 输出父轮廓的索引
              << " Next: " << hierarchy[i][0]  // 输出同级的下一个轮廓的索引
              << " Previous: " << hierarchy[i][1]  // 输出同级的前一个轮廓的索引
              << " Child: " << hierarchy[i][2]  // 输出子轮廓的索引
              << std::endl;
  }

//   // 定义颜色数组，每个层级对应一个颜色
//   std::vector<cv::Scalar> colors = {
//       cv::Scalar(255, 0, 0),  // 蓝色
//       cv::Scalar(0, 255, 0),  // 绿色
//       cv::Scalar(0, 0, 255),  // 红色
//                               // 可以添加更多的颜色
//   };

//   // 绘制轮廓并根据层级选择颜色
//   cv::Mat resultImage = cv::Mat::zeros(grayscaleImage.size(), CV_8UC3);
//   cv::cvtColor(grayscaleImage, resultImage, cv::COLOR_GRAY2BGR);
//   for (int i = 0; i < contours.size(); ++i) {
//     int level = 0;  // 默认为第一层级
//     int parentIdx = hierarchy[i][3];  // 获取父轮廓的索引
//     while (parentIdx != -1) {
//       // 循环向上追溯，找到最外层的父轮廓
//       level++;
//       parentIdx = hierarchy[parentIdx][3];
//     }
//     // cv::drawContours(resultImage, contours, i, colors[level], 2);
//     cv::drawContours(resultImage, contours, i, colors[level], 2, cv::LINE_8,
//                      hierarchy);
//   }

    // // 在原始图像上绘制轮廓，使用绿色标记
    cv::Mat resultImage = cv::Mat::zeros(grayscaleImage.size(), CV_8UC3);
    cv::cvtColor(grayscaleImage, resultImage, cv::COLOR_GRAY2BGR);
    cv::drawContours(resultImage, contours, -1, cv::Scalar(0, 255, 0), 2);
    cv::imshow("5. drawContours", resultImage);
    cv::imwrite("result_image_1.jpg", resultImage);

    // 进行轮廓近似
    std::vector<std::vector<cv::Point>> approxContours;
    for (const auto& contour : contours) {
      std::vector<cv::Point> approxCurve;
      cv::approxPolyDP(contour, approxCurve, 3.0, true);
      approxContours.push_back(approxCurve);
    }

    // 连接近似后的轮廓，使用红色线
  cv::Mat result_img = cv::Mat::zeros(grayscaleImage.size(), CV_8UC3);

  for (const auto& contour : approxContours) {
    cv::polylines(resultImage, contour, true, cv::Scalar(0, 0, 255), 2);
    cv::polylines(result_img, contour, true, cv::Scalar(0, 0, 255), 2);
    }
    cv::imshow("6.result img ", result_img);
    cv::imshow("7.resultImage", resultImage);

    cv::imwrite("result_image_2.jpg", resultImage);
    cv::waitKey(0);
  }

void Seg(PointCloudT::Ptr cloud ) 
{
  // 去除无效点
//   std::vector<int> indices;
//   pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

  pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
  // pcl::IndicesPtr indices(new std::vector<int>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filter_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  //  pass.setFilterLimits (point.z-0.8, point.z+1.5);// 0 1
  pass.setFilterLimits(0.0, 2.0);  // 0 1
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

  printf("num of clusters:%d\n", cluster_indices.size());
  PointCloudT::Ptr cluster(new PointCloudT);

  for (const pcl::PointIndices& indices : cluster_indices) {
    
    for (std::vector<int>::const_iterator it = indices.indices.begin();
         it != indices.indices.end(); ++it) {
      
      cluster->points.push_back(filter_cloud->points[*it]);
    }
  }

  Findcountours(cluster);
}



int main(int argc, char * argv[])
{
    //读取pcd文件
    std::string file_path;
    file_path = argv[1];
    PointCloudT::Ptr cloud(new PointCloudT);
    if (pcl::io::loadPCDFile<PointT>(file_path, *cloud) == -1)
    {
        PCL_ERROR("Couldn't read the file.\n");
        return -1;
    }

    Seg(cloud);
    // Findcountours(cloud);

    return 0;
}
