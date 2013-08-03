
#ifndef UTILS_H
#define UTILS_H

#include <pcl/common/common.h>
#include <pcl/recognition/color_gradient_dot_modality.h>
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/core/core.hpp>

#include "Features.h"

void rgbCloudToImage(const pcl::PointCloud<pcl::PointXYZRGBA>& cloud, cv::Mat& image);

namespace features
{

void displayKeypoints(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, 
                      const pcl::PointCloud<BaseKeypoint>::Ptr &keypoints, 
                      pcl::visualization::PCLVisualizer &viewer, 
                      pcl::PointRGB color = pcl::PointRGB(0, 255, 0),
                      unsigned int size = 3, 
                      const std::string &cloud_id = "cloud",
                      const std::string &keypoints_id = "keypoints", 
                      int viewport = 0);

}
#endif