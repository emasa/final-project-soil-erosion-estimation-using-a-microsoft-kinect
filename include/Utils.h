
#ifndef UTILS_H
#define UTILS_H

#include <pcl/common/common.h>
#include <opencv2/core/core.hpp>

void rgbCloudToImage(const pcl::PointCloud<pcl::PointXYZRGBA>& cloud, cv::Mat& image)
{
    if (!cloud.isOrganized()) 
        throw pcl::UnorganizedPointCloudException("require Organized cloud to be converted to image");
    
    image.create(cloud.height, cloud.width, CV_8UC3);

    if (!cloud.empty()) {
        for (int h=0; h<image.rows; ++h) {
            for (int w=0; w<image.cols; ++w) {
                const auto& point = cloud.at(w, h);

                image.at<cv::Vec3b>(h, w)[0] = point.b;
                image.at<cv::Vec3b>(h, w)[1] = point.g;
                image.at<cv::Vec3b>(h, w)[2] = point.r;
            }
        }
    }
}

#endif