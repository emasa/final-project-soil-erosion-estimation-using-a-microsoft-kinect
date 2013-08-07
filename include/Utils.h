
#ifndef UTILS_H
#define UTILS_H

#include <pcl/common/common.h>
#include <pcl/recognition/color_gradient_dot_modality.h>
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/core/core.hpp>

namespace features
{

template<typename KeypoinT>
void displayKeypoints(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, 
                      const typename pcl::PointCloud<KeypoinT>::Ptr &keypoints, 
                      pcl::visualization::PCLVisualizer &viewer, 
                      pcl::PointRGB color = pcl::PointRGB(0, 255, 0),
                      unsigned int size = 3, 
                      const std::string &cloud_id = "cloud",
                      const std::string &keypoints_id = "keypoints", 
                      int viewport = 0)
{
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> cloud_handler(cloud);

    viewer.addPointCloud(cloud, cloud_handler, cloud_id, viewport);

    pcl::visualization::PointCloudColorHandlerCustom<KeypoinT> 
    	keypoints_handler (keypoints, color.r, color.g, color.b);
   
    viewer.addPointCloud(keypoints, keypoints_handler, keypoints_id, viewport);
    
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 
    										 size, keypoints_id, viewport);

    // set the same pose (a bug in PCLVisualizer flip the keypoints 180 degrees)
    auto pose = viewer.getViewerPose(viewport);
    viewer.updatePointCloudPose(cloud_id, pose);
    viewer.updatePointCloudPose(keypoints_id, pose);
}

}
#endif // namespace features
