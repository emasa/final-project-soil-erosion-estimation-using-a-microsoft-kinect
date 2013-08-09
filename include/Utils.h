
#ifndef UTILS_H
#define UTILS_H

#include <pcl/common/transforms.h>
#include <pcl/recognition/color_gradient_dot_modality.h>
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/core/core.hpp>

namespace features
{

template<typename PointInT, typename KeypoinT>
void displayKeypoints(const typename pcl::PointCloud<PointInT>::Ptr &cloud, 
                      const typename pcl::PointCloud<KeypoinT>::Ptr &keypoints, 
                      pcl::visualization::PCLVisualizer &viewer, 
                      const pcl::PointRGB color = pcl::PointRGB(0, 255, 0),
                      const unsigned int size = 3, 
                      const std::string &cloud_id = "cloud",
                      const std::string &keypoints_id = "keypoints", 
                      const int viewport = 0)
{
    pcl::visualization::PointCloudColorHandlerRGBField<PointInT> cloud_handler(cloud);
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

template<typename PointInT, typename KeypointT>
void displayMatches(const typename pcl::PointCloud<PointInT>::Ptr &cloud_src, 
                    const typename pcl::PointCloud<KeypointT>::Ptr &keypoints_src, 
                    const typename pcl::PointCloud<PointInT>::Ptr &cloud_dest,
                    const typename pcl::PointCloud<KeypointT>::Ptr &keypoints_dest,
                    const pcl::Correspondences &matches,
                    pcl::visualization::PCLVisualizer &viewer,
                    const float tx = 0, // meters 
                    const float ty = 0,
                    const float tz = 0,
                    const unsigned int size = 3,
                    const std::string &cloud_src_id = "cloud_src_id",
                    const std::string &keypoints_src_id = "keypoints_src_id",                     
                    const std::string &cloud_dest_id = "cloud_dest_id",
                    const std::string &keypoints_dest_id = "keypoints_dest_id", 
                    const int viewport = 0)
{
  Eigen::Matrix<float, 3, 1> T;
  T(0, 0) = tx; T(1, 0) = ty; T(2, 0) = tz;

  auto R = Eigen::Quaternion<float>::Identity();

  typename pcl::PointCloud<PointInT>::Ptr tmp_cloud_dest(new pcl::PointCloud<PointInT>);
  typename pcl::PointCloud<KeypointT>::Ptr tmp_keypoints_dest(new pcl::PointCloud<KeypointT>);

  if (tx == 0.0f && ty == 0.0f && tz == 0.0f) 
  {
    tmp_cloud_dest = cloud_dest;
    tmp_keypoints_dest = keypoints_dest;
  } else {
    pcl::transformPointCloud<PointInT, float>(*cloud_dest, *tmp_cloud_dest, T, R);
    pcl::transformPointCloud<KeypointT, float>(*keypoints_dest, *tmp_keypoints_dest, T, R);
  }

  displayKeypoints<PointInT, KeypointT>(cloud_src, keypoints_src, 
                   viewer, pcl::PointRGB(0, 255, 0), 
                   size, cloud_src_id, keypoints_src_id, viewport);

  displayKeypoints<PointInT, KeypointT>(tmp_cloud_dest, tmp_keypoints_dest, 
                                        viewer, pcl::PointRGB(0, 0, 255), 
                   size, cloud_dest_id, keypoints_dest_id, viewport);

  viewer.addCorrespondences<KeypointT>(keypoints_src, tmp_keypoints_dest, matches);
}

}

#endif // namespace features
