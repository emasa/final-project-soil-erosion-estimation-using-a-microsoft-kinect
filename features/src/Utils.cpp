
#include <pcl/point_types.h>
#include <pcl/recognition/color_gradient_dot_modality.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "features/Utils.h"

template<> void 
displayKeypoints<pcl::PointXYZRGBA, pcl::PointWithScale>
	(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, 
     const pcl::PointCloud<pcl::PointWithScale>::Ptr &keypoints, 
     pcl::visualization::PCLVisualizer &viewer, 
     const pcl::PointRGB &color,
     const unsigned int size, 
     const std::string &cloud_id,
     const std::string &keypoints_id, 
     const int viewport);