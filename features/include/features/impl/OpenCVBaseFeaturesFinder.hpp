
#ifndef OPENCV_BASE_FEATURES_FINDER_HPP
#define OPENCV_BASE_FEATURES_FINDER_HPP

#include <vector>
#include <stdexcept>

#include <pcl/exceptions.h>
#include <pcl/common/common.h>
#include <pcl/common/point_tests.h>
#include <pcl/conversions.h>
#include <pcl/search/organized.h>
#include <opencv2/features2d/features2d.hpp>

#include "features/OpenCVBaseFeaturesFinder.h"

// TODO: chequar que cloud, parametros de la camara y detectores y extractores
// de keypoints han sido seteados antes de usarlos

inline int closer(double value)
{
	int floor_value = static_cast<int>(value);
	return value - floor_value < 0.5 ? floor_value : floor_value + 1;
}

template <typename PointInT, typename KeypointT, typename DescriptorT> void
OpenCVBaseFeaturesFinder<PointInT, KeypointT, DescriptorT>::setInputCloud(const PointCloudInPtr &cloud)
{
	if (!cloud) throw std::invalid_argument("cloud is null");

	rgb_cloud_ = cloud;

	try { // convert to pcl image
		pcl::toPCLPointCloud2 (*rgb_cloud_, pcl_rgb_image_);
	} catch (std::runtime_error& e) {
		throw pcl::UnorganizedPointCloudException("Cloud needs to be organized");
	}
	// memory is managed by pcl_rgb_image_
	cv::Mat tmp_image(pcl_rgb_image_.height, pcl_rgb_image_.width, CV_8UC3, 
				      static_cast<uchar*>(pcl_rgb_image_.data.data()), 
				      pcl_rgb_image_.step);

	cv_rgb_image_ = tmp_image;

	resetCameraParameters();
}

template <typename PointInT, typename KeypointT, typename DescriptorT> void
OpenCVBaseFeaturesFinder<PointInT, KeypointT, DescriptorT>::resetCameraParameters()
{
	if (!rgb_cloud_) throw std::invalid_argument("Cloud is a null ptr");

	// compute camera matrix
	pcl::search::OrganizedNeighbor<PointInT> search;
	search.setInputCloud(rgb_cloud_);

	Eigen::Matrix3f C; // camera matrix
	search.computeCameraMatrix(C);
	fx_ = C(0, 0); fy_= C(1, 1); cx_ = C(0, 2); cy_ = C(1, 2);
}

template <typename PointInT, typename KeypointT, typename DescriptorT> void
OpenCVBaseFeaturesFinder<PointInT, KeypointT, DescriptorT>::computeKeypointsAndDescriptors(PointCloudKeypoint &keypoints, 
				  				   					      				  			   	   PointCloudDescriptor &descriptors)
{
	// TODO: improve
	PointCloudKeypoint tmp_keypoints;
	computeKeypoints(tmp_keypoints);
	computeDescriptors(tmp_keypoints, descriptors, keypoints);
}

template <typename PointInT, typename KeypointT, typename DescriptorT> void
OpenCVBaseFeaturesFinder<PointInT, KeypointT, DescriptorT>::computeKeypoints(PointCloudKeypoint &keypoints)
{
	keypoint_detector_->detect(cv_rgb_image_, cv_keypoints_);
	getKeypoints3D(cv_keypoints_, keypoints);
}

template <typename PointInT, typename KeypointT, typename DescriptorT> void
OpenCVBaseFeaturesFinder<PointInT, KeypointT, DescriptorT>::computeDescriptors(const PointCloudKeypoint &keypoints, 
 											  				  			   	   PointCloudDescriptor &descriptors, 
 											 				  			   	   PointCloudKeypoint &remaining_keypoints)
{	
	getKeypoints2D(keypoints, cv_keypoints_);

	// Keypoints for which a descriptor cannot be computed are removed
	// Sometimes new keypoints can be added	
	descriptor_extractor_->compute(cv_rgb_image_, cv_keypoints_, cv_descriptors_);
		
	getKeypoints3D(cv_keypoints_, remaining_keypoints);

	convertDescriptors(cv_descriptors_, descriptors);
}

template <typename PointInT, typename KeypointT, typename DescriptorT> void
OpenCVBaseFeaturesFinder<PointInT, KeypointT, DescriptorT>::getKeypoints3D(const std::vector<cv::KeyPoint> &cv_keypoints,
				  						    			  			   	   PointCloudKeypoint &keypoints)
{
	keypoints.clear(); 

	KeypointT keypoint3D;
	for (const auto &keypoint2D : cv_keypoints)
	{
		if ( !(0 <= keypoint2D.pt.x && keypoint2D.pt.x < rgb_cloud_->width && 
			   0 <= keypoint2D.pt.y && keypoint2D.pt.y < rgb_cloud_->height) ) 
			continue;

		const auto& point3D = rgb_cloud_->at(closer(keypoint2D.pt.x), 
											 closer(keypoint2D.pt.y));

		if (!pcl::isFinite(point3D)) continue;
	
		backProject(keypoint2D, keypoint3D, point3D.z);
		keypoints.push_back(keypoint3D);
	}
}

template <typename PointInT, typename KeypointT, typename DescriptorT> void
OpenCVBaseFeaturesFinder<PointInT, KeypointT, DescriptorT>::getKeypoints2D(const PointCloudKeypoint &keypoints,
														  				   std::vector<cv::KeyPoint> &cv_keypoints) 
{
	cv_keypoints.clear(); 
	
	cv::KeyPoint keypoint2D;
	for (const auto& keypoint3D : keypoints) 
	{
		if ( !pcl::isFinite(keypoint3D) ) continue; 
		
		project(keypoint3D, keypoint2D);			
	
		if ( !(0 <= keypoint2D.pt.x && keypoint2D.pt.x < cv_rgb_image_.cols && 
			   0 <= keypoint2D.pt.y && keypoint2D.pt.y < cv_rgb_image_.rows) ) 
			continue;

		cv_keypoints.push_back(keypoint2D);
	}
}


template <typename PointInT, typename KeypointT, typename DescriptorT> void
OpenCVBaseFeaturesFinder<PointInT, KeypointT, DescriptorT>::convertDescriptors(const cv::Mat &cv_descriptors, 
				  		 										  			   PointCloudDescriptor &descriptors) 
{
	auto descriptors_len = cv_descriptors.rows;
	auto descriptor_size = cv_descriptors.cols;

	descriptors.resize(descriptors_len);

	for (auto idx = 0; idx < descriptors_len; ++idx)
	{
		auto & dest = descriptors[idx].descriptor;
		dest.resize(descriptor_size);
		cv_descriptors.row(idx).copyTo(dest);
	}
}

#endif // OPENCV_BASE_FEATURES_FINDER_HPP