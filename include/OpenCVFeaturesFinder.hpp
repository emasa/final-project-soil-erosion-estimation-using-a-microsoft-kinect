
#ifndef OPENCV_FEATURES_FINDER_HPP
#define OPENCV_FEATURES_FINDER_HPP

#include <vector>

#include <pcl/common/common.h>
#include <pcl/conversions.h>
#include <opencv2/features2d/features2d.hpp>

#include <pcl/search/organized.h>

#include "Features.h"
#include "OpenCVFeaturesFinder.h"
#include "Utils.h"

// TODO: chequar que cloud, parametros de la camara y detectores y extractores
// de keypoints han sido seteados antes de usarlos

namespace features 
{

// TODO: mover definiciones a otro archivo

template<typename PointT>
inline bool isGoodPoint(const PointT& pt)
{
	return pcl_isfinite(pt.x) && pcl_isfinite(pt.y) && pcl_isfinite(pt.z);
}

inline uint closer(double value)
{
	double floor_value = floor(value);
	return value - floor_value < 0.5 ? floor_value : floor_value + 1;
}

template<typename KeypointType3D>
inline void project(const KeypointType3D &keypoint3D, cv::KeyPoint &keypoint2D, 
					double fx, double fy, double cx, double cy)
{
	keypoint2D.pt.x = fx * keypoint3D.x / keypoint3D.z + cx;
	keypoint2D.pt.y = fy * keypoint3D.y / keypoint3D.z + cy;
	
	keypoint2D.size     = keypoint3D.size;
	keypoint2D.angle    = keypoint3D.angle;
	keypoint2D.response = keypoint3D.response;
	keypoint2D.octave   = keypoint3D.octave;
}

template<typename KeypointType3D>
inline void backproject(const cv::KeyPoint &keypoint2D, KeypointType3D &keypoint3D, 
						double z, double fx, double fy, double cx, double cy)
{
	keypoint3D.x = (keypoint2D.pt.x - cx) * z / fx;
	keypoint3D.y = (keypoint2D.pt.y - cy) * z / fy;
	keypoint3D.z = z;

	keypoint3D.size     = keypoint2D.size;
	keypoint3D.angle    = keypoint2D.angle;
	keypoint3D.response = keypoint2D.response;
	keypoint3D.octave   = keypoint2D.octave;
}

template <typename PointInT, typename KeypointT> void
OpenCVFeaturesFinder<PointInT, KeypointT>::setInputCloud(const PointCloudInPtr &cloud)
{
	rgb_cloud_ = cloud;
	// TODO: if cloud isn't organized we have to manage a exception
	toPCLPointCloud2 (*rgb_cloud_, pcl_rgb_image_); // convert to pcl image

	// not memory allocation, memory is managed by pcl_rgb_image_
	cv::Mat tmp_image(pcl_rgb_image_.height, pcl_rgb_image_.width, CV_8UC3, 
				      static_cast<uchar*>(pcl_rgb_image_.data.data()), 
				      pcl_rgb_image_.step);

	cv_rgb_image_ = tmp_image;

	pcl::search::OrganizedNeighbor<PointInT> search;
	search.setInputCloud(rgb_cloud_);

	Eigen::Matrix3f C; // camera matrix
	search.computeCameraMatrix(C);
	fx_ = C(0, 0); fy_= C(1, 1); cx_ = C(0, 2); cy_ = C(1, 2);	
}

template <typename PointInT, typename KeypointT> void
OpenCVFeaturesFinder<PointInT, KeypointT>::computeKeypointsAndDescriptors(PointCloudKeypoint &keypoints, 
				  				   					      				  Descriptors &descriptors)
{
	std::vector<cv::KeyPoint> cv_keypoints;
	keypointDetector_->detect(cv_rgb_image_, cv_keypoints);
	computeDescriptors(cv_keypoints, descriptors, keypoints);
}

template <typename PointInT, typename KeypointT> void
OpenCVFeaturesFinder<PointInT, KeypointT>::computeKeypoints(PointCloudKeypoint &keypoints)
{
	std::vector<cv::KeyPoint> cv_keypoints;
	keypointDetector_->detect(cv_rgb_image_, cv_keypoints);

	std::vector<uint> indexes;
	convertKeypoints(cv_keypoints, keypoints, indexes, false);
}

template <typename PointInT, typename KeypointT> void
OpenCVFeaturesFinder<PointInT, KeypointT>::computeDescriptors(const PointCloudKeypoint &keypoints, 
 											  				  Descriptors &descriptors, 
 											 				  PointCloudKeypoint &remaining_keypoints)
{	
	std::vector<cv::KeyPoint> cv_keypoints;
	std::vector<uint> indexes;
	convertKeypoints(keypoints, cv_keypoints, indexes, false);	
	
	computeDescriptors(cv_keypoints, descriptors, remaining_keypoints);
}

template <typename PointInT, typename KeypointT> void
OpenCVFeaturesFinder<PointInT, KeypointT>::computeDescriptors(std::vector<cv::KeyPoint> &cv_keypoints, 
 											  				  Descriptors &descriptors, 
 											  				  PointCloudKeypoint &remaining_keypoints)
{	
	// Keypoints for which a descriptor cannot be computed are removed
	// Sometimes new keypoints can be added	
	Descriptors tmp_descriptors;
	descriptorExtractor_->compute(cv_rgb_image_, cv_keypoints, tmp_descriptors);
	
	std::vector<uint> indexes;
	convertKeypoints(cv_keypoints, remaining_keypoints, indexes);

	// TODO: refactorizar
	uint descriptors_size = tmp_descriptors.rows;
	if (descriptors_size == indexes.size())
	{
		descriptors = tmp_descriptors;
	} else 
	{   // filter descriptors
		descriptors.create(indexes.size(), tmp_descriptors.cols, tmp_descriptors.type());
		for (uint idx = 0; idx < indexes.size(); ++idx)
		{
			tmp_descriptors.row(indexes[idx]).copyTo(descriptors.row(idx));
		}
	}
}

template <typename PointInT, typename KeypointT> void
OpenCVFeaturesFinder<PointInT, KeypointT>::convertKeypoints(const std::vector<cv::KeyPoint> &cv_keypoints,
				  						    				PointCloudKeypoint &keypoints,
				  						    				std::vector<uint> &indexes,
				  						    				bool save_indexes)
{
	keypoints.clear(); 
	indexes.clear();

	KeypointT keypoint3D;
	for (uint idx = 0; idx < cv_keypoints.size(); ++idx)
	{
		const auto &keypoint2D = cv_keypoints[idx];	
		// filter cloud outliers
		if (!(0 < keypoint2D.pt.x && keypoint2D.pt.x < rgb_cloud_->width && 
			  0 <= keypoint2D.pt.y && keypoint2D.pt.y < rgb_cloud_->height)) 
			continue;

		const auto& point3D = rgb_cloud_->at(closer(keypoint2D.pt.x), 
											 closer(keypoint2D.pt.y));

		if (!isGoodPoint(point3D)) continue; 

		// uses keypoint2D.x, keypoint2D.y for keep projection invariant between 			
		// keypoint3D and (keypoint2D, point3D.z)		
		backproject(keypoint2D, keypoint3D, point3D.z, fx_, fy_, cx_, cy_);
		keypoints.push_back(keypoint3D);
				
		if (save_indexes)
			indexes.push_back(idx);
	}
}

template <typename PointInT, typename KeypointT> void
OpenCVFeaturesFinder<PointInT, KeypointT>::convertKeypoints(const PointCloudKeypoint &keypoints,
															std::vector<cv::KeyPoint> &cv_keypoints,
															std::vector<uint> &indexes,
															bool save_indexes)
{
	cv_keypoints.clear(); 
	
	cv::KeyPoint keypoint2D;
	for (uint idx = 0; idx < keypoints.size(); ++idx) 
	{
		const auto& keypoint3D = keypoints[idx];	

		if (!isGoodPoint(keypoint3D)) continue;
		
		project(keypoint3D, keypoint2D, fx_, fy_, cx_, cy_);			
		cv_keypoints.push_back(keypoint2D);

		if (save_indexes)
			indexes.push_back(idx);
	}
}

}// namespace features

#define PCL_INSTANTIATE_OpenCVFeaturesFinder(T,KT) template class PCL_EXPORTS features::OpenCVFeaturesFinder<T,KT>;

#endif // OPENCV_FEATURES_FINDER_HPP