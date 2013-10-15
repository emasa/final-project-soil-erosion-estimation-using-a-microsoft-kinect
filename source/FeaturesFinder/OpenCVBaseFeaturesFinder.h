
#ifndef OPENCV_BASE_FEATURES_FINDER_H
#define OPENCV_BASE_FEATURES_FINDER_H

#include <vector>
#include <memory>

#include <pcl/point_cloud.h>
#include <pcl/PCLImage.h>
#include <opencv2/features2d/features2d.hpp>

#include "FeaturesFinder/FeaturesFinder.h"

template<typename PointInT, typename KeypointT, typename DescriptorT>
class OpenCVBaseFeaturesFinder : public FeaturesFinder<PointInT, KeypointT, DescriptorT>
{
public:
	typedef std::shared_ptr<OpenCVBaseFeaturesFinder> Ptr;
	typedef std::shared_ptr<const OpenCVBaseFeaturesFinder> ConstPtr;

	typedef typename FeaturesFinder<PointInT, KeypointT, DescriptorT>::PointCloudIn PointCloudIn;
	typedef typename FeaturesFinder<PointInT, KeypointT, DescriptorT>::PointCloudInPtr PointCloudInPtr;
	typedef typename FeaturesFinder<PointInT, KeypointT, DescriptorT>::PointCloudInConstPtr PointCloudInConstPtr;

	typedef typename FeaturesFinder<PointInT, KeypointT, DescriptorT>::PointCloudKeypoint PointCloudKeypoint;	
	typedef typename FeaturesFinder<PointInT, KeypointT, DescriptorT>::PointCloudDescriptor PointCloudDescriptor;

	typedef cv::FeatureDetector FeatureDetector;
	typedef cv::Ptr<FeatureDetector> FeatureDetectorPtr;

	typedef cv::DescriptorExtractor DescriptorExtractor;
	typedef cv::Ptr<DescriptorExtractor> DescriptorExtractorPtr;

	OpenCVBaseFeaturesFinder(bool estimate_camera_parameters=false)
	: rgb_cloud_()
	, pcl_rgb_image_()
	, cv_rgb_image_()
	, keypoint_detector_() 
	, descriptor_extractor_()
	, fx_()
	, fy_()
	, cx_()
	, cy_()
	, estimate_camera_parameters_(estimate_camera_parameters)
	, cv_keypoints_()
	, cv_descriptors_()
	{}

	virtual ~OpenCVBaseFeaturesFinder() {}

	void 
	computeKeypointsAndDescriptors(PointCloudKeypoint &keypoints,
								   PointCloudDescriptor &descriptors);	

	void 
	computeKeypoints(PointCloudKeypoint &keypoints);

	void
	computeDescriptors(const PointCloudKeypoint &keypoints, 
 					   PointCloudDescriptor &descriptors, 
 					   PointCloudKeypoint &remaining_keypoints);

	void 
	setInputCloud(const PointCloudInPtr &cloud);

	void 
	setKeypointDetector(const FeatureDetectorPtr &keypoint_detector)
	{
		keypoint_detector_ = keypoint_detector;	
	}

	FeatureDetectorPtr 
	getKeypointDetector()
	{
		return keypoint_detector_;	
	}

	void 
	setDescriptorExtractor(const DescriptorExtractorPtr &descriptor_extractor)
	{
		descriptor_extractor_ = descriptor_extractor;
	}

	void 
	getDescriptorExtractor()
	{
		return descriptor_extractor_;
	}

	void
	setCameraParameters(float fx, float fy, float cx, float cy)
	{
		fx_ = fx; fy_ = fy; cx_ = cx; cy_ = cy;
	};

	void
	getCameraParameters(float &fx, float &fy, float &cx, float &cy)
	{
		fx = fx_; fy = fy_; cx = cx_; cy = cy_;
	};

	void 
	estimateCameraParameters()
	{
		estimate_camera_parameters_ = true;
	}
protected:

	virtual void 
	convertDescriptors(const cv::Mat &cv_descriptors, 
					   PointCloudDescriptor &descriptors) = 0;
	void 
	getKeypoints3D(const std::vector<cv::KeyPoint> &cv_keypoints,
					 PointCloudKeypoint &keypoints);

	void 
	getKeypoints2D(const PointCloudKeypoint &keypoints,
				   std::vector<cv::KeyPoint> &cv_keypoints);

	inline void 
	project(const KeypointT &keypoint3D, cv::KeyPoint &keypoint2D)
	{
		keypoint2D.pt.x = fx_ * keypoint3D.x / keypoint3D.z + cx_;
		keypoint2D.pt.y = fy_ * keypoint3D.y / keypoint3D.z + cy_;
		
		keypoint2D.size     = keypoint3D.size;
		keypoint2D.angle    = keypoint3D.angle;
		keypoint2D.response = keypoint3D.response;
		keypoint2D.octave   = keypoint3D.octave;
	}

	inline void 
	backProject(const cv::KeyPoint &keypoint2D, KeypointT &keypoint3D, double z)
	{
		keypoint3D.x = (keypoint2D.pt.x - cx_) * z / fx_;
		keypoint3D.y = (keypoint2D.pt.y - cy_) * z / fy_;
		keypoint3D.z = z;

		keypoint3D.size     = keypoint2D.size;
		keypoint3D.angle    = keypoint2D.angle;
		keypoint3D.response = keypoint2D.response;
		keypoint3D.octave   = keypoint2D.octave;
	}

	bool
	computeCameraParameters(const PointCloudInPtr &cloud);

	PointCloudInPtr rgb_cloud_; 
	pcl::PCLImage pcl_rgb_image_;
	cv::Mat cv_rgb_image_;

	FeatureDetectorPtr keypoint_detector_;
	DescriptorExtractorPtr descriptor_extractor_;

	float fx_, fy_, cx_, cy_; // camera parameters
	bool estimate_camera_parameters_;

	std::vector<cv::KeyPoint> cv_keypoints_;
	cv::Mat cv_descriptors_;
};

#include "FeaturesFinder/OpenCVBaseFeaturesFinder.hpp"

#endif // OPENCV_BASE_FEATURES_FINDER_H