
#ifndef OPENCV_BASE_FEATURES_FINDER_H
#define OPENCV_BASE_FEATURES_FINDER_H

#include <vector>
#include <memory>

#include <pcl/point_cloud.h>
#include <pcl/PCLImage.h>
#include <opencv2/features2d/features2d.hpp>

#include "features/FeaturesFinder.h"

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

	OpenCVBaseFeaturesFinder(const FeatureDetectorPtr &keypoint_detector = FeatureDetectorPtr(), 
						 	 const DescriptorExtractorPtr &descriptor_extractor = DescriptorExtractorPtr()) :	
		keypoint_detector_(keypoint_detector), 
		descriptor_extractor_(descriptor_extractor){}

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

	inline void 
	setKeypointDetector(const FeatureDetectorPtr &keypoint_detector)
	{
		keypoint_detector_ = keypoint_detector;	
	}

	inline FeatureDetectorPtr 
	getKeypointDetector()
	{
		return keypoint_detector_;	
	}

	inline void 
	setDescriptorExtractor(const DescriptorExtractorPtr &descriptor_extractor)
	{
		descriptor_extractor_ = descriptor_extractor;
	}

	inline void 
	getDescriptorExtractor()
	{
		return descriptor_extractor_;
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

	void 
	resetCameraParameters();

	PointCloudInPtr rgb_cloud_; 
	pcl::PCLImage pcl_rgb_image_;
	cv::Mat cv_rgb_image_;

	FeatureDetectorPtr keypoint_detector_;
	DescriptorExtractorPtr descriptor_extractor_;

	float fx_, fy_, cx_, cy_; // camera parameters

	std::vector<cv::KeyPoint> cv_keypoints_;
	cv::Mat cv_descriptors_;
};

#include <features/impl/OpenCVBaseFeaturesFinder.hpp>

#endif // OPENCV_BASE_FEATURES_FINDER_H