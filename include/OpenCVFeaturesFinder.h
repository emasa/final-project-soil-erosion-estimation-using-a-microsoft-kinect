
#ifndef OPENCV_FEATURES_FINDER_H
#define OPENCV_FEATURES_FINDER_H

#include <vector>
#include <memory>

#include <pcl/common/common.h>
#include <pcl/PCLImage.h>
#include <opencv2/features2d/features2d.hpp>

#include "Features.h"
#include "FeaturesFinder.h"

namespace features 
{
template<typename PointInT, typename KeypointT>
class OpenCVFeaturesFinder : public FeaturesFinder<PointInT, KeypointT>
{
public:
	typedef std::shared_ptr<OpenCVFeaturesFinder> Ptr;
	typedef std::shared_ptr<const OpenCVFeaturesFinder> ConstPtr;

	typedef typename FeaturesFinder<PointInT, KeypointT>::PointCloudIn PointCloudIn;
	typedef typename FeaturesFinder<PointInT, KeypointT>::PointCloudInPtr PointCloudInPtr;

	typedef typename FeaturesFinder<PointInT, KeypointT>::PointCloudKeypoint PointCloudKeypoint;	

	typedef cv::FeatureDetector FeatureDetector;
	typedef cv::Ptr<FeatureDetector> FeatureDetectorPtr;

	typedef cv::DescriptorExtractor DescriptorExtractor;
	typedef cv::Ptr<DescriptorExtractor> DescriptorExtractorPtr;

	OpenCVFeaturesFinder(const FeatureDetectorPtr &keypointDetector = FeatureDetectorPtr(), 
						 const DescriptorExtractorPtr &descriptorExtractor = DescriptorExtractorPtr()) :	
		keypointDetector_(keypointDetector), 
		descriptorExtractor_(descriptorExtractor){}

	virtual ~OpenCVFeaturesFinder() {};

	void 
	computeKeypoints(PointCloudKeypoint &keypoints);

	void 
	computeKeypointsAndDescriptors(PointCloudKeypoint &keypoints,
								   Descriptors &descriptors);	

	void
	computeDescriptors(const PointCloudKeypoint &keypoints, 
 					   Descriptors &descriptors, 
 					   PointCloudKeypoint &remaining_keypoints);
 
	void 
	setInputCloud(const PointCloudInPtr &cloud);

	void 
	setKeypointDetector(const FeatureDetectorPtr &keypointDetector)
	{
		keypointDetector_ = keypointDetector;	
	}

	void 
	setDescriptorExtractor(const DescriptorExtractorPtr &descriptorExtractor)
	{
		descriptorExtractor_ = descriptorExtractor;
	}

protected:
	virtual void 
	convertKeypoints(const std::vector<cv::KeyPoint> &cv_keypoints,
					 PointCloudKeypoint &keypoints,
					 std::vector<uint> &indexes,
					 bool save_indexes = true);

	virtual void 
	convertKeypoints(const PointCloudKeypoint &keypoints,
					 std::vector<cv::KeyPoint> &cv_keypoints,
					 std::vector<uint> &indexes,
					 bool save_indexes = true);

	virtual void 
	computeDescriptors(std::vector<cv::KeyPoint> &cv_keypoints,
					   Descriptors &descriptors, 
					   PointCloudKeypoint &remaining_keypoints);

	PointCloudInPtr rgb_cloud_; 
	cv::Mat cv_rgb_image_;
	pcl::PCLImage pcl_rgb_image_;

	FeatureDetectorPtr keypointDetector_;
	DescriptorExtractorPtr descriptorExtractor_;

	double fx_, fy_, cx_, cy_;
};

} // namespace features

#include <OpenCVFeaturesFinder.hpp>

#endif // OPENCV_FEATURES_FINDER_H