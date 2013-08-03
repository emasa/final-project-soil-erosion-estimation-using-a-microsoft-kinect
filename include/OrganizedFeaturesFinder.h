
#ifndef ORGANIZED_FEATURES_FINDER_H
#define ORGANIZED_FEATURES_FINDER_H

#include <vector>
#include <memory>

#include <pcl/common/common.h>
#include <opencv2/features2d/features2d.hpp>

#include "../include/FeaturesFinder.h"

namespace features 
{

class OrganizedFeaturesFinder : public FeaturesFinder
{
public:
	typedef std::shared_ptr<OrganizedFeaturesFinder> Ptr;
	typedef std::shared_ptr<const OrganizedFeaturesFinder> ConstPtr;

	OrganizedFeaturesFinder() = default;
	OrganizedFeaturesFinder(const cv::Ptr<cv::FeatureDetector> &keypointDetector_, 
						 	const cv::Ptr<cv::DescriptorExtractor> &descriptorExtractor_);

	virtual ~OrganizedFeaturesFinder() = default;

	virtual void find(pcl::PointCloud<BaseKeypoint> &keypoints, 
					  Descriptors &descriptors);	

	virtual void setCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_);

	void setKeypointDetector(const cv::Ptr<cv::FeatureDetector> &keypointDetector_);

	void setDescriptorExtractor(const cv::Ptr<cv::DescriptorExtractor> &descriptorExtractor_);

	// virtual void find(std::vector<OrganizedKeypoint> &keypoints,
	// 				  Descriptors &keypoints);

	// virtual void findKeypoints(std::vector<OrganizedKeypoint> &keypoints); 

	// virtual void computeDescriptors(const std::vector<OrganizedKeypoint> &keypoints, 
	// 							    Descriptors &keypoints);

protected:
	virtual void convertKeypoints(const std::vector<cv::KeyPoint> &cv_keypoints,
						  		  pcl::PointCloud<BaseKeypoint> &keypoints);

protected:
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud; 
	cv::Mat image;

	cv::Ptr<cv::FeatureDetector> keypointDetector;
	cv::Ptr<cv::DescriptorExtractor> descriptorExtractor;

	std::vector<cv::KeyPoint> cv_keypoints;
};

} // namespace features

#endif // ORGANIZED_FEATURES_FINDER_H