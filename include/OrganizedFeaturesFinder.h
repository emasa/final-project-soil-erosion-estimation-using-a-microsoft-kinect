
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

	OrganizedFeaturesFinder(const cv::Ptr<cv::FeatureDetector> &keypointDetector_= cv::Ptr<cv::FeatureDetector>(), 
						 	const cv::Ptr<cv::DescriptorExtractor> &descriptorExtractor = cv::Ptr<cv::DescriptorExtractor>());

	OrganizedFeaturesFinder(double fx, double fy, double cx, double cy,
							const cv::Ptr<cv::FeatureDetector> &keypointDetector = cv::Ptr<cv::FeatureDetector>(), 
						 	const cv::Ptr<cv::DescriptorExtractor> &descriptorExtractor = cv::Ptr<cv::DescriptorExtractor>());

	virtual ~OrganizedFeaturesFinder() = default;


	virtual void computeKeypointsAndDescriptors(pcl::PointCloud<BaseKeypoint> &keypoints, 
				  				   				Descriptors &descriptors);	

	virtual void computeKeypoints(pcl::PointCloud<BaseKeypoint> &keypoints);

	virtual void computeDescriptors(const pcl::PointCloud<BaseKeypoint> &keypoints, 
 									Descriptors &descriptors, 
 									pcl::PointCloud<BaseKeypoint> &remaining_keypoints);

	virtual void setCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud);

	void setKeypointDetector(const cv::Ptr<cv::FeatureDetector> &keypointDetector);

	void setDescriptorExtractor(const cv::Ptr<cv::DescriptorExtractor> &descriptorExtractor);

	void setCameraIntrinsics(double fx, double fy, double cx, double cy);


protected:
	virtual void convertKeypoints(const std::vector<cv::KeyPoint> &cv_keypoints,
		  						  pcl::PointCloud<BaseKeypoint> &keypoints,
		  						  std::vector<uint> &indexes,
		  						  bool save_indexes = true);

	virtual void convertKeypoints(const pcl::PointCloud<BaseKeypoint> &keypoints,
		  						  std::vector<cv::KeyPoint> &cv_keypoints,
		  						  std::vector<uint> &indexes,
		  						  bool save_indexes = true);

	virtual void computeDescriptors(std::vector<cv::KeyPoint> &cv_keypoints,
									Descriptors &descriptors, 
									pcl::PointCloud<BaseKeypoint> &remaining_keypoints);

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud; 
	cv::Mat image;

	cv::Ptr<cv::FeatureDetector> keypointDetector;
	cv::Ptr<cv::DescriptorExtractor> descriptorExtractor;

	double fx, fy, cx, cy;
	bool cameraIntrinsicsSeted;
};

} // namespace features

#endif // ORGANIZED_FEATURES_FINDER_H