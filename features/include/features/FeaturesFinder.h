
#ifndef FEATURES_FINDER_H
#define FEATURES_FINDER_H

#include <memory>

#include <pcl/point_cloud.h>

template<typename PointInT, typename KeypointT, typename DescriptorT> 
class FeaturesFinder
{
public:
	typedef std::shared_ptr<FeaturesFinder> Ptr;
	typedef std::shared_ptr<const FeaturesFinder> ConstPtr;
	
	typedef typename pcl::PointCloud<PointInT> PointCloudIn;
	typedef typename PointCloudIn::Ptr PointCloudInPtr;
	typedef typename PointCloudIn::ConstPtr PointCloudInConstPtr;

	typedef typename pcl::PointCloud<KeypointT> PointCloudKeypoint;
	typedef typename pcl::PointCloud<DescriptorT> PointCloudDescriptor;
	
	virtual ~FeaturesFinder() {};

	virtual void 
	computeKeypointsAndDescriptors(PointCloudKeypoint &keypoints, 
				  				   PointCloudDescriptor &descriptors) = 0;	

	virtual void 
	computeKeypoints(PointCloudKeypoint &keypoints) = 0;

	virtual void 
	computeDescriptors(const PointCloudKeypoint &keypoints, 
 					   PointCloudDescriptor &descriptors, 
 					   PointCloudKeypoint &remaining_keypoints) = 0;

	virtual void 
	setInputCloud(const PointCloudInPtr &cloud) = 0;
};

#endif // FEATURES_FINDER_H
