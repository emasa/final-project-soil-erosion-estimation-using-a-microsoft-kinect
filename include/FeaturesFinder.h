
#ifndef FEATURES_FINDER_H
#define FEATURES_FINDER_H

#include <memory>

#include <opencv2/core/core.hpp>
#include <pcl/common/common.h>

#include "Features.h"

namespace features
{

template<typename PointInT, typename KeypointT> 
class FeaturesFinder
{
public:
	typedef std::shared_ptr<FeaturesFinder> Ptr;
	typedef std::shared_ptr<const FeaturesFinder> ConstPtr;
	
	typedef typename pcl::PointCloud<PointInT> PointCloudIn;
	typedef typename PointCloudIn::Ptr PointCloudInPtr;

	typedef typename pcl::PointCloud<KeypointT> PointCloudKeypoint;
	
	virtual ~FeaturesFinder() {};

	virtual void 
	computeKeypointsAndDescriptors(PointCloudKeypoint &keypoints, 
				  				   Descriptors &descriptors) = 0;	

	virtual void 
	computeKeypoints(PointCloudKeypoint &keypoints) = 0;

	virtual void 
	computeDescriptors(const PointCloudKeypoint &keypoints, 
 					   Descriptors &descriptors, 
 					   PointCloudKeypoint &remaining_keypoints) = 0;

	virtual void 
	setInputCloud(const PointCloudInPtr &cloud) = 0;
};

} // namespace features

#endif // FEATURES_FINDER_H
