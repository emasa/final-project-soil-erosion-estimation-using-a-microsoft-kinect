
#ifndef FEATURES_FINDER_H
#define FEATURES_FINDER_H

#include <memory>

#include <pcl/common/common.h>

#include "../include/Features.h"

namespace features
{

class FeaturesFinder
{
public:
	typedef std::shared_ptr<FeaturesFinder> Ptr;
	typedef std::shared_ptr<const FeaturesFinder> ConstPtr;

	virtual ~FeaturesFinder() {};

	virtual void find(pcl::PointCloud<BaseKeypoint> &keypoints, 
					  Descriptors &descriptors) = 0;

	virtual void setCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud) = 0;
};


}

#endif // FEATURES_FINDER_H
