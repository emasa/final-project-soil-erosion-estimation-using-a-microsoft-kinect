#ifndef FEATURES_MATCHER_H
#define FEATURES_MATCHER_H

#include <memory>

#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>

template<typename PointInT>
class FeaturesMatcher
{
public:
	typedef std::shared_ptr<FeaturesMatcher> Ptr;
	typedef std::shared_ptr<const FeaturesMatcher> ConstPtr;

	typedef pcl::PointCloud<PointInT> PointCloudIn;
	typedef typename PointCloudIn::Ptr PointCloudInPtr;

	virtual ~FeaturesMatcher() {};

	virtual void 
	match(const PointCloudInPtr &src, 
		  const PointCloudInPtr &tgt,
		  pcl::Correspondences &correspondences) = 0;
};

#endif // FEATURES_MATCHER_H