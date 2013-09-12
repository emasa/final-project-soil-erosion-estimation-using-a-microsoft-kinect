#ifndef REAL_FEATURES_MATCHER
#define REAL_FEATURES_MATCHER
#define PCL_NO_PRECOMPILE

#include <memory>

#include <pcl/point_representation.h>
#include <pcl/correspondence.h>

#include "features/FeaturesMatcher.h"

template<typename PointInT>
class RealFeaturesMatcher : public FeaturesMatcher<PointInT>
{
public:
	typedef std::shared_ptr<RealFeaturesMatcher> Ptr;
	typedef std::shared_ptr<const RealFeaturesMatcher> ConstPtr;
	
	typedef typename FeaturesMatcher<PointInT>::PointCloudIn PointCloudIn;
	typedef typename PointCloudIn::Ptr PointCloudInPtr;	
	
	typedef typename pcl::PointRepresentation<PointInT>::ConstPtr PointRepresentationConstPtr;
	
	RealFeaturesMatcher(bool cross_check=false) 
		: cross_check_(cross_check), 
		  point_representation_() {}

	void 
	match(const PointCloudInPtr &src, 
		  const PointCloudInPtr &tgt,
		  pcl::Correspondences &correspondences);

	inline void 
	setPointRepresentation(const PointRepresentationConstPtr &point_representation)
	{
		point_representation_ = point_representation;
	}

	inline PointRepresentationConstPtr
	getPointRepresentation()
	{
		return point_representation_;
	}

	inline bool
	isCrossCheckEnabled()
	{
		return cross_check_;
	}

private:
	bool cross_check_;
	PointRepresentationConstPtr point_representation_;
};

#include "features/impl/RealFeaturesMatcher.hpp"

#endif // REAL_FEATURES_MATCHER