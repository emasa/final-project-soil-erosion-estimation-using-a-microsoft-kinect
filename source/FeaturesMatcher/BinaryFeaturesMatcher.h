#ifndef BINARY_FEATURES_MATCHER
#define BINARY_FEATURES_MATCHER

#include <memory>

#include <Descriptor/DescriptorRepresentation.h>
#include <pcl/correspondence.h>

#include "FeaturesMatcher/FeaturesMatcher.h"

template<typename PointInT>
class BinaryFeaturesMatcher : public FeaturesMatcher<PointInT>
{
public:
	typedef std::shared_ptr<BinaryFeaturesMatcher> Ptr;
	typedef std::shared_ptr<const BinaryFeaturesMatcher> ConstPtr;
	
	typedef typename FeaturesMatcher<PointInT>::PointCloudIn PointCloudIn;
	typedef typename PointCloudIn::Ptr PointCloudInPtr;	
	
	typedef typename pcl::BinaryPointRepresentation<PointInT>::ConstPtr PointRepresentationConstPtr;
	
	BinaryFeaturesMatcher(bool cross_check=false) 
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

#include "FeaturesMatcher/BinaryFeaturesMatcher.hpp"

#endif // BINARY_FEATURES_MATCHER