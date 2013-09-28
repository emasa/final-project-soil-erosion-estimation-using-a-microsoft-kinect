
#ifndef REAL_FEATURES_MATCHER_HPP
#define REAL_FEATURES_MATCHER_HPP
#define PCL_NO_PRECOMPILE

#include <pcl/correspondence.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/search/impl/kdtree.hpp>

#include "FeaturesMatcher/RealFeaturesMatcher.h"

template<typename PointInT> void 
RealFeaturesMatcher<PointInT>::match(const PointCloudInPtr &src, 
									 const PointCloudInPtr &tgt,
									 pcl::Correspondences &correspondences)
{
	pcl::registration::CorrespondenceEstimation<PointInT, PointInT, float> matcher;

	matcher.setInputSource(src);
	matcher.setInputTarget(tgt);
	matcher.setPointRepresentation(point_representation_);

	if (cross_check_) {
		matcher.determineReciprocalCorrespondences(correspondences);
	} else {
		matcher.determineCorrespondences(correspondences);
	}
}

#endif // REAL_FEATURES_MATCHER_HPP