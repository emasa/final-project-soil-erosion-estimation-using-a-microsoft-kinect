#ifndef OPENCV_REAL_FEATURES_FINDER_HPP
#define OPENCV_REAL_FEATURES_FINDER_HPP

#include <opencv2/core/core.hpp>

// #include "exceptions.h"
#include "features/descriptor_types.h"
#include "features/OpenCVRealFeaturesFinder.h"

template<typename PointInT, typename KeypointT, typename RealDescriptorT> void
OpenCVRealFeaturesFinder<PointInT, KeypointT, RealDescriptorT>::convertDescriptors(const cv::Mat &cv_descriptors, 
						   										  				   PointCloudDescriptor &descriptors)
{
	// if (cv_descriptors.type != CV_32F) throw NotRealDescriptor();

	OpenCVBaseFeaturesFinder<PointInT, KeypointT, RealDescriptorT>::convertDescriptors(cv_descriptors, descriptors);
}


#endif // OPENCV_REAL_FEATURES_FINDER_HPP
