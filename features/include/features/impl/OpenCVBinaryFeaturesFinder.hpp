#ifndef OPENCV_BINARY_FEATURES_FINDER_HPP
#define OPENCV_BINARY_FEATURES_FINDER_HPP

#include <opencv2/core/core.hpp>

// #include "exceptions.h"
#include "features/descriptor_types.h"
#include "features/OpenCVBinaryFeaturesFinder.h"

template<typename PointInT, typename KeypointT> void
OpenCVBinaryFeaturesFinder<PointInT, KeypointT>::convertDescriptors(const cv::Mat &cv_descriptors, 
						   											PointCloudDescriptor &descriptors)
{
	// if (cv_descriptors.type != CV_8U) throw NotBinaryDescriptor();

	OpenCVBaseFeaturesFinder<PointInT, KeypointT, CustomSizeBinaryDescriptor>::convertDescriptors(cv_descriptors, descriptors);
}


#endif // OPENCV_BINARY_FEATURES_FINDER_HPP
