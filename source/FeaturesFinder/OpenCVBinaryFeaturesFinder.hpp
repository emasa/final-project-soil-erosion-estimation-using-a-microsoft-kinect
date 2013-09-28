#ifndef OPENCV_BINARY_FEATURES_FINDER_HPP
#define OPENCV_BINARY_FEATURES_FINDER_HPP

#include <opencv2/core/core.hpp>

#include "FeaturesFinder/OpenCVBinaryFeaturesFinder.h"

template<typename PointInT, typename KeypointT, typename BinaryDescriptorT> void
OpenCVBinaryFeaturesFinder<PointInT, KeypointT, BinaryDescriptorT>::convertDescriptors(const cv::Mat &cv_descriptors, 
						   															   PointCloudDescriptor &descriptors)
{
	// if (cv_descriptors.type != CV_8U) throw NotBinaryDescriptor();

	OpenCVBaseFeaturesFinder<PointInT, KeypointT, BinaryDescriptorT>::convertDescriptors(cv_descriptors, descriptors);
}


#endif // OPENCV_BINARY_FEATURES_FINDER_HPP
