
#include <pcl/point_types.h>

#include "Descriptor/DescriptorType.h"
#include "FeaturesFinder/OpenCVRealFeaturesFinder.h"
#include "FeaturesFinder/OpenCVRealFeaturesFinder.hpp"

template<> class OpenCVRealFeaturesFinder<pcl::PointXYZRGBA, pcl::PointWithScale, SURFSignature64>;
template<> class OpenCVRealFeaturesFinder<pcl::PointXYZRGBA, pcl::PointWithScale, SURFSignature128>;