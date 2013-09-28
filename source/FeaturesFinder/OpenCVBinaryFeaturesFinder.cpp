
#include <pcl/point_types.h>

#include "Descriptor/DescriptorType.h"
#include "FeaturesFinder/OpenCVBinaryFeaturesFinder.h"
#include "FeaturesFinder/OpenCVBinaryFeaturesFinder.hpp"

template<> class OpenCVBinaryFeaturesFinder<pcl::PointXYZRGBA, pcl::PointWithScale, ORBSignature32>;
