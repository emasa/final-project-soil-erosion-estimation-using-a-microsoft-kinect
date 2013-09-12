
#include <pcl/point_types.h>

#include "features/descriptor_types.h"
#include "features/OpenCVBinaryFeaturesFinder.h"

template<> class OpenCVBinaryFeaturesFinder<pcl::PointXYZRGBA, pcl::PointWithScale, ORBSignature32>;
