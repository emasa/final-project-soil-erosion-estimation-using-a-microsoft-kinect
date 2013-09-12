
#include <pcl/point_types.h>

#include "features/descriptor_types.h"
#include "features/OpenCVRealFeaturesFinder.h"

template<> class OpenCVRealFeaturesFinder<pcl::PointXYZRGBA, pcl::PointWithScale, SURFSignature64>;
template<> class OpenCVRealFeaturesFinder<pcl::PointXYZRGBA, pcl::PointWithScale, SURFSignature128>;