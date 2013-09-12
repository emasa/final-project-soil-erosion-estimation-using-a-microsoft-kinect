
#include <pcl/point_types.h>

#include "features/descriptor_types.h"

#include "features/GlobalRegistration.h"

template<> class GlobalRegistration<pcl::PointXYZRGBA, pcl::PointWithScale, ORBSignature32>;
template<> class GlobalRegistration<pcl::PointXYZRGBA, pcl::PointWithScale, SURFSignature64>;
template<> class GlobalRegistration<pcl::PointXYZRGBA, pcl::PointWithScale, SURFSignature128>;
