
#include <pcl/point_types.h>

#include "Descriptor/DescriptorType.h"
#include "GlobalRegistration/GlobalRegistration.h"
#include "GlobalRegistration/GlobalRegistration.hpp"

template<> class GlobalRegistration<pcl::PointXYZRGBA, pcl::PointWithScale, ORBSignature32>;
template<> class GlobalRegistration<pcl::PointXYZRGBA, pcl::PointWithScale, SURFSignature64>;
template<> class GlobalRegistration<pcl::PointXYZRGBA, pcl::PointWithScale, SURFSignature128>;
