
#ifndef GLOBAL_REGISTRATION_FACTORY_H
#define GLOBAL_REGISTRATION_FACTORY_H

#include <pcl/point_types.h>

#include "Descriptor/DescriptorType.h"
#include "GlobalRegistration/GlobalRegistration.h"

class GlobalRegistrationFactory
{
public:
	GlobalRegistration<pcl::PointXYZRGBA, pcl::PointWithScale, SURFSignature128>::Ptr
	SURF(float fx, float fy, float cx, float cy, int hessian_threshold = 500) const;

	GlobalRegistration<pcl::PointXYZRGBA, pcl::PointWithScale, SURFSignature128>::Ptr
	ORBAndSURF(float fx, float fy, float cx, float cy, int nfeatures = 500) const;
};

#endif // GLOBAL_REGISTRATION_FACTORY_H