
#ifndef GLOBAL_REGISTRATION_FACTORY_H
#define GLOBAL_REGISTRATION_FACTORY_H

#include <pcl/point_types.h>

#include "features/descriptor_types.h"
#include "features/GlobalRegistration.h"

class GlobalRegistrationFactory
{
public:
	void
	create(GlobalRegistration<pcl::PointXYZRGBA, pcl::PointWithScale, SURFSignature128>::Ptr &registration) const;
};


#endif // GLOBAL_REGISTRATION_FACTORY_H