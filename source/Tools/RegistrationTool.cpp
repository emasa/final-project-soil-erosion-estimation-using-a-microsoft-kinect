
#include "GlobalRegistration/GlobalRegistration.h"
#include "Tools/RegistrationTool.h"

template<> class RegistrationTool<GlobalRegistration<pcl::PointXYZRGBA, pcl::PointWithScale, SURFSignature128>>; 