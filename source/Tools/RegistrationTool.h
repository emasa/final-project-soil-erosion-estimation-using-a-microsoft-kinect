
#ifndef REGISTRATION_TOOL_H
#define REGISTRATION_TOOL_H

#include <string>
#include <vector>
#include <memory>

#include <pcl/point_types.h>
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "Descriptor/DescriptorType.h"
#include "GlobalRegistration/GlobalRegistration.h"
#include "IO/CloudGenerator.h"

class RegistrationTool
{
public:
	typedef std::shared_ptr<RegistrationTool> Ptr;
	typedef std::shared_ptr<const RegistrationTool> ConstPtr;

	RegistrationTool();

	void
	initRegistering(const std::vector<std::string> &filenames);

	void 
	stopRegistering();

	void
	registerNewCloud();

	void
	updateVisualization(int cloud_idx);

	void 
	saveAlignedClouds(const std::string &directory);

	void 
	saveAlignedClouds();

	void
	keyboardManager(const pcl::visualization::KeyboardEvent &event);	

	void 
	run();

private:
	int processed_clouds_, visualized_clouds_;
	
	bool downsample_enabled_;
	bool registration_running_;

	pcl::visualization::PCLVisualizer viewer_;

	CloudGenerator generator_;

	GlobalRegistration<pcl::PointXYZRGBA, pcl::PointWithScale, SURFSignature128>::Ptr registration_;
};

#endif // REGISTRATION_TOOL_H