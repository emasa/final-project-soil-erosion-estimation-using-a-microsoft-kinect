
#ifndef REGISTRATION_TOOL_H
#define REGISTRATION_TOOL_H

#include <string>
#include <vector>
#include <memory>

#include <boost/function.hpp>

#include <pcl/point_types.h>
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/grabber.h>

#include "Descriptor/DescriptorType.h"
#include "GlobalRegistration/GlobalRegistration.h"
#include "IO/CloudGenerator.h"

template<typename RegistrationAlgorithm>
class RegistrationTool
{
public:
	typedef std::shared_ptr<RegistrationTool> Ptr;
	typedef std::shared_ptr<const RegistrationTool> ConstPtr;

	typedef typename RegistrationAlgorithm::Ptr RegistrationAlgorithmPtr;
	typedef CloudGenerator::GrabberPtr GrabberPtr;

	RegistrationTool(bool downsample=false);

	void 
	setRegistrationAlgorithm(const RegistrationAlgorithmPtr &registration);

	void
	start(const std::string &device_id="");

	void
	start(const std::vector<std::string> &clouds);

	void 
	finish();

	void 
	run();

	void
	captureAndRegister();

	void 
	saveAlignedClouds(const std::string &directory);

	void
	updateRegistrationVisualization(int cloud_idx);

	void
	updateStreamingVisualization(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud);

	void
	keyboardManager(const pcl::visualization::KeyboardEvent &event);	

private:

	void
	commonStart(const boost::function<GrabberPtr ()> &grabber_factory);

	int processed_clouds_, visualized_clouds_;
	
	bool started_, finished_;

	bool downsample_enabled_;
	pcl::visualization::PCLVisualizer viewer_;
	int stream_viewport_, registration_viewport_;
	boost::mutex mutex_;

	GrabberPtr grabber_;
	CloudGenerator generator_;

	RegistrationAlgorithmPtr registration_;
};

#include "Tools/RegistrationTool.hpp"

#endif // REGISTRATION_TOOL_H