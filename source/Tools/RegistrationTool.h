
#ifndef REGISTRATION_TOOL_H
#define REGISTRATION_TOOL_H

#include <string>
#include <vector>
#include <memory>

#include <boost/function.hpp>
#include <boost/filesystem.hpp> 

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
private:
	enum DeviceMode { CAMERA, FILES };

public:
	typedef std::shared_ptr<RegistrationTool> Ptr;
	typedef std::shared_ptr<const RegistrationTool> ConstPtr;

	typedef typename RegistrationAlgorithm::Ptr RegistrationAlgorithmPtr;

	RegistrationTool(bool backup_enabled=true, int digits=5);

	bool
	setUpOutputDirectory(const std::string &dir);

	void 
	setRegistrationAlgorithm(const RegistrationAlgorithmPtr &registration);

	void
	registerFromFiles(const std::vector<std::string>& clouds);

	void
	registerFromCamera(const std::string& device_id="");

	void
	registerFromFilesAndThenFromCamera(const std::vector<std::string>& clouds, 
									   const std::string& device_id="");

private:
	bool
	captureAndRegister();

	void
	checkpoint(int idx = -1);

	void
	backup(int idx = -1);

	void 
	finish();

	void
	initVisualization();

	void
	runVisualizationLoop();

	void
	updateRegistrationVisualization(int idx = -1);

	void
	updateStreamingVisualization();

	bool
	start();

	bool
	initDevice();

	void
	keyboardManager(const pcl::visualization::KeyboardEvent &event);	

	boost::filesystem::path
	cloudPathFormat(const boost::filesystem::path &dir, int idx);

	int processed_clouds_;
	
	bool started_, finished_;

	pcl::visualization::PCLVisualizer viewer_;
	int stream_viewport_, registration_viewport_;
	boost::mutex mutex_;
	bool visualization_enabled_;

	CloudGenerator generator_;
	bool is_device_generating_;

	DeviceMode mode_;
	std::string device_id_;
	std::vector<std::string> clouds_;

	RegistrationAlgorithmPtr registration_;

	int digits_;
	bool backup_enabled_;
	boost::filesystem::path root_dir_, registration_dir_, backup_dir_;
};

#include "Tools/RegistrationTool.hpp"

#endif // REGISTRATION_TOOL_H