
#ifndef REGISTRATION_TOOL_HPP
#define REGISTRATION_TOOL_HPP

#include <vector>
#include <string>
#include <iostream>
#include <sstream>
#include <cmath>

#include <boost/thread/thread.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/filesystem.hpp> 
#include <boost/format.hpp>

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/file_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_grabber.h>
#include <pcl/exceptions.h>	
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "Common/Status.h"
#include "Utils/Utils.h"
#include "IO/CloudGenerator.h"
#include "Tools/RegistrationTool.h"

template<typename RegistrationAlgorithm>
RegistrationTool<RegistrationAlgorithm>::RegistrationTool(bool backup_enabled, int digits)
	: processed_clouds_(0)
	, visualized_clouds_(0)
	, started_(false) 
	, finished_(false)
	, generator_()
	, is_device_generating_(false)
	, registration_()
	, visualization_enabled_(false)
	, viewer_("Visual registration tool", false)
	, stream_viewport_()
	, registration_viewport_()
	, root_dir_()
	, registration_dir_()
	, backup_enabled_(backup_enabled)
	, backup_dir_()
	, digits_(digits)
{
	assert(0 < digits_); assert(digits_ < 10);
}

template<typename RegistrationAlgorithm> void
RegistrationTool<RegistrationAlgorithm>::registerFromFiles(const std::vector<std::string>& clouds)
{
	mode_ = FILES;
	clouds_ = clouds;
	if ( !start() ) return;

	for (int count = 0; count < clouds.size(); ++count)
	{
		captureAndRegister();
	}

	finish();
	checkpoint();
}

template<typename RegistrationAlgorithm> void
RegistrationTool<RegistrationAlgorithm>::registerFromCamera(const std::string& device_id)
{
	mode_ = CAMERA;
	device_id_ = device_id;
	if ( !start() ) return;

	initVisualization();
	runVisualizationLoop();
}

template<typename RegistrationAlgorithm> void
RegistrationTool<RegistrationAlgorithm>::registerFromFilesAndThenFromCamera
	(const std::vector<std::string>& clouds, const std::string& device_id)
{
	mode_ = FILES;
	clouds_ = clouds;
	if ( !start() ) return;	

	int to_visualize = 0;
	for (int count = 0 ; count < clouds.size() ; ++count)
	{
		if ( captureAndRegister() ) ++to_visualize;
	}

	mode_ = CAMERA;
	device_id_ = device_id;

	if ( !initDevice() ) return;

	visualization_enabled_ = true;

	initVisualization();

	updateVisualization();

	runVisualizationLoop();
}

template<typename RegistrationAlgorithm> bool
RegistrationTool<RegistrationAlgorithm>::start()
{
	if (started_) 
	{
		PCL_WARN("Registration is %s...\n", !finished_ ? "finished" : "running"); 
		return false;
	}
	
	if ( root_dir_.empty() || registration_dir_.empty() || (backup_enabled_ && backup_dir_.empty()) )
	{
		PCL_ERROR("Output directory wasn't setup properly ... ERROR\n"); 
		return false;	
	} 

	if ( !registration_ ) 
	{
		PCL_ERROR("Registration algorithm wasn't seted... ERROR\n"); 
		return false;
	}

	if ( !initDevice() ) 
	{
		return false;
	}

	PCL_INFO("Starting registration... OK\n");
	started_ = true;

	return true;
}

template<typename RegistrationAlgorithm> bool
RegistrationTool<RegistrationAlgorithm>::initDevice()
{

	if (generator_.isRunning()) generator_.stopGenerating();
	
	Status st = SUCCESS;
	try 
	{	
		CloudGenerator::GrabberPtr grabber;
		if (mode_ == CAMERA) 
		{
			grabber = std::make_shared<pcl::OpenNIGrabber>(device_id_);
		} else if (mode_ == FILES) 
		{
			grabber = std::make_shared<pcl::PCDGrabber<pcl::PointXYZRGBA>>(clouds_);
		}
		
		st = generator_.setGrabberImplementation(grabber);	
	} catch (pcl::IOException &e) {
		st = DEVICE_NOT_WORKING;
	}
	PCL_INFO("Initializing generator...%s\n", st == SUCCESS ? "OK" : "ERROR");

	if (st == SUCCESS) 
	{
		st = generator_.startGenerating();
		PCL_INFO("Starting to generate...%s\n", st == SUCCESS ? "OK" : "ERROR");
	}

	if (st == SUCCESS)
	{
		is_device_generating_ = true;
		PCL_INFO("Opening device...%s\n", "OK");
	} else {
		PCL_ERROR("Opening device...%s\n", "ERROR");
	}

	return st == SUCCESS;
}

template<typename RegistrationAlgorithm> void
RegistrationTool<RegistrationAlgorithm>::finish()
{
	if (!started_ || finished_) 
	{
		PCL_WARN("Registration is not running...\n"); return;
	}
	// stop generating clouds
	generator_.stopGenerating();
	// perform global optimazition
	registration_->globalOptimize();

	PCL_INFO("Total aligned %i / %i clouds.\n\n", registration_->getNumClouds(), 
												  processed_clouds_);
	
	finished_ = true;
}

template<typename RegistrationAlgorithm> bool
RegistrationTool<RegistrationAlgorithm>::captureAndRegister()
{
	if (!started_ || finished_)	
	{
		PCL_WARN("Registration is not running...\n"); 
		return false;
	}

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);				
	if (generator_.generate(cloud) != SUCCESS)
	{
		is_device_generating_ = false;
		PCL_ERROR("Generating cloud... ERROR\n\n"); 
		return false;
	}

	PCL_INFO("Registering cloud %i\n", ++processed_clouds_);

	try 
	{	
		if (registration_->proccessCloud(cloud) != SUCCESS) 
		{
			PCL_ERROR("Error found. Skipping cloud... ERROR\n\n");
			return false;
		}
	}catch(pcl::UnorganizedPointCloudException &e) 
	{
		PCL_ERROR("Organized cloud is required... ERROR\n\n", processed_clouds_);
		return false;
	}

	if (backup_enabled_) 
	{
		backup(registration_->getNumClouds() - 1);
	}

	PCL_INFO("Successfully aligned...\n\n");
	return true;
}

template<typename RegistrationAlgorithm> void
RegistrationTool<RegistrationAlgorithm>::updateVisualization()
{
	namespace vis = pcl::visualization;

	PCL_INFO("Updating visualization...\n");

	for (int cloud_idx = 0 ; cloud_idx < registration_->getNumClouds() ; ++cloud_idx)
	{
		auto cloud = registration_->getInputCloud(cloud_idx);
		auto cloud_str = std::to_string(cloud_idx);

		if (cloud_idx == visualized_clouds_)
		{
			++visualized_clouds_;
			vis::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> color_handler(cloud);
			viewer_.addPointCloud<pcl::PointXYZRGBA>(cloud, color_handler, cloud_str, registration_viewport_);	
	      	// viewer_.setPointCloudRenderingProperties (vis::PCL_VISUALIZER_IMMEDIATE_RENDERING, 1.0, cloud_str);		
		}

		auto PITCH_ROTATION = pcl::getTransformation(0, 0, 0, 0, M_PI, 0);
		auto visual_pose = PITCH_ROTATION * registration_->getTransformation(cloud_idx);
		viewer_.updatePointCloudPose(cloud_str, visual_pose);
	}

	updateStreamingVisualization();
}

template<typename RegistrationAlgorithm> void
RegistrationTool<RegistrationAlgorithm>::updateStreamingVisualization()
{
	if (registration_->getNumClouds() == 0) return;

	auto last_cloud = registration_->getInputCloud(registration_->getNumClouds() - 1);
	viewer_.removePointCloud("last_cloud", stream_viewport_);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> color_handler(last_cloud);
	viewer_.addPointCloud(last_cloud, color_handler, "last_cloud", stream_viewport_);	
	viewer_.updatePointCloudPose("last_cloud", viewer_.getViewerPose(stream_viewport_));
}

template<typename RegistrationAlgorithm> void
RegistrationTool<RegistrationAlgorithm>::initVisualization()
{
	auto cb = boost::bind (&RegistrationTool::keyboardManager, this, _1);
	viewer_.registerKeyboardCallback (cb);

	viewer_.createViewPort(0, 0, 0.4, 1, stream_viewport_);
	viewer_.createViewPortCamera (stream_viewport_);
	viewer_.createViewPort(0.4, 0, 1, 1, registration_viewport_);
	viewer_.createViewPortCamera (registration_viewport_);

	viewer_.createInteractor();
}

template<typename RegistrationAlgorithm> void
RegistrationTool<RegistrationAlgorithm>::runVisualizationLoop()
{
	PCL_INFO("Running visualization... OK\n");

	int sleep_time = 100, visualization_time = 100; // in milliseconds
	while (started_ && !finished_)
	{
		viewer_.spinOnce (visualization_time);
		boost::this_thread::sleep (boost::posix_time::milliseconds (sleep_time));
	}
}

template<typename RegistrationAlgorithm> void
RegistrationTool<RegistrationAlgorithm>::setRegistrationAlgorithm(const RegistrationAlgorithmPtr &registration)
{
	registration_ = registration;
}

template<typename RegistrationAlgorithm> bool
RegistrationTool<RegistrationAlgorithm>::setUpOutputDirectory(const std::string &dir)
{
	namespace fs = boost::filesystem;

	auto setUpDir = [](const fs::path & dir_path) -> bool
					{
						if (fs::exists(dir_path) && !fs::is_directory(dir_path))
						{
							PCL_ERROR("%s isn't a directory path.\n", dir_path.c_str()); 
							return false;
						}
						try{
							fs::create_directories(dir_path);
						} catch(boost::filesystem3::filesystem_error &e) {
							PCL_ERROR("Can't create %s directory.\n", dir_path.c_str()); 
							return false;
						}
						return true;
					};

	root_dir_ = fs::path(dir);

	registration_dir_ = root_dir_ / "registration";
	bool success = setUpDir(registration_dir_);
	
	if (success && backup_enabled_)
	{
		backup_dir_ = root_dir_ / "backup";
		success = setUpDir(backup_dir_);	
	}

	return success;
}

template<typename RegistrationAlgorithm> void
RegistrationTool<RegistrationAlgorithm>::checkpoint(int idx)
{	
	assert( idx == -1 || (0 <= idx && idx < registration_->getNumClouds()) );
	assert(!root_dir_.empty()); assert(!registration_dir_.empty()); 

	int start_idx = idx != -1 ? idx   : 0;
	int stop_idx  = idx != -1 ? idx+1 : registration_->getNumClouds();
	for (int cloud_idx = start_idx ; cloud_idx < stop_idx ; ++cloud_idx)
	{	
		pcl::PointCloud<pcl::PointXYZRGBA> aligned_cloud;

		pcl::transformPointCloud(*registration_->getInputCloud(cloud_idx), 
						    	 aligned_cloud, 
								 registration_->getTransformation(cloud_idx));
		
		auto cloud_path = cloudPathFormat(registration_dir_, cloud_idx);
		pcl::PCLPointCloud2 blob;
		pcl::toPCLPointCloud2(aligned_cloud, blob);
		saveCloud(cloud_path.c_str(), blob);
	}
}

template<typename RegistrationAlgorithm> void
RegistrationTool<RegistrationAlgorithm>::backup(int idx)
{	
	assert( idx == -1 || (0 <= idx && idx < registration_->getNumClouds()) );
	assert(!root_dir_.empty()); assert(!backup_enabled_ || !backup_dir_.empty());

	if (!backup_enabled_)
	{
		PCL_WARN("Backup is not enabled\n"); return;
	}

	int start_idx = idx != -1 ? idx   : 0;
	int stop_idx  = idx != -1 ? idx+1 : registration_->getNumClouds();
	for (int cloud_idx = start_idx ; cloud_idx < stop_idx ; ++cloud_idx)
	{			
		auto cloud_path = cloudPathFormat(backup_dir_, cloud_idx);
		pcl::PCLPointCloud2 blob;
		pcl::toPCLPointCloud2(*registration_->getInputCloud(cloud_idx), blob);
		saveCloud(cloud_path.c_str(), blob);
	}
}

template<typename RegistrationAlgorithm> boost::filesystem::path
RegistrationTool<RegistrationAlgorithm>::cloudPathFormat(const boost::filesystem::path &dir, int idx)
{
	assert(0 <= idx); assert(idx < static_cast<int>(std::pow(10, digits_)));

	std::ostringstream ss;
	ss << std::setw( digits_ ) << std::setfill( '0' ) << idx;

	return dir / (ss.str() + ".pcd");
}

template<typename RegistrationAlgorithm> void
RegistrationTool<RegistrationAlgorithm>::keyboardManager(const pcl::visualization::KeyboardEvent &event)
{
	if (event.keyDown())
	{
		std::string key = event.getKeySym();
		if (key == "space")
		{
			if ( captureAndRegister() )
			{
				updateVisualization();
			}
		} else if (key == "p" || key == "P")
		{
			finish();
			checkpoint();
		} else if (key == "s" || key == "S")
		{
			checkpoint();
		}		
	}
}

#endif // REGISTRATION_TOOL_HPP