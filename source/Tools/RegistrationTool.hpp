
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
#include <pcl/io/grabber.h>
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
	, grabber_()
	, generator_()
	, registration_()
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

	auto cb = boost::bind (&RegistrationTool::keyboardManager, this, _1);
	viewer_.registerKeyboardCallback (cb);

	// viewer_.createViewPort(0, 0, 0.5, 1, stream_viewport_);
	// viewer_.createViewPortCamera (stream_viewport_);
	
	// viewer_.createViewPort(0.5, 0, 1, 1, registration_viewport_);

	viewer_.createViewPort(0, 0, 1, 1, registration_viewport_);
	viewer_.createViewPortCamera (registration_viewport_);
}

template<typename RegistrationAlgorithm> void
RegistrationTool<RegistrationAlgorithm>::start(const std::string& device_id)
{
	boost::function<GrabberPtr ()> grabber_factory = 
 		[&device_id]() -> GrabberPtr { return std::make_shared<pcl::OpenNIGrabber>(device_id); };
	
	commonStart(grabber_factory);
}

template<typename RegistrationAlgorithm> void
RegistrationTool<RegistrationAlgorithm>::start(const std::vector<std::string>& clouds)
{
	boost::function<GrabberPtr ()> grabber_factory = 
		[&clouds]() -> GrabberPtr { return std::make_shared<pcl::PCDGrabber<pcl::PointXYZRGBA>>(clouds); };
	
	commonStart(grabber_factory);
}

template<typename RegistrationAlgorithm> void
RegistrationTool<RegistrationAlgorithm>::commonStart(const boost::function<GrabberPtr ()> &grabber_factory)
{
	assert ( grabber_factory );

	if (started_) 
	{
		PCL_WARN("Registration is %s...\n", !finished_ ? "finished" : "running"); return;
	}
	
	if ( root_dir_.empty() || registration_dir_.empty() || (backup_enabled_ && backup_dir_.empty()) )
	{
		PCL_ERROR("Output directory wasn't setup properly ... ERROR\n"); return;	
	} 

	if ( !registration_ ) 
	{
		PCL_ERROR("Registration algorithm wasn't seted... ERROR\n"); return;
	}

	Status device_status = SUCCESS;
	try {
		grabber_ = grabber_factory();		
	} catch (pcl::IOException &e) {
		device_status = DEVICE_NOT_WORKING;
	}
	PCL_INFO("Creating grabber...%s\n", 
			 device_status == SUCCESS ? "OK" : "ERROR");

	if (device_status == SUCCESS) 
	{
		try {
			boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> cb = 
				boost::bind(&RegistrationTool::updateStreamingVisualization, this, _1);
			// grabber_->registerCallback(cb);
		} catch (pcl::IOException &e) {
			device_status = DEVICE_NOT_WORKING;
		}
		PCL_INFO("Setting streaming callback...%s\n", 
				 device_status == SUCCESS ? "OK" : "ERROR");
	}

	if (device_status == SUCCESS) 
	{
		device_status = generator_.setGrabberImplementation(grabber_);
		PCL_INFO("Initializing generator...%s\n", 
				 device_status == SUCCESS ? "OK" : "ERROR");
	}

	if (device_status == SUCCESS) 
	{
		device_status = generator_.startGenerating();
		PCL_INFO("Starting to generate...%s\n",
				 device_status == SUCCESS ? "OK" : "ERROR");
	}

	if ( device_status == SUCCESS )
	{
		PCL_INFO("Opening device...%s\n", "OK");
	} else {
		PCL_ERROR("Opening device...%s\n", "ERROR"); return;
	}

	PCL_INFO("Starting registration... OK\n");
	started_ = true;
}

template<typename RegistrationAlgorithm> void
RegistrationTool<RegistrationAlgorithm>::setRegistrationAlgorithm(const RegistrationAlgorithmPtr &registration)
{
	registration_ = registration;
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
	// update screen after global optimization
	for (int cloud_idx = 0; cloud_idx < visualized_clouds_; ++cloud_idx) 
	{
		updateRegistrationVisualization(cloud_idx);
	}

	PCL_INFO("Total aligned %i / %i clouds.\n\n", visualized_clouds_, 
												  processed_clouds_);
	
	finished_ = true;
}

template<typename RegistrationAlgorithm> void
RegistrationTool<RegistrationAlgorithm>::captureAndRegister()
{
	if (!started_ || finished_)	
	{
		PCL_WARN("Registration is not running...\n"); return;
	}

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);				
	if (generator_.generate(cloud) != SUCCESS)
	{
		PCL_ERROR("Generating cloud... ERROR\n\n"); return;
	}

	PCL_INFO("Registering cloud %i\n", ++processed_clouds_);
	try 
	{	
		if (registration_->proccessCloud(cloud) == SUCCESS) 
		{
			PCL_INFO("Updating visualization...\n");
			updateRegistrationVisualization(visualized_clouds_);
			PCL_INFO("Successfully aligned...\n\n");
			if (backup_enabled_) 
			{
				backup(registration_->getNumClouds() - 1);
			}
		} else {
			PCL_ERROR("Error found. Skipping cloud... ERROR\n\n");
		}
	}
	catch(pcl::UnorganizedPointCloudException &e) 
	{
		PCL_ERROR("Organized cloud is required... ERROR\n\n", processed_clouds_);
	}
}

template<typename RegistrationAlgorithm> void
RegistrationTool<RegistrationAlgorithm>::updateRegistrationVisualization(int cloud_idx)
{
	namespace vis = pcl::visualization;
	assert( 0 <= cloud_idx ); assert(cloud_idx <= visualized_clouds_ );

	std::string cloud_idx_str = std::to_string(cloud_idx);
	if (cloud_idx == visualized_clouds_)
	{
		++visualized_clouds_;

		auto cloud = registration_->getInputCloud(cloud_idx);
		// required to use multiple colors handlers
		pcl::PCLPointCloud2::Ptr blob_cloud(new pcl::PCLPointCloud2);
		pcl::toPCLPointCloud2(*cloud, *blob_cloud);

		mutex_.lock ();
		// color_handler needs to be a Ptr to be used wit PCLPointCLoud2
		vis::PointCloudColorHandler<pcl::PCLPointCloud2>::Ptr color_handler;

	    Eigen::Vector4f origin;
	    Eigen::Quaternionf orientation;		

		color_handler.reset(new vis::PointCloudColorHandlerRGBField<pcl::PCLPointCloud2>(blob_cloud));
		viewer_.addPointCloud(blob_cloud, color_handler, origin, orientation, cloud_idx_str, registration_viewport_);
		
		color_handler.reset(new vis::PointCloudColorHandlerGenericField<pcl::PCLPointCloud2>(blob_cloud, "z"));				
		viewer_.addPointCloud(blob_cloud, color_handler, origin, orientation, cloud_idx_str, registration_viewport_);
	
      	viewer_.setPointCloudRenderingProperties (vis::PCL_VISUALIZER_IMMEDIATE_RENDERING, 1.0, cloud_idx_str);

		mutex_.unlock ();
	} 
	// TODO: usar origin, orientation
	// TODO: aplicar solo cuando cloud_idx < visualized_clouds
	auto PITCH_ROTATION = pcl::getTransformation(0, 0, 0, 0, M_PI, 0);
	auto visual_pose = PITCH_ROTATION * registration_->getTransformation(cloud_idx);
	mutex_.lock ();
	viewer_.updatePointCloudPose(cloud_idx_str, visual_pose);
	mutex_.unlock ();		
}

template<typename RegistrationAlgorithm> void
RegistrationTool<RegistrationAlgorithm>::updateStreamingVisualization(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
{
	mutex_.lock ();
	viewer_.removePointCloud("stream", stream_viewport_);
	viewer_.addPointCloud(cloud, "stream", stream_viewport_);	
	mutex_.unlock ();
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
RegistrationTool<RegistrationAlgorithm>::checkpoint()
{	
	assert(!root_dir_.empty()); assert(!registration_dir_.empty()); 

	for (int cloud_idx = 0; cloud_idx < registration_->getNumClouds(); ++cloud_idx)
	{	
		pcl::PointCloud<pcl::PointXYZRGBA> aligned_cloud;

		pcl::transformPointCloud(*registration_->getInputCloud(cloud_idx), 
						    	 aligned_cloud, 
								 registration_->getTransformation(cloud_idx));
		
		auto cloud_path = cloudPathFormat(registration_dir_, cloud_idx);
		pcl::io::savePCDFileBinaryCompressed (cloud_path.c_str(), aligned_cloud);
		PCL_INFO("Saving aligned cloud at : %s\n", cloud_path.c_str());
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
		pcl::io::savePCDFileBinaryCompressed (cloud_path.c_str(), 
											  *registration_->getInputCloud(cloud_idx));
		PCL_INFO("Saving backup cloud at : %s\n", cloud_path.c_str());
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
			captureAndRegister();
		} else if (key == "p" || key == "P")
		{
			finish();
		} else if (key == "s" || key == "S")
		{
			checkpoint();
		}
	}
}

template<typename RegistrationAlgorithm> void
RegistrationTool<RegistrationAlgorithm>::run()
{
	if (started_ && !finished_)
	{
		PCL_INFO("Running visualization... OK\n");
		viewer_.createInteractor();
	}
	while (started_ && !finished_)
	{
		viewer_.spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::milliseconds (100));
	}
}

#endif // REGISTRATION_TOOL_HPP