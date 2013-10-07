
#ifndef REGISTRATION_TOOL_HPP
#define REGISTRATION_TOOL_HPP

#include <vector>
#include <string>

#include <boost/thread/thread.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/filesystem.hpp> 

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
#include <pcl/filters/voxel_grid.h>

#include "Common/Status.h"
#include "Utils/Utils.h"
#include "IO/CloudGenerator.h"
#include "Tools/RegistrationTool.h"

template<typename RegistrationAlgorithm>
RegistrationTool<RegistrationAlgorithm>::RegistrationTool(bool downsample)
	: processed_clouds_(0)
	, visualized_clouds_(0)
	, started_(false) 
	, finished_(false)
	, grabber_()
	, generator_()
	, registration_()
	, downsample_enabled_(downsample)
	, viewer_("Visual registration tool", false)
	, stream_viewport_()
	, registration_viewport_()
{
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
	assert( 0 <= cloud_idx ); assert(cloud_idx <= visualized_clouds_ );

	if (cloud_idx == visualized_clouds_)
	{
		++visualized_clouds_;

		auto cloud = registration_->getInputCloud(cloud_idx);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr visual_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
		if (downsample_enabled_)
		{
			float leaf_size = 0.005;
			pcl::VoxelGrid<pcl::PointXYZRGBA> downsampler;
			downsampler.setLeafSize (leaf_size, leaf_size, leaf_size);
			downsampler.setInputCloud (cloud);    			
			downsampler.filter (*visual_cloud);
		} else {
			visual_cloud = cloud;
		}

		mutex_.lock ();
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgba_handler (visual_cloud);
		viewer_.addPointCloud<pcl::PointXYZRGBA>(visual_cloud, rgba_handler, std::to_string(cloud_idx), registration_viewport_);
		// pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZRGBA> z_handler(visual_cloud, "z");				
		// viewer_.addPointCloud<pcl::PointXYZRGBA>(visual_cloud, z_handler, std::to_string(cloud_idx), registration_viewport_);
		mutex_.unlock ();
	} 
	// TODO: usar origin, orientation
	// TODO: aplicar solo cuando cloud_idx < visualized_clouds
	auto PITCH_ROTATION = pcl::getTransformation(0, 0, 0, 0, M_PI, 0);
	auto visual_pose = PITCH_ROTATION * registration_->getTransformation(cloud_idx);
	mutex_.lock ();
	viewer_.updatePointCloudPose(std::to_string(cloud_idx), visual_pose);
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

template<typename RegistrationAlgorithm> void
RegistrationTool<RegistrationAlgorithm>::saveAlignedClouds(const std::string &directory)
{	
	namespace fs = boost::filesystem;
	if (registration_->getNumClouds() == 0)
	{
		PCL_INFO("There are not clouds to save.\n"); return;
	}

	fs::path directory_path(directory);

	if (fs::exists(directory_path) && !fs::is_directory(directory_path))
	{
		PCL_ERROR("%s isn't a directory path.\n", directory.c_str()); return;
	} else 
	{
		try{
			fs::create_directories(directory_path);
		} catch(boost::filesystem3::filesystem_error &e) {
			PCL_ERROR("Can't create %s directory.\n", directory.c_str()); return;
		}
	} 

	pcl::PointCloud<pcl::PointXYZRGBA> aligned_cloud;
	for (int cloud_idx = 0; cloud_idx < registration_->getNumClouds(); ++cloud_idx)
	{			
		pcl::transformPointCloud(*registration_->getInputCloud(cloud_idx), 
						    	 aligned_cloud, 
								 registration_->getTransformation(cloud_idx));
		
		auto file_path = directory_path / (std::to_string(cloud_idx) + ".pcd");

		pcl::io::savePCDFileBinaryCompressed (file_path.native(), aligned_cloud);
		PCL_INFO("Saving cloud at : %s\n", file_path.c_str());
	}
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