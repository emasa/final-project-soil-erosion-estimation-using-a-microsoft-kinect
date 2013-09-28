
#include <vector>
#include <string>

#include <boost/thread/thread.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/filesystem.hpp> 

#include <pcl/common/common.h>
#include <pcl/common/file_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>

#include "Common/Status.h"
#include "Utils/Utils.h"
#include "IO/CloudGenerator.h"
#include "GlobalRegistration/GlobalRegistration.h"
#include "GlobalRegistration/GlobalRegistrationFactory.h"
#include "Tools/RegistrationTool.h"

using namespace std;
using namespace boost::filesystem;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::visualization;

void
downsample(const PointCloud<PointXYZRGBA>::Ptr& cloud, 
		   PointCloud<PointXYZRGBA>::Ptr& downsampled_cloud, 
		   float leaf_size)
{
 	VoxelGrid<PointXYZRGBA> grid;
 	// create 3d box with size leaf_size x leaf_size x leaf_size
 	// and replace its values with one weighted value
    grid.setLeafSize (leaf_size, leaf_size, leaf_size);
    grid.setInputCloud (cloud);    
    grid.filter (*downsampled_cloud);
}

RegistrationTool::RegistrationTool()
: processed_clouds_(0)
, visualized_clouds_(0)
, downsample_enabled_(true)
, generator_()
, viewer_("Registration tool")
, registration_()
, registration_running_(false)
{}

void
RegistrationTool::initRegistering(const vector<string> &filenames)
{
	if (registration_running_) 
	{
		PCL_WARN("Registration is already running...\n"); return;
	}

	// start generating clouds
	if (generator_.create(filenames) == SUCCESS && 
		generator_.startGenerating() == SUCCESS)
	{
		PCL_INFO("Opening device... OK\n");
	} else {
		PCL_ERROR("The device cannot be open... ERROR\n"); return;
	}

	//init registration algorithm
	registration_.reset();
	GlobalRegistrationFactory().create(registration_);
	// clear visualization	
	viewer_.removeAllPointClouds();
	// reset counters
	processed_clouds_  = 0;
	visualized_clouds_ = 0;

	PCL_INFO("Starting registration... OK\n");
	registration_running_ = true;
}

void 
RegistrationTool::stopRegistering()
{
	if (!registration_running_) 
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
		updateVisualization(cloud_idx);
	}

	PCL_INFO("Total aligned %i / %i clouds.\n\n", visualized_clouds_, 
												  processed_clouds_);
	
	registration_running_ = false;
}

void
RegistrationTool::registerNewCloud()
{
	if (!registration_running_) 
	{
		PCL_WARN("Registration is not running...\n"); return;
	}

	PointCloud<PointXYZRGBA>::Ptr cloud (new PointCloud<PointXYZRGBA>);				
	if (generator_.generate(cloud) != SUCCESS)
	{
		PCL_ERROR("Generating cloud... ERROR\n\n"); return;
	}

	PCL_INFO("Registering cloud %i\n", ++processed_clouds_);
	try 
	{	
		if (registration_->proccessCloud(cloud) == SUCCESS) 
		{
			PCL_INFO("Successfully aligned...\n\n");
			updateVisualization(visualized_clouds_);
		} else {
			PCL_ERROR("Error found. Skipping cloud... ERROR\n\n");
		}
	}
	catch(UnorganizedPointCloudException &e) 
	{
		PCL_ERROR("Organized cloud is required... ERROR\n\n", processed_clouds_);
	}
}

void
RegistrationTool::updateVisualization(int cloud_idx)
{
	assert( 0 <= cloud_idx ); assert(cloud_idx <= visualized_clouds_ );

	if (cloud_idx == visualized_clouds_)
	{
		++visualized_clouds_;

		auto cloud = registration_->getInputCloud(cloud_idx);
		PointCloud<PointXYZRGBA>::Ptr visual_cloud (new PointCloud<PointXYZRGBA>);
		if (downsample_enabled_)
		{
			downsample(cloud, visual_cloud, 0.005);
		} else {
			visual_cloud = cloud;
		}

		PointCloudColorHandlerRGBField<PointXYZRGBA> handler (visual_cloud);		
		viewer_.addPointCloud<PointXYZRGBA>(visual_cloud, handler, to_string(cloud_idx));
	}
	
	auto PITCH_ROTATION = getTransformation(0, 0, 0, 0, M_PI, 0);
	auto visual_pose = PITCH_ROTATION * registration_->getTransformation(cloud_idx);
	viewer_.updatePointCloudPose(to_string(cloud_idx), visual_pose);
}

void 
RegistrationTool::saveAlignedClouds(const string &directory)
{	
	if (registration_->getNumClouds() == 0)
	{
		PCL_INFO("There are not clouds to save.\n");
		return;
	}

	path directory_path(directory);

	if (exists(directory_path) && !is_directory(directory_path))
	{
		PCL_ERROR("%s isn't a directory path.\n", directory.c_str()); return;
	} else {
		try{
			create_directories(directory_path);
		} catch(boost::filesystem3::filesystem_error &e) {
			PCL_ERROR("Can't create %s directory.\n", directory.c_str()); return;
		}
	} 

	PointCloud<PointXYZRGBA> aligned_cloud;
	for (int cloud_idx = 0; cloud_idx < registration_->getNumClouds(); ++cloud_idx)
	{			
		transformPointCloud(*registration_->getInputCloud(cloud_idx), 
						    aligned_cloud, 
							registration_->getTransformation(cloud_idx));
		
		auto file_path = directory_path / (to_string(cloud_idx) + ".pcd");

		savePCDFile (file_path.native(), aligned_cloud, true); // binary mode
		PCL_INFO("Saving cloud at : %s\n", file_path.c_str());
	}
}

void 
RegistrationTool::saveAlignedClouds()
{
	if (registration_->getNumClouds() == 0)
	{
		PCL_INFO("There are not clouds to save.\n"); return;
	}

	if (registration_running_)
	{
		string answer;
		while(answer != "YES" && answer != "NO") 
		{
			PCL_INFO("Registration is running yet... Continue saving clouds ? YES / NO : \n");
			std::cin >> answer;
		}
		if (answer == "NO") return;
	}

	PCL_INFO("Enter the directory path : \n");
	string directory; cin >> directory;
	
	saveAlignedClouds(directory);
}

void
RegistrationTool::keyboardManager(const KeyboardEvent &event)
{
	if (event.keyDown())
	{
		string key = event.getKeySym();
		if (key == "space")
		{
			registerNewCloud();
		} else if (key == "p" || key == "P")
		{
			stopRegistering();
		} else if (key == "a" || key == "A")
		{
			saveAlignedClouds();
		}
	}
}

void 
RegistrationTool::run()
{
	auto cb = boost::bind (&RegistrationTool::keyboardManager, this, _1);
	viewer_.registerKeyboardCallback (cb);

	while (!viewer_.wasStopped())
	{
		viewer_.spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::milliseconds (100));
	}
}