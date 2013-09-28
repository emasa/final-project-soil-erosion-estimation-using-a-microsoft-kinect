
#include <vector>
#include <string>
#include <memory>

#include <pcl/common/common.h>
#include <pcl/common/file_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/pcd_grabber.h>	
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>

#include <boost/thread/thread.hpp>

#include "Utils/Utils.h"
#include "GlobalRegistration/GlobalRegistration.h"
#include "GlobalRegistration/GlobalRegistrationFactory.h"
#include "IO/CloudGenerator.h"

using namespace std;
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

int main(int argc, char* argv[])
{
	// pcl::console::setVerbosityLevel(pcl::console::L_DEBUG);
	if (argc < 3)
	{
		PCL_ERROR("use: program cloud_file0 .. cloud_fileN. Clouds must be organized\n");
		return -1;
	} 

	std::vector<string> filenames(argv + 1, argv + argc);

	CloudGenerator generator;
	generator.setGrabberImplementation(std::make_shared<PCDGrabber<PointXYZRGBA>>(filenames));
	generator.startGenerating();

	std::vector<PointCloud<PointXYZRGBA>::Ptr> clouds;
	
	GlobalRegistration<PointXYZRGBA, PointWithScale, SURFSignature128>::Ptr registration;
	GlobalRegistrationFactory().create(registration);
	registration->setInliersThreshold(30);

	int n_clouds = argc - 1;
	for (int cloud_idx = 0; cloud_idx < n_clouds ; ++cloud_idx)
	{
		PointCloud<PointXYZRGBA>::Ptr cloud (new PointCloud<PointXYZRGBA>);
		
		generator.generate(cloud);

		if (!cloud->isOrganized()) 
		{
			PCL_ERROR("cloud %i is not organized. registration aborted\n", cloud_idx);
			return -1;
		}

		PCL_INFO("processing cloud %i / %i \n", cloud_idx + 1, n_clouds);
		
		Status st = registration->proccessCloud(cloud); 
		if (st == SUCCESS) 
		{
			PCL_INFO("successfully registered cloud %i.\n\n", cloud_idx);

			// downsamples cloud to save space and reduce visualizer
			PointCloud<PointXYZRGBA>::Ptr downsampled_cloud (new PointCloud<PointXYZRGBA>);
			downsample(cloud, downsampled_cloud, 0.005);
			
			clouds.push_back(downsampled_cloud);
		} else {
			PCL_ERROR("error registering cloud %i. error : %s. skip cloud.\n", cloud_idx, 
				      st == NOT_ENOUGH_FEATURES ? "NOT_ENOUGH_FEATURES" : "NOT_ENOUGH_INLIERS");
		}
	}

	generator.stopGenerating();

	if (clouds.size() > 1) registration->globalOptimize();

	PCL_INFO("successfully aligned %i / %i clouds.\n\n", clouds.size(), n_clouds);

	PointCloud<PointXYZRGBA>::Ptr final_cloud (new PointCloud<PointXYZRGBA>);
	PointCloud<PointXYZRGBA>::Ptr concateneted_cloud (new PointCloud<PointXYZRGBA>);

	for (int idx = 0; idx < clouds.size(); ++idx) 
	{
		const auto& cloud = clouds[idx];
		PointCloud<PointXYZRGBA> registered_cloud;
		transformPointCloud<PointXYZRGBA, float>(*cloud, registered_cloud, registration->getTransformation(idx));
 		*concateneted_cloud += registered_cloud;
 	}

 	final_cloud = concateneted_cloud;
	PCL_INFO("downsampled cloud %i points\n", final_cloud->size());

	// visualization setup
	PCLVisualizer viewer;
	// add rgb cloud    
    PointCloudColorHandlerRGBField<PointXYZRGBA> handler (final_cloud);
	viewer.addPointCloud<PointXYZRGBA>(final_cloud, handler, "cloud");
	// improves point of view
    viewer.updatePointCloudPose("cloud", viewer.getViewerPose());
 	// displays cloud until a key is pressed
	while (!viewer.wasStopped())
	{
		viewer.spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}

	return 0;
}