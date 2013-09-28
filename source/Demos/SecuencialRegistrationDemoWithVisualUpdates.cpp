
#include <vector>
#include <string>
#include <memory>

#include <pcl/common/common.h>
#include <pcl/common/file_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_grabber.h>	

#include <boost/thread/thread.hpp>

#include "Utils/Utils.h"
#include "IO/CloudGenerator.h"
#include "GlobalRegistration/GlobalRegistration.h"
#include "GlobalRegistration/GlobalRegistrationFactory.h"



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
	if (argc < 3)
	{
		PCL_ERROR("use: program cloud_file0 .. cloud_fileN. Clouds must be organized\n");
		return -1;
	} 

	std::vector<string> filenames(argv + 1, argv + argc);

	CloudGenerator generator;
	generator.setGrabberImplementation(std::make_shared<PCDGrabber<PointXYZRGBA>>(filenames));
	generator.startGenerating();

	GlobalRegistration<PointXYZRGBA, PointWithScale, SURFSignature128>::Ptr registration;
	GlobalRegistrationFactory().create(registration);
	registration->setInliersThreshold(30);
	
	int cloud_idx = 0, n_clouds = argc - 1, clouds_aligned = 0;

	Eigen::Affine3f PITCH_ROT = pcl::getTransformation(0, 0, 0, 0, 3.14159265, 0); 

	PCLVisualizer viewer;
	while (!viewer.wasStopped())
	{
		if (cloud_idx < n_clouds) 
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

				PointCloudColorHandlerRGBField<PointXYZRGBA> handler (downsampled_cloud);
				viewer.addPointCloud<PointXYZRGBA>(downsampled_cloud, handler, to_string(clouds_aligned));
				viewer.updatePointCloudPose(to_string(clouds_aligned), 
											PITCH_ROT * registration->getTransformation(clouds_aligned));
				
				++clouds_aligned;
			} else {
				PCL_ERROR("error registering cloud %i. error : %s. skip cloud.\n", cloud_idx, 
				      	  st == NOT_ENOUGH_FEATURES ? "NOT_ENOUGH_FEATURES" : "NOT_ENOUGH_INLIERS");
			}

			++cloud_idx;

			if (cloud_idx == n_clouds)
			{
				if (clouds_aligned > 1) registration->globalOptimize();

				for (int idx = 0; idx < clouds_aligned; ++idx)
			 		viewer.updatePointCloudPose(to_string(idx), 
			 									PITCH_ROT * registration->getTransformation(idx));

				PCL_INFO("successfully aligned %i / %i clouds.\n\n", clouds_aligned, n_clouds);
				
				generator.stopGenerating();
			}

			viewer.spinOnce (1);
		} else {
			viewer.spinOnce (100);
			boost::this_thread::sleep (boost::posix_time::milliseconds (100));
		}
	}

	return 0;
}