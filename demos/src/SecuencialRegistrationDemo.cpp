
#include <vector>
#include <string>
#include <memory>

#include <pcl/common/common.h>
#include <pcl/common/file_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>

#include <boost/thread/thread.hpp>
#include <Eigen/StdVector> // required to use std::vector with Eigen::affine3f
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Affine3f)

#include "features/Utils.h"
#include "features/GlobalRegistration.h"
#include "features/GlobalRegistrationFactory.h"

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

	std::vector<PointCloud<PointXYZRGBA>::Ptr> clouds;
	
	GlobalRegistration<PointXYZRGBA, PointWithScale, SURFSignature128>::Ptr registration;
	GlobalRegistrationFactory().create(registration);
	registration->setInliersThreshold(30);

	int n_clouds = argc - 1;
	for (int cloud_idx = 0, name_idx = 1; cloud_idx < n_clouds ; ++cloud_idx, ++name_idx)
	{
		auto &filename = argv[name_idx];
		string shortname = pcl::getFilenameWithoutPath(filename);

		PointCloud<PointXYZRGBA>::Ptr cloud (new PointCloud<PointXYZRGBA>);
		
		if (loadPCDFile(filename, *cloud) == -1)
		{
			PCL_ERROR("coudn't read file %s. registration aborted.\n", shortname.c_str());
			return -1;
		}
		if (!cloud->isOrganized()) 
		{
			PCL_ERROR("%s is not organized. registration aborted\n", shortname.c_str());
			return -1;
		}

		PCL_INFO("processing cloud %i / %i \n", cloud_idx + 1, n_clouds);
		
		Status st = registration->proccessCloud(cloud); 
		if (st == SUCCESS) 
		{
			PCL_INFO("successfully registered %s.\n\n", shortname.c_str());

			// downsamples cloud to save space and reduce visualizer
			PointCloud<PointXYZRGBA>::Ptr downsampled_cloud (new PointCloud<PointXYZRGBA>);
			downsample(cloud, downsampled_cloud, 0.005);
			
			clouds.push_back(downsampled_cloud);
		} else {
			PCL_ERROR("error registering %s. %s. skip cloud.\n", shortname.c_str(), 
				      st == NOT_ENOUGH_FEATURES ? "NOT_ENOUGH_FEATURES" : "NOT_ENOUGH_INLIERS");
		}
	}

	if (clouds.size() > 1) registration->globalOptimize();

	PCL_INFO("successfully aligned %i / %i clouds.\n\n", clouds.size(), n_clouds);

	PointCloud<PointXYZRGBA>::Ptr final_cloud (new PointCloud<PointXYZRGBA>);
	PointCloud<PointXYZRGBA>::Ptr concateneted_cloud (new PointCloud<PointXYZRGBA>);

	std::vector<Eigen::Affine3f> transformations;
	registration->getTransformations(transformations);

	for (int idx = 0; idx < clouds.size(); ++idx) 
	{
		const auto& cloud = clouds[idx];
		PointCloud<PointXYZRGBA> registered_cloud;
		transformPointCloud<PointXYZRGBA, float>(*cloud, registered_cloud, transformations[idx]);
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