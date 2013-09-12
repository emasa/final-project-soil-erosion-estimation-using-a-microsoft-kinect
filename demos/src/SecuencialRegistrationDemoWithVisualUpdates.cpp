
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

bool
loadPointCloud(const string& filename, PointCloud<PointXYZRGBA> &cloud)
{
	string shortname = pcl::getFilenameWithoutPath(filename);
	
	if (loadPCDFile(filename, cloud) == -1)
	{
		PCL_ERROR("coudn't read file %s. registration aborted.\n", shortname.c_str());
		return false;
	}
	if (!cloud.isOrganized()) 
	{
		PCL_ERROR("%s is not organized. registration aborted\n", shortname.c_str());
		return false;
	}

	return true;
}


int main(int argc, char* argv[])
{
	if (argc < 3)
	{
		PCL_ERROR("use: program cloud_file0 .. cloud_fileN. Clouds must be organized\n");
		return -1;
	} 

	std::vector<string> filenames(argv + 1, argv + argc);

	GlobalRegistration<PointXYZRGBA, PointWithScale, SURFSignature128>::Ptr registration;
	GlobalRegistrationFactory().create(registration);
	registration->setInliersThreshold(30);
	
	int cloud_idx = 0, n_clouds = argc - 1, clouds_aligned = 0;

	float PI = 3.14159265; // rotacion alrededor del eje Y
	Eigen::Affine3f PITCH_ROT = pcl::getTransformation(0, 0, 0, 0, PI, 0); 

	PCLVisualizer viewer;
	while (!viewer.wasStopped())
	{
		if (!filenames.empty()) 
		{
			string filename = filenames.back(); filenames.pop_back();	
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

				std::vector<Eigen::Affine3f> transformations;
				registration->getTransformations(transformations);

				PointCloudColorHandlerRGBField<PointXYZRGBA> handler (downsampled_cloud);
				viewer.addPointCloud<PointXYZRGBA>(downsampled_cloud, handler, to_string(cloud_idx));
				viewer.updatePointCloudPose(to_string(cloud_idx), PITCH_ROT * transformations[cloud_idx]);
				
				++clouds_aligned;
			} else {
				PCL_ERROR("error registering %s. %s. skip cloud.\n", shortname.c_str(), 
					      st == NOT_ENOUGH_FEATURES ? "NOT_ENOUGH_FEATURES" : "NOT_ENOUGH_INLIERS");
			}

			++cloud_idx;

			if (cloud_idx == n_clouds)
			{
				if (clouds_aligned > 1) 
				{
					registration->globalOptimize();
				}

				std::vector<Eigen::Affine3f> transformations;
				registration->getTransformations(transformations);

				for (int idx = 0; idx < transformations.size(); ++idx) 
				{
			 		viewer.updatePointCloudPose(to_string(idx), PITCH_ROT * transformations[idx]);
			 	}

				PCL_INFO("successfully aligned %i / %i clouds.\n\n", clouds_aligned, n_clouds);
			}

			viewer.spinOnce (1);
		} else {
			viewer.spinOnce (100);
			boost::this_thread::sleep (boost::posix_time::milliseconds (100));
		}
	}

	return 0;
}