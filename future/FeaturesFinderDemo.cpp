
#include <iostream>
#include <memory>

#include <pcl/common/common.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/features2d/features2d.hpp>
#include <boost/thread/thread.hpp>

#include "features/descriptor_types.h"
#include "features/FeaturesFinder.h"
#include "features/OpenCVBinaryFeaturesFinder.h"
#include "features/Utils.h"

using namespace pcl;
using namespace pcl::io;
using namespace pcl::visualization;

FeaturesFinder<PointXYZRGBA, PointWithScale, CustomSizeBinaryDescriptor>::Ptr 
createORBFinder() 
{
	// ORB (default parameters)
	auto finder = std::make_shared<OpenCVBinaryFeaturesFinder<PointXYZRGBA, PointWithScale>>(); 
	finder->setKeypointDetector(cv::FeatureDetector::create("ORB"));
	finder->setDescriptorExtractor(cv::DescriptorExtractor::create("ORB"));

	return finder;
}

int main(int argc, char* argv[])
{
	if (argc != 2)
	{
		PCL_ERROR("use: program cloud_file\n");
		return -1;
	}

	PointCloud<PointXYZRGBA>::Ptr cloud (new PointCloud<PointXYZRGBA>);
	if (loadPCDFile(argv[1], *cloud) == -1)
	{
		PCL_ERROR("coudn't read file %s\n", argv[1]);
		return -1;
	}

	auto finder = createORBFinder(); 
	finder->setInputCloud(cloud);
	
	PointCloud<PointWithScale>::Ptr initial_keypoints (new PointCloud<PointWithScale>);
	PointCloud<PointWithScale>::Ptr keypoints (new PointCloud<PointWithScale>);	
	PointCloud<CustomSizeBinaryDescriptor>::Ptr descriptors (new PointCloud<CustomSizeBinaryDescriptor>);

	finder->computeKeypoints(*initial_keypoints);
	finder->computeDescriptors(*initial_keypoints, *descriptors, *keypoints);

	PCL_INFO("# initial keypoints: %i\n", initial_keypoints->size() );	
	PCL_INFO("# keypoints with descriptor : %i\n", keypoints->size() );
	PCL_INFO("# descriptors : %i\n", descriptors->size() );

	// visualize features
	PCLVisualizer viewer;
 	viewer.initCameraParameters();

	displayKeypoints<PointXYZRGBA, PointWithScale>(cloud, keypoints, viewer, PointRGB(0, 255, 0), 3);

 	// displays cloud until a key is pressed
	while (!viewer.wasStopped())
	{
		viewer.spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}

	return 0;
}