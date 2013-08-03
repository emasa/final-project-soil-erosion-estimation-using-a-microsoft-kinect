
#include <iostream>
#include <vector>

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/features2d/features2d.hpp>
#include <boost/thread/thread.hpp>

#include "../include/Features.h"
#include "OrganizedFeaturesFinder.h"

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::visualization;
using namespace cv;
using namespace features;

int main(int argc, char* argv[])
{
	if (argc != 2)
	{
		cerr << "use: program cloud_file" << endl;
		return -1;
	}

	PointCloud<PointXYZRGBA>::Ptr cloud (new PointCloud<PointXYZRGBA>);
	if (loadPCDFile(argv[1], *cloud) == -1)
	{
		cerr << "coudn't read file " << argv[1] << endl;
		return -1;
	}

	// ORB (default parameters)
	// OrganizedFeaturesFinder featuresFinder(FeatureDetector::create("ORB"),
	// 									   	  DescriptorExtractor::create("ORB"));
	OrganizedFeaturesFinder featuresFinder;
	featuresFinder.setKeypointDetector(FeatureDetector::create("ORB"));	
	featuresFinder.setDescriptorExtractor(DescriptorExtractor::create("ORB"));
	featuresFinder.setCloud(cloud);

	// features
	PointCloud<BaseKeypoint>::Ptr keypoints(new PointCloud<features::BaseKeypoint>);
	features::Descriptors descriptors;

	// find features
	featuresFinder.find(*keypoints, descriptors);

	// visualize features
	PCLVisualizer viewer;

	PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> cloud_handler(cloud);
	viewer.addPointCloud(cloud, cloud_handler, "cloud");

	PointCloudColorHandlerCustom<BaseKeypoint> keypoints_handler(keypoints, 0, 255, 0);
	viewer.addPointCloud(keypoints, keypoints_handler, "keypoints");
 	viewer.setPointCloudRenderingProperties (PCL_VISUALIZER_POINT_SIZE, 3, "keypoints");

 	vector<Camera> cameras;
 	viewer.getCameras(cameras);

	while (!viewer.wasStopped())
	{
		viewer.spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}

	return 0;
}