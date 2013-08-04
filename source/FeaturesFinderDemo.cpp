
#include <iostream>
#include <vector>

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/features2d/features2d.hpp>
#include <boost/thread/thread.hpp>

#include "Features.h"
#include "OrganizedFeaturesFinder.h"
#include "Utils.h"


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

	// TODO: utilizar mejores paramatros
	// fx, fy son sacados de una kinect de internet, cx, cy defaults 
	double fx = 529.215080, fy = 525.563936;
	double cx = (cloud->width - 1.f) / 2.f, cy = (cloud->height - 1.f) / 2.f;

	// ORB (default parameters)
	OrganizedFeaturesFinder featuresFinder(fx, fy, cx, cy,
										   FeatureDetector::create("ORB"),
										   DescriptorExtractor::create("ORB"));
	featuresFinder.setCloud(cloud);

	PointCloud<BaseKeypoint>::Ptr keypoints0(new PointCloud<features::BaseKeypoint>);
	PointCloud<BaseKeypoint>::Ptr tmp_keypoints0(new PointCloud<features::BaseKeypoint>);
	PointCloud<BaseKeypoint>::Ptr keypoints1(new PointCloud<features::BaseKeypoint>);
	
	Descriptors descriptors0, descriptors1;
	
	featuresFinder.computeKeypoints(*tmp_keypoints0);
	featuresFinder.computeDescriptors(*tmp_keypoints0, descriptors0, *keypoints0);
	
	featuresFinder.computeKeypointsAndDescriptors(*keypoints1, descriptors1);
	
	// visualize features
	PCLVisualizer viewer;
 	viewer.initCameraParameters();

	displayKeypoints(cloud, keypoints0, viewer, PointRGB(255, 0, 0), 3, "c0", "k0-blue");
	displayKeypoints(cloud, keypoints1, viewer, PointRGB(0, 0, 255), 3, "c1", "k1-red");

 	// displays cloud until a key is pressed
	while (!viewer.wasStopped())
	{
		viewer.spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}

	return 0;
}