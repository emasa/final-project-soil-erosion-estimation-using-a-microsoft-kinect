
#include <iostream>
#include <vector>

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/features2d/features2d.hpp>
#include <boost/thread/thread.hpp>

#include "Utils.h"
#include "Features.h"

#include "FeaturesFinder.h"
#include "OpenCVFeaturesFinder.h"
#include "PCLFeaturesFinder.h"

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
	auto opencvFinder = make_shared<OpenCVFeaturesFinder<PointXYZRGBA, PointWithScale>>(); 
	opencvFinder->setCameraIntrinsics(fx, fy, cx, cy);
	opencvFinder->setKeypointDetector(FeatureDetector::create("ORB"));
	opencvFinder->setDescriptorExtractor(DescriptorExtractor::create("ORB"));
	opencvFinder->setInputCloud(cloud);

	// FeaturesFinder<PointXYZRGBA, PointWithScale>::Ptr finder = opencvFinder;
	// FPFH 
	float normal_radius = 0.025, fphp_radius = normal_radius;

	auto pclFinder = make_shared<FPFHEstimationFeaturesFinder<PointXYZRGBA, PointWithScale>>();
	pclFinder->setRadiusSearchNormals(normal_radius);
	pclFinder->setRadiusSearchFPFH(fphp_radius);

	FeaturesFinder<PointXYZRGBA, PointWithScale>::Ptr finder = pclFinder;
	finder->setInputCloud(cloud);

	PointCloud<PointWithScale>::Ptr keypoints1(new PointCloud<PointWithScale>);
	PointCloud<PointWithScale>::Ptr tmp_keypoints1(new PointCloud<PointWithScale>);	
	Descriptors descriptors1;
	
	opencvFinder->computeKeypoints(*tmp_keypoints1);
	finder->computeDescriptors(*tmp_keypoints1, descriptors1, *keypoints1);

	cout << "# tmp keypoints : " << tmp_keypoints1->size() << endl;	
	cout << "# keypoints : " << keypoints1->size() << endl;
	cout << "# descriptors : " << descriptors1.rows << endl;

	// visualize features
	PCLVisualizer viewer;
 	viewer.initCameraParameters();

	displayKeypoints<PointWithScale>(cloud, keypoints1, viewer, PointRGB(0, 255, 0));

 	// displays cloud until a key is pressed
	while (!viewer.wasStopped())
	{
		viewer.spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}

	return 0;
}