
#include <iostream>
#include <vector>
#include <memory>

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
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

// TODO: bug auto opencvFinder = make_shared<OpenCVFeaturesFinder<PointInT, KeypointT>>("ORB", "ORB");

template<typename PointInT, typename KeypointT> 
typename OpenCVFeaturesFinder<PointInT, KeypointT>::Ptr 
createOpenCVFeaturesFinder(const char* feature_detector, const char* descriptor_extractor) 
{
	// ORB (default parameters)
	auto opencvFinder = make_shared<OpenCVFeaturesFinder<PointInT, KeypointT>>(); 
	opencvFinder->setKeypointDetector(FeatureDetector::create(feature_detector));
	opencvFinder->setDescriptorExtractor(DescriptorExtractor::create(descriptor_extractor));

	return opencvFinder;
}

template<typename PointInT, typename KeypointT> 
typename FPFHEstimationFeaturesFinder<PointInT, KeypointT>::Ptr 
createPCLFeaturesFinder()
{
	float normal_radius = 0.025, fphp_radius = 0.025;

	auto pclFinder = make_shared<FPFHEstimationFeaturesFinder<PointInT, KeypointT>>();
	
	pclFinder->setRadiusSearchNormals(normal_radius);
	pclFinder->setRadiusSearchFPFH(fphp_radius);
	return pclFinder;
}


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
	cv::initModule_nonfree();

	// ORB (default parameters)
	auto opencvFinder = createOpenCVFeaturesFinder<PointXYZRGBA, PointWithScale>("ORB", "ORB"); 
	opencvFinder->setInputCloud(cloud);
	
	auto pclFinder = createPCLFeaturesFinder<PointXYZRGBA, PointWithScale>();
	pclFinder->setInputCloud(cloud);

	PointCloud<PointWithScale>::Ptr keypoints1(new PointCloud<PointWithScale>);
	PointCloud<PointWithScale>::Ptr tmp_keypoints1(new PointCloud<PointWithScale>);	
	Descriptors descriptors1;
	
	opencvFinder->computeKeypoints(*tmp_keypoints1);
	// pclFinder->computeDescriptors(*tmp_keypoints1, descriptors1, *keypoints1);

	cout << "# tmp keypoints : " << tmp_keypoints1->size() << endl;	
	cout << "# keypoints : " << keypoints1->size() << endl;
	cout << "# descriptors : " << descriptors1.rows << endl;

	// visualize features
	PCLVisualizer viewer;
 	viewer.initCameraParameters();

	displayKeypoints<PointXYZRGBA, PointWithScale>(cloud, tmp_keypoints1, viewer, PointRGB(0, 255, 0), 3);

 	// displays cloud until a key is pressed
	while (!viewer.wasStopped())
	{
		viewer.spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}

	return 0;
}