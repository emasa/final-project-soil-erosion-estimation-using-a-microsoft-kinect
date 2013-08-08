
#include <iostream>
#include <vector>
#include <memory>

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

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

template<typename PointInT, typename KeypointT> 
typename OpenCVFeaturesFinder<PointInT, KeypointT>::Ptr 
createOpenCVFeaturesFinder(const char* feature_detector, const char* descriptor_extractor) 
{
	// TODO: utilizar mejores paramatros
	// fx, fy son sacados de una kinect de internet, cx, cy defaults 
	double fx = 529.215080, fy = 525.563936;
	double cx = (480 - 1.f) / 2.f, cy = (640 - 1.f) / 2.f;

	// ORB (default parameters)
	auto opencvFinder = make_shared<OpenCVFeaturesFinder<PointInT, KeypointT>>(); 
	opencvFinder->setCameraIntrinsics(fx, fy, cx, cy);
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
	if (argc != 3)
	{
		cerr << "use: program cloud_file0 cloud_file1" << endl;
		return -1;
	} 

	int N = argc - 1;

	vector<PointCloud<PointXYZRGBA>::Ptr> vec_cloud;
	
	for (int idx = 1; idx <= N ; ++idx)
	{
		PointCloud<PointXYZRGBA>::Ptr cloud (new PointCloud<PointXYZRGBA>);
		if (loadPCDFile(argv[idx], *cloud) == -1)
		{
			cerr << "coudn't read file " << argv[idx] << endl;
			return -1;
		}
		vec_cloud.push_back(cloud);
	}
	cv::initModule_nonfree();
	
	auto opencvFinder = createOpenCVFeaturesFinder<PointXYZRGBA, PointWithScale>("SIFT", "ORB");
	auto pclFinder = createPCLFeaturesFinder<PointXYZRGBA, PointWithScale>();

	vector<PointCloud<PointWithScale>::Ptr> vec_keypoints(vec_cloud.size());
	vector<features::Descriptors> vec_descriptors(vec_cloud.size());

	PointCloud<PointWithScale>::Ptr tmp_keypoints(new PointCloud<PointWithScale>);

	for (int idx = 0; idx < N; ++idx) 
	{
		vec_keypoints[idx].reset(new PointCloud<PointWithScale>);

		opencvFinder->setInputCloud(vec_cloud[idx]);
		pclFinder->setInputCloud(vec_cloud[idx]);
		
		opencvFinder->computeKeypoints(*tmp_keypoints);
		pclFinder->computeDescriptors(*tmp_keypoints, vec_descriptors[idx], *vec_keypoints[idx]);

		cout << " cloud " << idx << endl;
		cout << "# keypoints : " << vec_keypoints[idx]->size() << endl;
		cout << "# descriptors : " << vec_descriptors[idx].rows << endl;
	}

	auto matcher = DescriptorMatcher::create("FlannBased");
	
	vector<DMatch> cv_matches;
	matcher->match(vec_descriptors[0], vec_descriptors[1], cv_matches);
 	
 	Correspondences pcl_matches;
 	for (const auto & cv_match : cv_matches)
 	{
 		pcl_matches.emplace_back(cv_match.queryIdx, cv_match.trainIdx, cv_match.distance);
 	}

	// visualize features
	PCLVisualizer viewer;
 	viewer.initCameraParameters();

	displayMatches<PointXYZRGBA, PointWithScale>(vec_cloud[0], vec_keypoints[0], 
												   vec_cloud[1], vec_keypoints[1], 
												   pcl_matches, viewer, 1.); // 1 m

 	// displays cloud until a key is pressed
	while (!viewer.wasStopped())
	{
		viewer.spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}

	return 0;
}