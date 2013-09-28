#define PCL_NO_PRECOMPILE
// std includes
#include <iostream>
#include <memory>
// pcl includes
#include <pcl/common/common.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
// opencv includes
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
// boost includes
#include <boost/thread/thread.hpp>
// project includes
#include "features/descriptor_types.h"
#include "features/types.h"
#include "features/FeaturesFinder.h"
#include "features/OpenCVRealFeaturesFinder.h"
#include "features/RealFeaturesMatcher.h"
#include "features/Utils.h"

using namespace pcl;
using namespace pcl::io;
using namespace pcl::visualization;

FeaturesFinder<PointXYZRGBA, PointWithScale, SURFSignature64>::Ptr 
createSURFFinder()
{
	// SURF
	auto finder = std::make_shared<OpenCVRealFeaturesFinder<PointXYZRGBA, PointWithScale, SURFSignature64>>();
	
	cv::initModule_nonfree();

	auto detector = cv::FeatureDetector::create("SURF");
	auto extractor = cv::DescriptorExtractor::create("SURF");

	double hessianThreshold = 800;
	detector->set("hessianThreshold", hessianThreshold);
	extractor->set("hessianThreshold", hessianThreshold);

	finder->setKeypointDetector(detector);
	finder->setDescriptorExtractor(extractor);

	return finder;
}

int main(int argc, char* argv[])
{
	if (argc != 3)
	{
		PCL_ERROR("use: program cloud_file_src cloud_file_tgt\n");
		return -1;
	}

	PointCloud<PointXYZRGBA>::Ptr cloud[2]; 
	CloudFeatures<PointWithScale, SURFSignature64> features[2];

	auto finder = createSURFFinder(); 

	for (int idx = 0; idx < 2 ; ++idx)
	{
		cloud[idx].reset(new PointCloud<PointXYZRGBA>);
		if (loadPCDFile(argv[idx + 1], *cloud[idx]) == -1)
		{
			PCL_ERROR("coudn't read file %s\n", argv[idx + 1]);
			return -1;
		}

		finder->setInputCloud(cloud[idx]);
		finder->computeKeypointsAndDescriptors(*(features[idx].keypoints), 
											   *(features[idx].descriptors));
		
		const char* name = idx == 0 ? "src" : "tgt";
		PCL_INFO("# keypoints %s : %i\n", name, features[idx].keypoints->size() );
		PCL_INFO("# descriptors %s : %i\n", name, features[idx].descriptors->size() );
	}

	RealFeaturesMatcher<PointWithScale, SURFSignature64> matcher(true);
	MatchesInfo matches_info;

	DefaultPointRepresentation<SURFSignature64>::ConstPtr repr(new DefaultPointRepresentation<SURFSignature64>);

	matcher.setPointRepresentation(repr);
	matcher.match(features[0], features[1], matches_info);

	PCL_INFO("# matches : %i | cross_check : %s\n", matches_info.matches.size(), 
												    matcher.isCrossCheckEnabled() ? "YES" : "NO");

	// visualize features
	PCLVisualizer viewer;
 	viewer.initCameraParameters();

	displayMatches<PointXYZRGBA, PointWithScale>(cloud[0], features[0].keypoints,
												 cloud[1], features[1].keypoints,
												 matches_info.matches, viewer, 1.5);

 	// displays cloud until a key is pressed
	while (!viewer.wasStopped())
	{
		viewer.spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}

	return 0;
}