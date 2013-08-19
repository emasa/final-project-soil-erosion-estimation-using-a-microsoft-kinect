
#include <memory>
#include <algorithm>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/utils.h>
#include <pcl/io/pcd_io.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <boost/range/algorithm/equal.hpp>

#include "gtest/gtest.h"

#include "features/descriptor_types.h"
#include "features/FeaturesFinder.h"
#include "features/OpenCVBinaryFeaturesFinder.h"
#include "features/OpenCVRealFeaturesFinder.h"

using namespace pcl;
using namespace pcl::io;

FeaturesFinder<PointXYZRGBA, PointWithScale, CustomSizeBinaryDescriptor>::Ptr 
createORBFinder() 
{
	// ORB (default parameters)
	auto finder = std::make_shared<OpenCVBinaryFeaturesFinder<PointXYZRGBA, PointWithScale>>(); 
	finder->setKeypointDetector(cv::FeatureDetector::create("ORB"));
	finder->setDescriptorExtractor(cv::DescriptorExtractor::create("ORB"));

	return finder;
}

FeaturesFinder<PointXYZRGBA, PointWithScale, CustomSizeRealDescriptor>::Ptr 
createSURFFinder()
{
	// SURF
	auto finder = std::make_shared<OpenCVRealFeaturesFinder<PointXYZRGBA, PointWithScale>>();
	
	cv::initModule_nonfree();

	auto detector = cv::FeatureDetector::create("SURF");
	auto extractor = cv::DescriptorExtractor::create("SURF");

	double hessianThreshold = 1000;
	detector->set("hessianThreshold", hessianThreshold);
	extractor->set("hessianThreshold", hessianThreshold);

	finder->setKeypointDetector(detector);
	finder->setDescriptorExtractor(extractor);

	return finder;
}

PointCloud<PointXYZRGBA>::Ptr cloud; // global variable

TEST(OpenCVFeaturesFinderTestCase, BasicBinaryFeatureTest)
{
	auto finder = createORBFinder();
	finder->setInputCloud(cloud);

	PointCloud<PointWithScale> tmp_keypoints, keypoints1, keypoints2;	
	PointCloud<CustomSizeBinaryDescriptor> descriptors1, descriptors2;

	finder->computeKeypoints(tmp_keypoints);
	finder->computeDescriptors(tmp_keypoints, descriptors1, keypoints1);

	finder->computeKeypointsAndDescriptors(keypoints2, descriptors2);

	ASSERT_EQ(keypoints1.size(), descriptors1.size());
	ASSERT_EQ(keypoints2.size(), descriptors2.size());
	
	ASSERT_EQ(keypoints1.size(), keypoints2.size());

	float EPS = 1.0e-5;

	int size = static_cast<int>(keypoints1.size());

	for (int idx = 0; idx < size ; ++idx)
	{
		const auto &p1 = keypoints1[idx], &p2 = keypoints2[idx];
		EXPECT_NEAR(p1.x, p2.x, EPS); 
		EXPECT_NEAR(p1.y, p2.y, EPS); 
		EXPECT_NEAR(p1.z, p2.z, EPS);
		EXPECT_NEAR(p1.scale, p2.scale, EPS); 
		EXPECT_NEAR(p1.angle, p2.angle, EPS);
	    EXPECT_NEAR(p1.response, p2.response, EPS); 
	    EXPECT_EQ(p1.octave, p2.octave);
	}
	
	for (int idx = 0; idx < size ; ++idx)
	{
		const auto &d1 = descriptors1[idx], &d2 = descriptors1[idx]; 

		EXPECT_EQ(d1.descriptor.size(), d2.descriptor.size());

		EXPECT_TRUE( boost::algorithm::equals(d1.descriptor, d2.descriptor) );
	}
}

TEST(OpenCVFeaturesFinderTestCase, BasicRealFeatureTest)
{
	auto finder = createSURFFinder();
	finder->setInputCloud(cloud);

	PointCloud<PointWithScale> tmp_keypoints, keypoints1, keypoints2;	
	PointCloud<CustomSizeRealDescriptor> descriptors1, descriptors2;

	finder->computeKeypoints(tmp_keypoints);
	finder->computeDescriptors(tmp_keypoints, descriptors1, keypoints1);

	finder->computeKeypointsAndDescriptors(keypoints2, descriptors2);

	ASSERT_EQ(keypoints1.size(), descriptors1.size());
	ASSERT_EQ(keypoints2.size(), descriptors2.size());
	
	ASSERT_EQ(keypoints1.size(), keypoints2.size());

	float EPS = 1.0e-5;

	int size = static_cast<int>(keypoints1.size());

	for (int idx = 0; idx < size ; ++idx)
	{
		const auto &p1 = keypoints1[idx], &p2 = keypoints2[idx];
		EXPECT_NEAR(p1.x, p2.x, EPS); 
		EXPECT_NEAR(p1.y, p2.y, EPS); 
		EXPECT_NEAR(p1.z, p2.z, EPS);
		EXPECT_NEAR(p1.scale, p2.scale, EPS); 
		EXPECT_NEAR(p1.angle, p2.angle, EPS);
	    EXPECT_NEAR(p1.response, p2.response, EPS); 
	    EXPECT_EQ(p1.octave, p2.octave);
	}
	
	for (int idx = 0; idx < size ; ++idx)
	{
		const auto &d1 = descriptors1[idx], &d2 = descriptors1[idx]; 
		
		EXPECT_EQ(d1.descriptor.size(), d2.descriptor.size());

		for (int field_idx = 0; field_idx < d1.descriptor.size() ; ++field_idx)
			EXPECT_NEAR(d1.descriptor[field_idx], d2.descriptor[field_idx], EPS);
	}
}

int main(int argc, char* argv[])
{
	if (argc != 2)
	{
		PCL_ERROR("use: test cloud_file\n");
		return -1;
	}

	cloud.reset(new PointCloud<PointXYZRGBA>);
	
	if (loadPCDFile(argv[1], *cloud) == -1)
	{
		PCL_ERROR("couldn't read file %s\n", argv[1]);
		return -1;
	}

	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}