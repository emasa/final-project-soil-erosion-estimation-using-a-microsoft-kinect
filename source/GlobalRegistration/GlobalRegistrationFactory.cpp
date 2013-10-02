#define PCL_NO_PRECOMPILE

#include <memory>

#include <pcl/point_types.h>
#include <pcl/point_representation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

#include "Descriptor/DescriptorType.h"
#include "Descriptor/DescriptorRepresentation.h"
#include "FeaturesFinder/FeaturesFinder.h"
#include "FeaturesFinder/OpenCVRealFeaturesFinder.h"
#include "FeaturesMatcher/FeaturesMatcher.h"
#include "FeaturesMatcher/RealFeaturesMatcher.h"
#include "GlobalRegistration/GlobalRegistration.h"
#include "GlobalRegistration/GlobalRegistrationFactory.h"

using namespace pcl;
using namespace pcl::registration;

GlobalRegistration<PointXYZRGBA, PointWithScale, SURFSignature128>::Ptr
GlobalRegistrationFactory::ORBAndSURF() const
{
	auto registration = std::make_shared<GlobalRegistration<PointXYZRGBA, PointWithScale, SURFSignature128>>();

	// feature detector setup	
	cv::initModule_nonfree();
	auto finder = std::make_shared<OpenCVRealFeaturesFinder<PointXYZRGBA, PointWithScale, SURFSignature128>>();
	finder->setKeypointDetector(cv::FeatureDetector::create("ORB"));	
	finder->setDescriptorExtractor(cv::DescriptorExtractor::create("SURF"));
	registration->setFeaturesFinder(finder);
	
	// matcher setup	
	auto matcher = std::make_shared<RealFeaturesMatcher<SURFSignature128>>(true);
	DefaultPointRepresentation<SURFSignature128>::ConstPtr repr(new DefaultPointRepresentation<SURFSignature128>);
	matcher->setPointRepresentation(repr);
	registration->setFeaturesMatcher(matcher);
	
	// rejector outlier setup
	CorrespondenceRejectorSampleConsensus<PointWithScale>::Ptr rejector (new CorrespondenceRejectorSampleConsensus<PointWithScale>);
	rejector->setMaximumIterations(DEFAULT_RANSAC_MAX_ITER);
	registration->setOutliersRejector(rejector);

	// pair transformation setup
	TransformationEstimationLM<PointWithScale, PointWithScale>::Ptr estimator(new TransformationEstimationLM<PointWithScale, PointWithScale>);
	registration->setPairTransformationEstimation(estimator);

	return registration;
}