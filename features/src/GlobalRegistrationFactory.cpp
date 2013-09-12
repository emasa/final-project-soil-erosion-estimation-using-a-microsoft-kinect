#define PCL_NO_PRECOMPILE

#include <memory>

#include <pcl/point_types.h>
#include <pcl/point_representation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_lm.h>

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

#include "features/FeaturesFinder.h"
#include "features/OpenCVRealFeaturesFinder.h"
#include "features/FeaturesMatcher.h"
#include "features/RealFeaturesMatcher.h"
#include "features/descriptor_types.h"
#include "features/descriptor_representations.h"

#include "features/GlobalRegistration.h"
#include "features/GlobalRegistrationFactory.h"

using namespace pcl;
using namespace pcl::registration;

void
GlobalRegistrationFactory::create(GlobalRegistration<PointXYZRGBA, PointWithScale, SURFSignature128>::Ptr &registration) const
{
	if (!registration)
		registration = std::make_shared<GlobalRegistration<PointXYZRGBA, PointWithScale, SURFSignature128>>();


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
}