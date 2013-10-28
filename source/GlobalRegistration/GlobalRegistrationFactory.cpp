#define PCL_NO_PRECOMPILE

#include <memory>

#include <pcl/point_types.h>
#include <pcl/point_representation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/icp.h>
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

const int RANSAC_MAX_ITER = 100;
const int ICP_MAX_ITER = 50;

// GlobalRegistrationFactory::ORBAndSURF(float fx, float fy, float cx, float cy) const
GlobalRegistration<PointXYZRGBA, PointWithScale, SURFSignature128>::Ptr
GlobalRegistrationFactory::SURF(float fx, float fy, float cx, float cy, int hessian_threshold) const
{
	auto registration = std::make_shared<GlobalRegistration<PointXYZRGBA, PointWithScale, SURFSignature128>>(true, true);

	// feature detector setup	
	cv::initModule_nonfree();
	auto finder = std::make_shared<OpenCVRealFeaturesFinder<PointXYZRGBA, PointWithScale, SURFSignature128>>();
	finder->setKeypointDetector(cv::FeatureDetector::create("SURF"));
	finder->getKeypointDetector()->set("hessianThreshold", hessian_threshold);

	finder->setDescriptorExtractor(cv::DescriptorExtractor::create("SURF"));		
	finder->getDescriptorExtractor()->set("hessianThreshold", hessian_threshold);
	
	finder->setCameraParameters(fx, fy, cx, cy);
	registration->setFeaturesFinder(finder);
	
	// matcher setup	
	auto matcher = std::make_shared<RealFeaturesMatcher<SURFSignature128>>(true);
	DefaultPointRepresentation<SURFSignature128>::ConstPtr 
		repr(new DefaultPointRepresentation<SURFSignature128>);
	matcher->setPointRepresentation(repr);
	registration->setFeaturesMatcher(matcher);
	
	// rejector outlier setup
	CorrespondenceRejectorSampleConsensus<PointWithScale>::Ptr 
		rejector(new CorrespondenceRejectorSampleConsensus<PointWithScale>);
	rejector->setMaximumIterations(RANSAC_MAX_ITER);
	registration->setOutliersRejector(rejector);

	// pair transformation setup
	TransformationEstimationLM<PointWithScale, PointWithScale>::Ptr 
		estimator(new TransformationEstimationLM<PointWithScale, PointWithScale>);
	// TransformationEstimationSVD<PointWithScale, PointWithScale>::Ptr 
	// 	estimator(new TransformationEstimationSVD<PointWithScale, PointWithScale>);
	registration->setPairTransformationEstimation(estimator);

	// icp setup
	pcl::IterativeClosestPoint<PointWithScale, PointWithScale>::Ptr 
		icp(new pcl::IterativeClosestPoint<PointWithScale, PointWithScale>);
	icp->setUseReciprocalCorrespondences(true);
	icp->setMaximumIterations(ICP_MAX_ITER);
	registration->setICP(icp);

	return registration;
}

GlobalRegistration<PointXYZRGBA, PointWithScale, SURFSignature128>::Ptr
GlobalRegistrationFactory::ORBAndSURF(float fx, float fy, float cx, float cy, int nfeatures) const
{
	auto registration = std::make_shared<GlobalRegistration<PointXYZRGBA, PointWithScale, SURFSignature128>>(true, true);

	// feature detector setup	
	cv::initModule_nonfree();
	auto finder = std::make_shared<OpenCVRealFeaturesFinder<PointXYZRGBA, PointWithScale, SURFSignature128>>();
	finder->setKeypointDetector(cv::Ptr<cv::ORB>(new cv::ORB(nfeatures)));
	finder->setDescriptorExtractor(cv::DescriptorExtractor::create("SURF"));
		
	finder->setCameraParameters(fx, fy, cx, cy);
	registration->setFeaturesFinder(finder);
	
	// matcher setup	
	auto matcher = std::make_shared<RealFeaturesMatcher<SURFSignature128>>(true);
	DefaultPointRepresentation<SURFSignature128>::ConstPtr 
		repr(new DefaultPointRepresentation<SURFSignature128>);
	matcher->setPointRepresentation(repr);
	registration->setFeaturesMatcher(matcher);
	
	// rejector outlier setup
	CorrespondenceRejectorSampleConsensus<PointWithScale>::Ptr 
		rejector(new CorrespondenceRejectorSampleConsensus<PointWithScale>);
	rejector->setMaximumIterations(RANSAC_MAX_ITER);
	registration->setOutliersRejector(rejector);

	// pair transformation setup
	TransformationEstimationLM<PointWithScale, PointWithScale>::Ptr 
		estimator(new TransformationEstimationLM<PointWithScale, PointWithScale>);
	// TransformationEstimationSVD<PointWithScale, PointWithScale>::Ptr 
	// 	estimator(new TransformationEstimationSVD<PointWithScale, PointWithScale>);
	registration->setPairTransformationEstimation(estimator);

	// icp setup
	pcl::IterativeClosestPoint<PointWithScale, PointWithScale>::Ptr 
		icp(new pcl::IterativeClosestPoint<PointWithScale, PointWithScale>);
	icp->setUseReciprocalCorrespondences(true);
	icp->setMaximumIterations(ICP_MAX_ITER);
	registration->setICP(icp);

	return registration;
}
