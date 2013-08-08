#ifndef PCL_FEATURES_FINDER_H
#define PCL_FEATURES_FINDER_H

#include <vector>
#include <memory>

#include <pcl/common/common.h>
#include <pcl/features/feature.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/keypoints/keypoint.h>

#include "Features.h"
#include "FeaturesFinder.h"

namespace features 
{

template<typename PointInT, typename KeypointT>
class FPFHEstimationFeaturesFinder : public FeaturesFinder<PointInT, KeypointT>
{
public:
	typedef std::shared_ptr<FPFHEstimationFeaturesFinder> Ptr;
	typedef std::shared_ptr<const FPFHEstimationFeaturesFinder> ConstPtr;

	typedef typename FeaturesFinder<PointInT, KeypointT>::PointCloudIn PointCloudIn;
	typedef typename PointCloudIn::Ptr PointCloudInPtr;

	typedef typename FeaturesFinder<PointInT, KeypointT>::PointCloudKeypoint PointCloudKeypoint;

	typedef typename pcl::Keypoint<PointInT, KeypointT> KeypointDetector;
	typedef typename KeypointDetector::Ptr KeypointDetectorPtr;
	
	typedef typename pcl::FPFHEstimationOMP<PointInT, pcl::Normal, pcl::FPFHSignature33> DescriptorExtractor;
	typedef typename DescriptorExtractor::Ptr DescriptorExtractorPtr;

	typedef typename pcl::NormalEstimationOMP<PointInT, pcl::Normal> NormalEstimator;
	typedef typename NormalEstimator::Ptr NormalEstimatorPtr;

	FPFHEstimationFeaturesFinder(const KeypointDetectorPtr &keypointDetector = KeypointDetectorPtr());
	
	virtual ~FPFHEstimationFeaturesFinder() {};

	void 
	computeKeypointsAndDescriptors(PointCloudKeypoint &keypoints, 
				  				   Descriptors &descriptors);	

	void 
	computeKeypoints(PointCloudKeypoint &keypoints);

	void 
	computeDescriptors(const PointCloudKeypoint &keypoints, 
 					   Descriptors &descriptors, 
 					   PointCloudKeypoint &remaining_keypoints);

	void 
	computeNormals(const PointCloudInPtr &keypoints_xyz, 
				   pcl::PointCloud<pcl::Normal> &normals, 
				   std::vector<int> &indeces);

	void 
	setInputCloud(const PointCloudInPtr &cloud)
	{
		cloud_ = cloud;
	}

	void
	setKeypointDetector(const KeypointDetectorPtr& keypointDetector)
	{
		keypointDetector_ = keypointDetector;
	}

	void 
	setRadiusSearchNormals(float radius)
	{
		normalEstimator_->setKSearch(0);
		normalEstimator_->setRadiusSearch(radius);
	}

	void 
	setKSearchNormals(int k)
	{
		normalEstimator_->setRadiusSearch(0);
		normalEstimator_->setKSearch(k);
	}

	void 
	setRadiusSearchFPFH(float radius)
	{
		descriptorExtractor_->setKSearch(0);
		descriptorExtractor_->setRadiusSearch(radius);
	}

	void 
	setKSearchFPFH(int k)
	{
		descriptorExtractor_->setRadiusSearch(0);
		descriptorExtractor_->setKSearch(k);
	}

	void 
	setNrSubdivisionsFPFH(int nr_bins_f1, int nr_bins_f2, int nr_bins_f3)
	{
		descriptorExtractor_->setNrSubdivisions(nr_bins_f1, nr_bins_f2, nr_bins_f3);
	}

protected:
	void convertDescriptors(const pcl::PointCloud<pcl::FPFHSignature33> &pcl_descriptors,
							Descriptors &descriptors);

	PointCloudInPtr cloud_;

	KeypointDetectorPtr keypointDetector_;
	DescriptorExtractorPtr descriptorExtractor_;
	NormalEstimatorPtr normalEstimator_; 
};

} // namespace features

#include "PCLFeaturesFinder.hpp"

#endif // PCL_FEATURES_FINDER_H
