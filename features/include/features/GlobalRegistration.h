#ifndef GLOBAL_REGISTRATION_H
#define GLOBAL_REGISTRATION_H
#define PCL_NO_PRECOMPILE

#include <memory>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/correspondence.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation.h>
#include <pcl/registration/lum.h>

#include <pcl/search/brute_force.h>

#include "features/types.h"
#include "features/FeaturesFinder.h"
#include "features/FeaturesMatcher.h"

enum Status { SUCCESS, NOT_ENOUGH_INLIERS, NOT_ENOUGH_FEATURES };

const int DEFAULT_INLIERS_THRESHOLD = 40;
const float DEFAULT_LOCATION_RADIUS = 0.6; // meters
const int DEFAULT_RANSAC_MAX_ITER = 100;

struct CloudLocation
{
	PCL_ADD_POINT4D 
	float rel_x;
	float rel_y;
	float rel_z;
	int idx;

	inline CloudLocation (const CloudLocation &p)
	{
		x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
		rel_x = p.rel_x; rel_y = p.rel_y; rel_z = p.rel_z; idx = p.idx;
	}

	inline CloudLocation ()
	{
		x = y = z = 0.0f;
		data[3] = 1.0f;
		rel_x = rel_y = rel_z = 0;
		idx = -1;
	}

	inline CloudLocation (float _x, float _y, float _z, 
						  float _rel_x, float _rel_y, float _rel_z, int _idx)
	{
		x = _x; y = _y; z = _z;
		data[3] = 1.0f;
		rel_x = _rel_x; rel_y = _rel_y; rel_z = _rel_z; idx = _idx;
	}
	
	// TODO : definir
	// friend std::ostream& operator << (std::ostream& os, const PointXYZ& p);
	
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment


POINT_CLOUD_REGISTER_POINT_STRUCT ( CloudLocation,
                                    (float, x, x)
                                    (float, y, y)
                                    (float, z, z)
                                    (float, rel_x, rel_x)
                                    (float, rel_y, rel_y)
                                    (float, rel_z, rel_z)
                                    (int, idx, idx)
                                  )

template <typename PointInT, typename KeypointT, typename DescriptorT>
class GlobalRegistration
{

public:
	typedef std::shared_ptr<GlobalRegistration> Ptr;
	typedef std::shared_ptr<const GlobalRegistration> ConstPtr;

	typedef typename pcl::PointCloud<PointInT> PointCloudIn;
	typedef typename PointCloudIn::Ptr PointCloudInPtr;

	typedef typename pcl::PointCloud<KeypointT> KeypointCloud;
	typedef typename KeypointCloud::Ptr KeypointCloudPtr;

	typedef typename pcl::PointCloud<DescriptorT> DescriptorCloud;
	typedef typename DescriptorCloud::Ptr DescriptorCloudPtr;

	typedef FeaturesFinder<PointInT, KeypointT, DescriptorT> FeaturesFinderT;
	typedef typename FeaturesFinderT::Ptr FeaturesFinderTPtr;

	typedef FeaturesMatcher<DescriptorT> FeaturesMatcherT;
	typedef typename FeaturesMatcherT::Ptr FeaturesMatcherTPtr;

	typedef typename pcl::registration::CorrespondenceRejectorSampleConsensus<KeypointT> OutliersRejector;
	typedef typename OutliersRejector::Ptr OutliersRejectorPtr;

	typedef typename pcl::registration::TransformationEstimation<KeypointT, KeypointT, float> PairTransformationEstimation;
	typedef typename PairTransformationEstimation::Ptr PairTransformationEstimationPtr;

	typedef typename pcl::registration::LUM<KeypointT> GlobalAlignment;

protected:
	struct FeaturedCloud
	{
		FeaturedCloud(const PointCloudInPtr &cloud_arg = PointCloudInPtr()):
			cloud(cloud_arg),
			keypoints(new KeypointCloud), 
			descriptors(new DescriptorCloud) 
		{
			if (!cloud) cloud.reset(new PointCloudIn);
		}

		PointCloudInPtr cloud;
		KeypointCloudPtr keypoints;
		DescriptorCloudPtr descriptors;
	};

public:
	GlobalRegistration() :
		featured_clouds_(),
		features_finder_(),
		features_matcher_(),
		outliers_rejector_(),
		transformation_estimation_(),
		// icp_refinement_(false),
		global_alignment_(),
		inliers_threshold_(DEFAULT_INLIERS_THRESHOLD),
		radius_(DEFAULT_LOCATION_RADIUS),
		cloud_location_search_(),
		cloud_locations_(new pcl::PointCloud<CloudLocation>)
		{
			cloud_location_search_.setInputCloud(cloud_locations_);
		}

	~GlobalRegistration() {}

	virtual Status 
	proccessCloud(const PointCloudInPtr& cloud);

	void 
	setFeaturesFinder(const FeaturesFinderTPtr &features_finder)
	{
		features_finder_ = features_finder;	
	}

	FeaturesFinderTPtr
	getFeaturesFinder()
	{
		return features_finder_;	
	}

	void 
	setFeaturesMatcher(const FeaturesMatcherTPtr &features_matcher)
	{
		features_matcher_ = features_matcher;	
	}

	FeaturesMatcherTPtr
	getFeaturesMatcher()
	{
		return features_matcher_;	
	}

	void 
	setPairTransformationEstimation(const PairTransformationEstimationPtr &transformation_estimation)
	{
		transformation_estimation_ = transformation_estimation;
	}

	PairTransformationEstimationPtr
	getPairTransformationEstimation()
	{
		return transformation_estimation_;
	}

	void 
	setOutliersRejector(const OutliersRejectorPtr& outliers_rejector)
	{
		outliers_rejector_ = outliers_rejector;
	}

	OutliersRejectorPtr
	getOutliersRejector()
	{
		return outliers_rejector_;
	}

	void
	setInliersThreshold(int inliers_threshold)
	{
		inliers_threshold_ = inliers_threshold;
	}

	int
	getInliersThreshold()
	{
		return inliers_threshold_;
	}

	void
	setLocationRadius(float radius)
	{
		radius_ = radius;
	}

	float
	getLocationRadius()
	{
		return radius_;
	}

	void
	globalOptimize();

	void
	getTransformations(std::vector<Eigen::Affine3f>& transformations);

private:
	bool
	computeMatches(const FeaturedCloud& src, 
				   const FeaturedCloud& tgt,
				   MatchesInfo &matches_info);

	void 
	estimateTransformation(const FeaturedCloud& src, 
						   const FeaturedCloud& tgt, 
						   MatchesInfo &matches_info);

	void 
	initGlobalAlignment(const FeaturedCloud& new_featured_cloud);

	void 
	updateGlobalAlignment (const FeaturedCloud &new_featured_cloud, 
				 		   const MatchesInfo &matches_info);

	bool 
	findNewEdges();

	CloudLocation& 
	computeCloudLocation(int idx);

	FeaturesFinderTPtr features_finder_;

	FeaturesMatcherTPtr features_matcher_;

	OutliersRejectorPtr outliers_rejector_;	

	PairTransformationEstimationPtr transformation_estimation_;

	GlobalAlignment global_alignment_;

	int inliers_threshold_;

	float radius_;

	std::vector<FeaturedCloud> featured_clouds_;

	pcl::search::BruteForce<CloudLocation> cloud_location_search_;

	pcl::PointCloud<CloudLocation>::Ptr cloud_locations_;

	//	bool icp_refinement_;

};

#include <features/impl/GlobalRegistration.hpp>

#endif // GLOBAL_REGISTRATION_H