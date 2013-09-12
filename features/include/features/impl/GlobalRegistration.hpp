
#ifndef GLOBAL_REGISTRATION_HPP
#define GLOBAL_REGISTRATION_HPP

#include <vector>
#include <algorithm>
#include <assert.h>

#include <Eigen/StdVector>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/correspondence.h>
#include <pcl/console/print.h>

#include "features/types.h"
#include "features/GlobalRegistration.h"

template<typename PointInT, typename KeypointT, typename DescriptorT> Status 
GlobalRegistration<PointInT, KeypointT, DescriptorT>::proccessCloud(const PointCloudInPtr& cloud)
{
	FeaturedCloud featured_cloud(cloud);
	features_finder_->setInputCloud(featured_cloud.cloud);
	features_finder_->computeKeypointsAndDescriptors(*featured_cloud.keypoints, 
													 *featured_cloud.descriptors);		

	int num_features = featured_cloud.keypoints->size();
	if ( num_features < inliers_threshold_ ) 
	{
		PCL_WARN("%i features are not enough. min required : %i\n", 
				  num_features, inliers_threshold_);
		return NOT_ENOUGH_FEATURES;
	}
	PCL_INFO("# features found : %i\n", num_features);
	
	if (featured_clouds_.empty())
	{
		initGlobalAlignment(featured_cloud);
	} else {	
		const auto &last_featured_cloud = featured_clouds_.back();	
		MatchesInfo matches_info;
		bool ransac = computeMatches(featured_cloud, last_featured_cloud, matches_info);

		int num_inliers = matches_info.matches->size();
		if ( num_inliers < inliers_threshold_ ) 
		{
			PCL_WARN("%i inliers [%s ransac] are not enough. min required : %i\n", 
					  num_inliers, ransac ? "after" : "before", inliers_threshold_);
			return NOT_ENOUGH_INLIERS;
		}

		estimateTransformation(featured_cloud, last_featured_cloud, matches_info);

		updateGlobalAlignment(featured_cloud, matches_info);
	} 

	auto last_loc = cloud_locations_->back();
	PCL_INFO("position : (%f, %f, %f)\n", last_loc.x, last_loc.y, last_loc.z);

	return SUCCESS; 
}

template<typename PointInT, typename KeypointT, typename DescriptorT> void 
GlobalRegistration<PointInT, KeypointT, DescriptorT>::globalOptimize()
{
	PCL_INFO("applying global optimization\n");
	global_alignment_.compute();
	for (int idx = 0 ; idx < cloud_locations_->size() ; ++idx)
		computeCloudLocation(idx);
}

template<typename PointInT, typename KeypointT, typename DescriptorT> void 
GlobalRegistration<PointInT, KeypointT, DescriptorT>::getTransformations(std::vector<Eigen::Affine3f>& transformations)
{
	transformations.resize(global_alignment_.getNumVertices());
	for (int idx = 0 ; idx < transformations.size() ; ++idx)
	{
		transformations[idx] = global_alignment_.getTransformation(idx);
	}
}

template<typename PointInT, typename KeypointT, typename DescriptorT> bool 
GlobalRegistration<PointInT, KeypointT, DescriptorT>::computeMatches(const FeaturedCloud& src, 
																	 const FeaturedCloud& tgt,
																	 MatchesInfo &matches_info)
{
	pcl::CorrespondencesPtr tmp_matches(new pcl::Correspondences);
	features_matcher_->match(src.descriptors, tgt.descriptors, *tmp_matches); 
	PCL_INFO("# initial matches [before ransac]: %i\n", tmp_matches->size());

	if (tmp_matches->size() < inliers_threshold_) 
	{
		PCL_INFO("too few inliers. avoid ransac rejection\n");
		matches_info.matches = tmp_matches;
		return false;
	}

	outliers_rejector_->setInputSource(src.keypoints);
	outliers_rejector_->setInputTarget(tgt.keypoints);
	outliers_rejector_->getRemainingCorrespondences(*tmp_matches, *matches_info.matches);		
	matches_info.transformation = outliers_rejector_->getBestTransformation();
	
	PCL_INFO("# geometrically consistent matches [after ransac]: %i\n", matches_info.matches->size());

	return true;
}

template<typename PointInT, typename KeypointT, typename DescriptorT> void 
GlobalRegistration<PointInT, KeypointT, DescriptorT>::estimateTransformation(const FeaturedCloud& src, 
																			 const FeaturedCloud& tgt, 
																			 MatchesInfo &matches_info)
{
	transformation_estimation_->estimateRigidTransformation (*src.keypoints, 
												   		     *tgt.keypoints, 
										  		   		     *matches_info.matches, 
										  		   		     matches_info.transformation);
	
	// if (icp_refinement_) { }
}


template<typename PointInT, typename KeypointT, typename DescriptorT> void 
GlobalRegistration<PointInT, KeypointT, DescriptorT>::initGlobalAlignment(const FeaturedCloud& new_featured_cloud)
{
	featured_clouds_.push_back(new_featured_cloud);
	global_alignment_.addPointCloud(new_featured_cloud.keypoints);
	computeCloudLocation(0);
}

template<typename PointInT, typename KeypointT, typename DescriptorT> void 
GlobalRegistration<PointInT, KeypointT, DescriptorT>::updateGlobalAlignment (const FeaturedCloud &new_featured_cloud, 
	  																		 const MatchesInfo &matches_info)
{
	int new_vertex = global_alignment_.getNumVertices(), last_vertex = new_vertex - 1; 
	Eigen::Affine3f last_pose = global_alignment_.getTransformation(last_vertex);
	Eigen::Affine3f relative_pose (matches_info.transformation);
	Eigen::Affine3f new_pose = last_pose * relative_pose;
	
	Eigen::Vector6f new_pose_vec;
	pcl::getTranslationAndEulerAngles(new_pose, new_pose_vec(0), new_pose_vec(1),
												new_pose_vec(2), new_pose_vec(3), 
												new_pose_vec(4), new_pose_vec(5));

	featured_clouds_.push_back(new_featured_cloud);
	global_alignment_.addPointCloud(new_featured_cloud.keypoints, new_pose_vec);
	global_alignment_.setCorrespondences(new_vertex, last_vertex, matches_info.matches);

	if (findNewEdges()) 
	{
		globalOptimize();	
	}
}

template<typename PointInT, typename KeypointT, typename DescriptorT> CloudLocation& 
GlobalRegistration<PointInT, KeypointT, DescriptorT>::computeCloudLocation(int idx)
{
	assert(idx >= 0) ; assert(static_cast<int>(featured_clouds_.size()) > idx);
	
	if ( static_cast<int>(cloud_locations_->size()) <= idx )
		cloud_locations_->resize(idx + 1); 

	auto &loc = (*cloud_locations_)[idx];

	if ( loc.idx == -1 ) 
	{
		Eigen::Vector4f rel_centroid;
		int valid = pcl::compute3DCentroid(*featured_clouds_[idx].cloud, rel_centroid);		
		assert( valid ); // TODO: resolve what to do when valid == 0

		loc.rel_x = rel_centroid(0); 
		loc.rel_y = rel_centroid(1); 
		loc.rel_z = rel_centroid(2);
		loc.idx = idx;		
	}

	assert ( loc.idx == idx ); // check consistency

	// update reference
	loc.x = loc.rel_x; loc.y = loc.rel_y; loc.z = loc.rel_z; 
	loc = pcl::transformPoint(loc, global_alignment_.getTransformation(idx));

	cloud_location_search_.setInputCloud(cloud_locations_); // update (maybe don't needed)
	
	return loc;
}


template<typename PointInT, typename KeypointT, typename DescriptorT> bool 
GlobalRegistration<PointInT, KeypointT, DescriptorT>::findNewEdges()
{
	int idx = static_cast<int>(featured_clouds_.size()) - 1;
	const auto &loc = computeCloudLocation(idx);

	std::vector< int > k_indices;
	std::vector< float > k_sqr_distances;

	int n_found = cloud_location_search_.radiusSearch (loc, radius_, k_indices, k_sqr_distances);

	std::sort(k_indices.begin(), k_indices.end()); // can't use k_sqr_distances any longer 

	int edges = 0; 
	int max_edges = 2, window_size = 4; // TODO: definir como atributos

	PCL_INFO("searching for new edges :\n");

	for (auto ngb_idx : k_indices)
	{
		auto &ngb_loc = (*cloud_locations_)[ ngb_idx ];
		
		if (ngb_loc.idx == loc.idx) continue;
		if (ngb_loc.idx >= loc.idx - window_size) 
		{
			PCL_INFO("skipping cloud %i in clouds window of size %i\n", 
		  			 ngb_loc.idx, window_size);
			continue; // skip clouds in window_size
		}

		MatchesInfo matches_info;
		computeMatches(featured_clouds_[loc.idx], featured_clouds_[ngb_loc.idx], matches_info);

		int num_inliers = matches_info.matches->size();
		if ( num_inliers < inliers_threshold_ ) 
		{
			PCL_INFO("clouds %i, %i are close but %i inliers are not enough. min required : %i\n", 
					  ngb_loc.idx, loc.idx, num_inliers, inliers_threshold_);
		} else {
			PCL_INFO("adding edge %i->%i to the graph.\n", loc.idx, ngb_loc.idx);
			global_alignment_.setCorrespondences(loc.idx, ngb_loc.idx, matches_info.matches);

			if ( ++edges == max_edges ) break; // added enough edges
		}
	}	

	PCL_INFO("%i new edges added\n", edges);

	return edges > 0;
}

#endif // GLOBAL_REGISTRATION_HPP