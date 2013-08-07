#ifndef PCL_FEATURES_FINDER_HPP
#define PCL_FEATURES_FINDER_HPP

#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>

#include "PCLFeaturesFinder.h"

namespace features
{

template<typename PointInT, typename KeypointT>
FPFHEstimationFeaturesFinder<PointInT, KeypointT>::FPFHEstimationFeaturesFinder(const KeypointDetectorPtr& keypointDetector) :
	keypointDetector_(keypointDetector),
	normalEstimator_(new NormalEstimator),
	descriptorExtractor_(new DescriptorExtractor) {}

template<typename PointInT, typename KeypointT> void
FPFHEstimationFeaturesFinder<PointInT, KeypointT>::computeKeypointsAndDescriptors(PointCloudKeypoint &keypoints, 
								  				   		  						  Descriptors &descriptors)
{
	PointCloudKeypoint tmp_keypoints;
	computeKeypoints(tmp_keypoints);
	computeDescriptors(tmp_keypoints, descriptors, keypoints);
}	

template<typename PointInT, typename KeypointT> void
FPFHEstimationFeaturesFinder<PointInT, KeypointT>::computeKeypoints(PointCloudKeypoint &keypoints) 
{
	keypoints.clear();

	keypointDetector_->setInputCloud(cloud_);
	keypointDetector_->compute(keypoints);

	std::vector<int> index;
	pcl::removeNaNFromPointCloud(keypoints, keypoints, index);
}

template<typename PointInT, typename KeypointT> void
FPFHEstimationFeaturesFinder<PointInT, KeypointT>::computeDescriptors(const PointCloudKeypoint &keypoints,
											  						  Descriptors &descriptors, 
											  						  PointCloudKeypoint &remaining_keypoints)
{	
	PointCloudInPtr keypoints_xyz(new PointCloudIn);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::IndicesPtr index(new std::vector<int>);

	// keypoints_xyz keeps xyz fields and has the surface type
	pcl::copyPointCloud(keypoints, *keypoints_xyz);

	computeNormals(keypoints_xyz, *normals, index);
	
	descriptorExtractor_->setInputCloud(keypoints_xyz);
	// descriptorExtractor_->setSearchSurface(cloud_);
	descriptorExtractor_->setInputNormals(normals);

	pcl::PointCloud<pcl::FPFHSignature33> pcl_descriptors;
	descriptorExtractor_->compute(pcl_descriptors);

	convertDescriptors(pcl_descriptors, descriptors);

	// TODO: filter descriptors & keypoints if neccesary

	pcl::copyPointCloud(keypoints, *index, remaining_keypoints);
}

template<typename PointInT, typename KeypointT> void
FPFHEstimationFeaturesFinder<PointInT, KeypointT>::computeNormals(PointCloudInPtr &keypoints_xyz, 
									      						  pcl::PointCloud<pcl::Normal> &normals, 
									      						  pcl::IndicesPtr &index)
{
	normals.clear();

	normalEstimator_->setInputCloud(keypoints_xyz);
	normalEstimator_->setSearchSurface(cloud_);
	normalEstimator_->compute(normals);
	
	// filter normals and then keypoints with index
	pcl::removeNaNNormalsFromPointCloud(normals, normals, *index);

	pcl::ExtractIndices<PointInT> filter;
	filter.setIndices(index);
	filter.filterDirectly(keypoints_xyz);
}

template<typename PointInT, typename KeypointT> void
FPFHEstimationFeaturesFinder<PointInT, KeypointT>::convertDescriptors(const pcl::PointCloud<pcl::FPFHSignature33> &pcl_descriptors, 
										      						  Descriptors &descriptors)
{
	pcl::DefaultPointRepresentation<pcl::FPFHSignature33> representation;
	
	descriptors.create(pcl_descriptors.size(), 
					   representation.getNumberOfDimensions(),
					   CV_32F);

	for (size_t idx = 0; idx < pcl_descriptors.size() ; ++idx)
	{
		float* descriptor_ptr = reinterpret_cast<float*>(descriptors.ptr(idx));
		representation.copyToFloatArray(pcl_descriptors[idx], descriptor_ptr);
	}
}

} // namespace features

#endif // PCL_FEATURES_FINDER_HPP