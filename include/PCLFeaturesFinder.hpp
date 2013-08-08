#ifndef PCL_FEATURES_FINDER_HPP
#define PCL_FEATURES_FINDER_HPP

#include <iostream>

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
	PointCloudKeypoint cleaned_keypoints;
	pcl::IndicesPtr	valid_indices (new std::vector<int>);
	pcl::removeNaNFromPointCloud(keypoints, cleaned_keypoints,  *valid_indices);

	PointCloudInPtr keypoints_xyz(new PointCloudIn);
	// keypoints_xyz keeps xyz fields with same type that cloud_
	// improve normals estimation using cloud_ as surface 	
	pcl::copyPointCloud(cleaned_keypoints, *keypoints_xyz);

	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	computeNormals(keypoints_xyz, *normals, *valid_indices);

	PointCloudInPtr keypoints_xyz_normals(new PointCloudIn);
	if (keypoints_xyz->size() == normals->size()) {
		keypoints_xyz_normals = keypoints_xyz;
	} else {
		pcl::copyPointCloud(*keypoints_xyz, *valid_indices, *keypoints_xyz_normals);
	}

	descriptorExtractor_->setInputCloud(keypoints_xyz_normals);
	descriptorExtractor_->setInputNormals(normals);

	pcl::PointCloud<pcl::FPFHSignature33> pcl_descriptors;
	descriptorExtractor_->compute(pcl_descriptors);

	// TODO: filter descriptors & keypoints if neccesary

	convertDescriptors(pcl_descriptors, descriptors);
	pcl::copyPointCloud(cleaned_keypoints, *valid_indices, remaining_keypoints);
}

template<typename PointInT, typename KeypointT> void
FPFHEstimationFeaturesFinder<PointInT, KeypointT>::computeNormals(const PointCloudInPtr &keypoints_xyz, 
									      						  pcl::PointCloud<pcl::Normal> &normals, 
									      						  std::vector<int> &indices)
{
	normals.clear();
	indices.clear();

	normalEstimator_->setInputCloud(keypoints_xyz);
	normalEstimator_->setSearchSurface(cloud_);
	normalEstimator_->compute(normals);
	
	pcl::removeNaNNormalsFromPointCloud(normals, normals, indices);

	// TODO: REPORTAR BUG A PCL : NO FUNCIONA !!!!
	// pcl::ExtractIndices<PointInT> filter;
	// filter.setIndices(index); 
	// filter.filterDirectly(keypoints_xyz);
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

#define PCL_INSTANTIATE_FPFHEstimationFeaturesFinder(T,KT) template class PCL_EXPORTS features::FPFHEstimationFeaturesFinder<T,KT>;

#endif // PCL_FEATURES_FINDER_HPP