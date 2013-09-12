
#ifndef OPENCV_BINARY_FEATURES_FINDER
#define OPENCV_BINARY_FEATURES_FINDER

#include <opencv2/core/core.hpp>

#include "features/descriptor_types.h"
#include "features/OpenCVBaseFeaturesFinder.h"

template<typename PointInT, typename KeypointT, typename BinaryDescriptorT>
class OpenCVBinaryFeaturesFinder 
	: public OpenCVBaseFeaturesFinder<PointInT, KeypointT, BinaryDescriptorT>
{
	public:
		typedef std::shared_ptr<OpenCVBinaryFeaturesFinder> Ptr;
		typedef std::shared_ptr<const OpenCVBinaryFeaturesFinder> ConstPtr;

		typedef typename OpenCVBaseFeaturesFinder<PointInT, KeypointT, BinaryDescriptorT>::PointCloudIn PointCloudIn;
		typedef typename OpenCVBaseFeaturesFinder<PointInT, KeypointT, BinaryDescriptorT>::PointCloudInPtr PointCloudInPtr;
		typedef typename OpenCVBaseFeaturesFinder<PointInT, KeypointT, BinaryDescriptorT>::PointCloudInConstPtr PointCloudInConstPtr;

		typedef typename OpenCVBaseFeaturesFinder<PointInT, KeypointT, BinaryDescriptorT>::PointCloudKeypoint PointCloudKeypoint;	
		typedef typename OpenCVBaseFeaturesFinder<PointInT, KeypointT, BinaryDescriptorT>::PointCloudDescriptor PointCloudDescriptor;
	
	protected:	
		void 
		convertDescriptors(const cv::Mat &cv_descriptors, 
						   PointCloudDescriptor &descriptors) override;
};

#include "features/impl/OpenCVBinaryFeaturesFinder.hpp"

#endif // OPENCV_BINARY_FEATURES_FINDER