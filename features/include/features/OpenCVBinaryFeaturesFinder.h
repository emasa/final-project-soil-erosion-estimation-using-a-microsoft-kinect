
#ifndef OPENCV_BINARY_FEATURES_FINDER
#define OPENCV_BINARY_FEATURES_FINDER

#include <opencv2/core/core.hpp>

#include "features/descriptor_types.h"
#include "features/OpenCVBaseFeaturesFinder.h"

template<typename PointInT, typename KeypointT>
class OpenCVBinaryFeaturesFinder 
	: public OpenCVBaseFeaturesFinder<PointInT, KeypointT, CustomSizeBinaryDescriptor>
{
	public:
		typedef std::shared_ptr<OpenCVBinaryFeaturesFinder> Ptr;
		typedef std::shared_ptr<const OpenCVBinaryFeaturesFinder> ConstPtr;

		typedef typename OpenCVBaseFeaturesFinder<PointInT, KeypointT, CustomSizeBinaryDescriptor>::PointCloudIn PointCloudIn;
		typedef typename OpenCVBaseFeaturesFinder<PointInT, KeypointT, CustomSizeBinaryDescriptor>::PointCloudInPtr PointCloudInPtr;
		typedef typename OpenCVBaseFeaturesFinder<PointInT, KeypointT, CustomSizeBinaryDescriptor>::PointCloudInConstPtr PointCloudInConstPtr;

		typedef typename OpenCVBaseFeaturesFinder<PointInT, KeypointT, CustomSizeBinaryDescriptor>::PointCloudKeypoint PointCloudKeypoint;	
		typedef typename OpenCVBaseFeaturesFinder<PointInT, KeypointT, CustomSizeBinaryDescriptor>::PointCloudDescriptor PointCloudDescriptor;
	
	protected:	
		void 
		convertDescriptors(const cv::Mat &cv_descriptors, 
						   PointCloudDescriptor &descriptors) override;
};

#include "features/impl/OpenCVBinaryFeaturesFinder.hpp"

#endif // OPENCV_BINARY_FEATURES_FINDER