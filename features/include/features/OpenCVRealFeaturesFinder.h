
#ifndef OPENCV_REAL_FEATURES_FINDER
#define OPENCV_REAL_FEATURES_FINDER

#include <opencv2/core/core.hpp>

#include "features/descriptor_types.h"
#include "features/OpenCVBaseFeaturesFinder.h"

template<typename PointInT, typename KeypointT>
class OpenCVRealFeaturesFinder 
	: public OpenCVBaseFeaturesFinder<PointInT, KeypointT, CustomSizeRealDescriptor>
{
	public:
		typedef std::shared_ptr<OpenCVRealFeaturesFinder> Ptr;
		typedef std::shared_ptr<const OpenCVRealFeaturesFinder> ConstPtr;

		typedef typename OpenCVBaseFeaturesFinder<PointInT, KeypointT, CustomSizeRealDescriptor>::PointCloudIn PointCloudIn;
		typedef typename OpenCVBaseFeaturesFinder<PointInT, KeypointT, CustomSizeRealDescriptor>::PointCloudInPtr PointCloudInPtr;
		typedef typename OpenCVBaseFeaturesFinder<PointInT, KeypointT, CustomSizeRealDescriptor>::PointCloudInConstPtr PointCloudInConstPtr;

		typedef typename OpenCVBaseFeaturesFinder<PointInT, KeypointT, CustomSizeRealDescriptor>::PointCloudKeypoint PointCloudKeypoint;	
		typedef typename OpenCVBaseFeaturesFinder<PointInT, KeypointT, CustomSizeRealDescriptor>::PointCloudDescriptor PointCloudDescriptor;
	
	protected:	
		void 
		convertDescriptors(const cv::Mat &cv_descriptors, 
						   PointCloudDescriptor &descriptors) override;
};

#include "features/impl/OpenCVRealFeaturesFinder.hpp"

#endif // OPENCV_REAL_FEATURES_FINDER