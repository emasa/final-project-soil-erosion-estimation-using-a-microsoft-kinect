
#ifndef OPENCV_REAL_FEATURES_FINDER
#define OPENCV_REAL_FEATURES_FINDER

#include <opencv2/core/core.hpp>

#include "FeaturesFinder/OpenCVBaseFeaturesFinder.h"

template<typename PointInT, typename KeypointT, typename RealDescriptorT>
class OpenCVRealFeaturesFinder 
	: public OpenCVBaseFeaturesFinder<PointInT, KeypointT, RealDescriptorT>
{
	public:
		typedef std::shared_ptr<OpenCVRealFeaturesFinder> Ptr;
		typedef std::shared_ptr<const OpenCVRealFeaturesFinder> ConstPtr;

		typedef typename OpenCVBaseFeaturesFinder<PointInT, KeypointT, RealDescriptorT>::PointCloudIn PointCloudIn;
		typedef typename OpenCVBaseFeaturesFinder<PointInT, KeypointT, RealDescriptorT>::PointCloudInPtr PointCloudInPtr;
		typedef typename OpenCVBaseFeaturesFinder<PointInT, KeypointT, RealDescriptorT>::PointCloudInConstPtr PointCloudInConstPtr;

		typedef typename OpenCVBaseFeaturesFinder<PointInT, KeypointT, RealDescriptorT>::PointCloudKeypoint PointCloudKeypoint;	
		typedef typename OpenCVBaseFeaturesFinder<PointInT, KeypointT, RealDescriptorT>::PointCloudDescriptor PointCloudDescriptor;
	
	protected:	
		void 
		convertDescriptors(const cv::Mat &cv_descriptors, 
						   PointCloudDescriptor &descriptors) override;
};

#include "FeaturesFinder/OpenCVRealFeaturesFinder.hpp"

#endif // OPENCV_REAL_FEATURES_FINDER