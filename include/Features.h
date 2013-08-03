
#ifndef FEATURES_H
#define FEATURES_H

#include <pcl/common/common.h>
#include <opencv2/core/core.hpp>

namespace features 
{

typedef pcl::PointWithScale BaseKeypoint;
typedef cv::Mat Descriptors;

// TODO: decidir como implementarlo
struct VisualKeypoint {
	BaseKeypoint base_keypoint;
	cv::KeyPoint keypoint2D;
};

} // namespace features

#endif // FEATURES_H