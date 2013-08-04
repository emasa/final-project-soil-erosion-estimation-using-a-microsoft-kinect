
#ifndef FEATURES_H
#define FEATURES_H

#include <pcl/common/common.h>
#include <opencv2/core/core.hpp>

namespace features 
{

// TODO: cambiar nombre BaseKeypoint
typedef pcl::PointWithScale BaseKeypoint;
typedef cv::Mat Descriptors;

// TODO: ver si definimos la clase Keypoints como un alias de
// vector<KEYPOINT, EIGEN<ALLOCATOR>> y refactorizar 

} // namespace features

#endif // FEATURES_H