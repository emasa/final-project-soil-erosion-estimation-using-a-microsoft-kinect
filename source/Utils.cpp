
#include <pcl/common/common.h>
#include <pcl/recognition/color_gradient_dot_modality.h>
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/core/core.hpp>

#include "Features.h"
#include "Utils.h"

using namespace std;
using namespace pcl;
using namespace pcl::visualization;
using namespace cv;

void rgbCloudToImage(const PointCloud<PointXYZRGBA>& cloud, Mat& image)
{
    if (!cloud.isOrganized()) 
        throw pcl::UnorganizedPointCloudException("require Organized cloud to be converted to image");
    
    image.create(cloud.height, cloud.width, CV_8UC3);

    if (!cloud.empty()) {
        for (int h=0; h<image.rows; ++h) {
            for (int w=0; w<image.cols; ++w) {
                const auto& point = cloud.at(w, h);

                image.at<Vec3b>(h, w)[0] = point.b;
                image.at<Vec3b>(h, w)[1] = point.g;
                image.at<Vec3b>(h, w)[2] = point.r;
            }
        }
    }
}

namespace features
{

void displayKeypoints(const PointCloud<PointXYZRGBA>::Ptr &cloud, 
                      const PointCloud<BaseKeypoint>::Ptr &keypoints, 
                      PCLVisualizer &viewer, 
                      PointRGB color,
                      unsigned int size, 
                      const string &cloud_id,
                      const string &keypoints_id, 
                      int viewport)
{
    PointCloudColorHandlerRGBField<PointXYZRGBA> cloud_handler(cloud);

    viewer.addPointCloud(cloud, cloud_handler, cloud_id, viewport);

    PointCloudColorHandlerCustom<BaseKeypoint> keypoints_handler (keypoints, 
    															  color.r, 
    															  color.g, 
    															  color.b);
   
    viewer.addPointCloud(keypoints, keypoints_handler, keypoints_id, viewport);
    
    viewer.setPointCloudRenderingProperties (PCL_VISUALIZER_POINT_SIZE, size, 
    										 keypoints_id, viewport);

    // set the same pose (a bug in PCLVisualizer flip the keypoints 180 degrees)
    auto pose = viewer.getViewerPose(viewport);
    viewer.updatePointCloudPose(cloud_id, pose);
    viewer.updatePointCloudPose(keypoints_id, pose);
}

}