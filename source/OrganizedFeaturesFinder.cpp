
#include <vector>

#include <pcl/common/common.h>
#include <opencv2/features2d/features2d.hpp>

#include "../include/OrganizedFeaturesFinder.h"
#include "../include/Utils.h"

using namespace std;
using namespace pcl;
using namespace cv;

namespace features 
{

OrganizedFeaturesFinder::OrganizedFeaturesFinder(const cv::Ptr<cv::FeatureDetector> &keypointDetector_, 
					 					   			    const cv::Ptr<cv::DescriptorExtractor> &descriptorExtractor_) :
	FeaturesFinder(), cloud(), image(), cv_keypoints(),
	keypointDetector(keypointDetector_), descriptorExtractor(descriptorExtractor_) 
	{}

void OrganizedFeaturesFinder::setCloud(const PointCloud<PointXYZRGBA>::Ptr &cloud_)
{
	// TODO: agregar chequeos de null pointers
	cloud = cloud_;
	rgbCloudToImage(*cloud, image);
}

void OrganizedFeaturesFinder::setKeypointDetector(const cv::Ptr<cv::FeatureDetector> &keypointDetector_) 
{
	// TODO: agregar chequeos de null pointers
	keypointDetector = keypointDetector_;	
}

void OrganizedFeaturesFinder::setDescriptorExtractor(const cv::Ptr<cv::DescriptorExtractor> &descriptorExtractor_)
{
	// TODO: agregar chequeos de null pointers
	descriptorExtractor = descriptorExtractor_;
}

void OrganizedFeaturesFinder::find(PointCloud<BaseKeypoint> &keypoints, 
				  				   Descriptors &descriptors)
{
	keypointDetector->detect(image, cv_keypoints);

	// Keypoints for which a descriptor cannot be computed are removed
	// Sometimes new keypoints can be added
	descriptorExtractor->compute(image, cv_keypoints, descriptors);
	
	convertKeypoints(cv_keypoints, keypoints);
}

// TODO: filtrar cv_keypoints
void OrganizedFeaturesFinder::convertKeypoints(const vector<cv::KeyPoint> &cv_keypoints_,
					  						   PointCloud<BaseKeypoint> &keypoints)
{
	keypoints.clear(); // TODO: decidir vaciamos keypoints 
	for (const auto& keypoint2D : cv_keypoints_)
	{
		// TODO: reveer si no es mejor calcular un unico indice y luego 
		// hacer los chequeos
		// filter cloud outliers
		if (0 <= keypoint2D.pt.x && keypoint2D.pt.x < cloud->width && 
			0 <= keypoint2D.pt.y && keypoint2D.pt.y < cloud->height)
		{
			// WARNING : implicit conversion float to int
			const auto& point3D = cloud->at(keypoint2D.pt.x, keypoint2D.pt.y);

			keypoints.push_back(BaseKeypoint(point3D.x, point3D.y, point3D.z, 
						   					 keypoint2D.size, keypoint2D.angle, 
						   					 keypoint2D.response, keypoint2D.octave));
		}
	}
}

// void OrganizedFeaturesFinder::find(std::vector<OrganizedKeypoint> &keypoints, 
// 								Descriptors &keypoints)
// {
// 	keypointDetector->detect(image, cv_keypoints);	
// 	descriptorExtractor->compute(cv_keypoints, descriptors);
// 	convertKeypoints(cv_keypoints, keypoints); // TODO: definir convertKeypoints
// }

// void OrganizedFeaturesFinder::findKeypoints(std::vector<OrganizedKeypoint> &keypoints)
// {
// 	keypointDetector->detect(image, cv_keypoints);
// 	convertKeypoints(cv_keypoints, keypoints); // TODO: definir convertKeypoints
// }

// void OrganizedFeaturesFinder::computeDescriptors(const std::vector<OrganizedKeypoint> &keypoints, 
// 											  Descriptors &descriptors)
// {	
// 	convertKeypoints(keypoints, cv_keypoints);	
// 	descriptorExtractor->compute(cv_keypoints, descriptors);
// }

}// namespace features