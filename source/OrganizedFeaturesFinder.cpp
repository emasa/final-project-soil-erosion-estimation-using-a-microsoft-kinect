
#include <vector>

#include <pcl/common/common.h>
#include <pcl/range_image/range_image_planar.h>

#include <opencv2/features2d/features2d.hpp>
#include <Eigen/StdVector>

#include "../include/OrganizedFeaturesFinder.h"
#include "../include/Utils.h"

using namespace std;
using namespace pcl;
using namespace cv;

// TODO: chequar que cloud, parametros de la camara y detectores y extractores
// de keypoints han sido seteados antes de usarlos

namespace features 
{

// TODO: mover definiciones a otro archivo

template<typename PointT>
inline bool isGoodPoint(const PointT& pt)
{
	return pcl_isfinite(pt.x) && pcl_isfinite(pt.y) && pcl_isfinite(pt.z);
}

inline uint closer(double value)
{
	double floor_value = floor(value);
	return value - floor_value < 0.5 ? floor_value : floor_value + 1;
}

inline void project(const BaseKeypoint &keypoint3D, cv::KeyPoint &keypoint2D, 
					double fx, double fy, double cx, double cy)
{
	keypoint2D.pt.x = fx * keypoint3D.x / keypoint3D.z + cx;
	keypoint2D.pt.y = fy * keypoint3D.y / keypoint3D.z + cy;
	
	keypoint2D.size     = keypoint3D.size;
	keypoint2D.angle    = keypoint3D.angle;
	keypoint2D.response = keypoint3D.response;
	keypoint2D.octave   = keypoint3D.octave;
}

inline void backproject(const cv::KeyPoint &keypoint2D, BaseKeypoint &keypoint3D, 
						double z, double fx, double fy, double cx, double cy)
{
	keypoint3D.x = (keypoint2D.pt.x - cx) * z / fx;
	keypoint3D.y = (keypoint2D.pt.y - cy) * z / fy;
	keypoint3D.z = z;

	keypoint3D.size     = keypoint2D.size;
	keypoint3D.angle    = keypoint2D.angle;
	keypoint3D.response = keypoint2D.response;
	keypoint3D.octave   = keypoint2D.octave;
}

OrganizedFeaturesFinder::OrganizedFeaturesFinder(const cv::Ptr<cv::FeatureDetector> &keypointDetector_, 
					 					   	     const cv::Ptr<cv::DescriptorExtractor> &descriptorExtractor_) :
	keypointDetector(keypointDetector_), 
	descriptorExtractor(descriptorExtractor_),
	cameraIntrinsicsSeted(false) {}

OrganizedFeaturesFinder::OrganizedFeaturesFinder(double fx_, double fy_, double cx_, double cy_,
												 const cv::Ptr<cv::FeatureDetector> &keypointDetector_, 
					 					   	     const cv::Ptr<cv::DescriptorExtractor> &descriptorExtractor_) :
	fx(fx_), 
	fy(fy_), 
	cx(cx_), 
	cy(cy_),
	keypointDetector(keypointDetector_), 
	descriptorExtractor(descriptorExtractor_),
	cameraIntrinsicsSeted(true) {}

void OrganizedFeaturesFinder::setCloud(const PointCloud<PointXYZRGBA>::Ptr &cloud_)
{
	cloud = cloud_;
	// TODO: if cloud isn't organized raise UnorganizedPointCloudException
	rgbCloudToImage(*cloud, image);
}

void OrganizedFeaturesFinder::setKeypointDetector(const cv::Ptr<cv::FeatureDetector> &keypointDetector_) 
{
	keypointDetector = keypointDetector_;	
}

void OrganizedFeaturesFinder::setDescriptorExtractor(const cv::Ptr<cv::DescriptorExtractor> &descriptorExtractor_)
{
	descriptorExtractor = descriptorExtractor_;
}

void OrganizedFeaturesFinder::setCameraIntrinsics(double fx_, double fy_, double cx_, double cy_)
{
	fx = fx_; fy = fy_; cx = cx_; cy = cy_; cameraIntrinsicsSeted = true;
}

void OrganizedFeaturesFinder::computeKeypointsAndDescriptors(PointCloud<BaseKeypoint> &keypoints, 
				  				   							 Descriptors &descriptors)
{
	std::vector<cv::KeyPoint> cv_keypoints;
	keypointDetector->detect(image, cv_keypoints);
	computeDescriptors(cv_keypoints, descriptors, keypoints);
}

void OrganizedFeaturesFinder::computeKeypoints(PointCloud<BaseKeypoint> &keypoints)
{
	std::vector<cv::KeyPoint> cv_keypoints;
	keypointDetector->detect(image, cv_keypoints);

	std::vector<uint> indexes;
	convertKeypoints(cv_keypoints, keypoints, indexes, false);
}

void OrganizedFeaturesFinder::computeDescriptors(const PointCloud<BaseKeypoint> &keypoints, 
 											     Descriptors &descriptors, 
 											     PointCloud<BaseKeypoint> &remaining_keypoints)
{	
	vector<cv::KeyPoint> cv_keypoints;
	std::vector<uint> indexes;
	convertKeypoints(keypoints, cv_keypoints, indexes, false);	
	computeDescriptors(cv_keypoints, descriptors, remaining_keypoints);
}

void OrganizedFeaturesFinder::computeDescriptors(vector<cv::KeyPoint> &cv_keypoints, 
 											     Descriptors &descriptors, 
 											     PointCloud<BaseKeypoint> &remaining_keypoints)
{	
	// Keypoints for which a descriptor cannot be computed are removed
	// Sometimes new keypoints can be added	
	Descriptors tmp_descriptors;
	descriptorExtractor->compute(image, cv_keypoints, tmp_descriptors);
	
	std::vector<uint> indexes;
	convertKeypoints(cv_keypoints, remaining_keypoints, indexes);

	uint descriptors_size = tmp_descriptors.rows;
	if (descriptors_size == indexes.size())
	{
		descriptors = tmp_descriptors;
	} else 
	{   // filter descriptors
		descriptors.create(indexes.size(), tmp_descriptors.cols, tmp_descriptors.type());
		for (uint idx = 0; idx < indexes.size(); ++idx)
		{
			tmp_descriptors.row(indexes[idx]).copyTo(descriptors.row(idx));
		}
	}
}

void OrganizedFeaturesFinder::convertKeypoints(const vector<cv::KeyPoint> &cv_keypoints,
					  						   PointCloud<BaseKeypoint> &keypoints,
					  						   vector<uint> &indexes,
					  						   bool save_indexes)
{
	keypoints.clear(); 
	indexes.clear();

	BaseKeypoint keypoint3D;
	for (uint idx = 0; idx < cv_keypoints.size(); ++idx)
	{
		const auto &keypoint2D = cv_keypoints[idx];	
		// filter cloud outliers
		if (!(0 < keypoint2D.pt.x && keypoint2D.pt.x < cloud->width && 
			  0 <= keypoint2D.pt.y && keypoint2D.pt.y < cloud->height)) 
			continue;

		const auto& point3D = cloud->at(closer(keypoint2D.pt.x), 
										closer(keypoint2D.pt.y));

		if (!isGoodPoint(point3D)) continue; 

		// uses keypoint2D.x, keypoint2D.y for keep projection invariant between 			
		// keypoint3D and (keypoint2D, point3D.z)		
		backproject(keypoint2D, keypoint3D, point3D.z, fx, fy, cx, cy);
		keypoints.push_back(keypoint3D);
				
		if (save_indexes)
			indexes.push_back(idx);
	}
}

void OrganizedFeaturesFinder::convertKeypoints(const PointCloud<BaseKeypoint> &keypoints,
											   vector<cv::KeyPoint> &cv_keypoints,
											   vector<uint> &indexes,
					  						   bool save_indexes)
{
	cv_keypoints.clear(); 
	
	cv::KeyPoint keypoint2D;
	for (uint idx = 0; idx < keypoints.size(); ++idx) 
	{
		const BaseKeypoint& keypoint3D = keypoints[idx];	

		if (!isGoodPoint(keypoint3D)) continue;
		
		project(keypoint3D, keypoint2D, fx, fy, cx, cy);			
		cv_keypoints.push_back(keypoint2D);

		if (save_indexes)
			indexes.push_back(idx);
	}
}


}// namespace features