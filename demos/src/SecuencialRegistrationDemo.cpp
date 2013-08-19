
#include <iostream>
#include <vector>
#include <memory>

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>

#include <pcl/visualization/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/filters/voxel_grid.h>

#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_lm.h>

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

#include <boost/thread/thread.hpp>

#include "Utils.h"
#include "Features.h"

#include "FeaturesFinder.h"
#include "OpenCVFeaturesFinder.h"
#include "PCLFeaturesFinder.h"

using namespace pcl;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::visualization;
using namespace pcl::registration;
using namespace cv;

using namespace features;

template<typename PointInT, typename KeypointT> 
typename OpenCVFeaturesFinder<PointInT, KeypointT>::Ptr 
createOpenCVFeaturesFinder(const char* feature_detector, const char* descriptor_extractor) 
{
	auto opencvFinder = std::make_shared<OpenCVFeaturesFinder<PointInT, KeypointT>>(); 
	opencvFinder->setKeypointDetector(FeatureDetector::create(feature_detector));
	opencvFinder->setDescriptorExtractor(DescriptorExtractor::create(descriptor_extractor));

	return opencvFinder;
}

int main(int argc, char* argv[])
{
	if (argc < 4)
	{
		cerr << "use: program final_cloud cloud_file0 .. cloud_fileN" << endl;
		return -1;
	} 

	std::vector<PointCloud<PointXYZRGBA>::Ptr> vec_cloud;
	
	for (int idx = 2; idx < argc ; ++idx)
	{
		PointCloud<PointXYZRGBA>::Ptr cloud (new PointCloud<PointXYZRGBA>);
		if (loadPCDFile(argv[idx], *cloud) == -1)
		{
			cerr << "coudn't read file " << argv[idx] << endl;
			return -1;
		}
		vec_cloud.push_back(cloud);
		cout << "loading file : " << argv[idx]
			 << "# total points " << cloud->size() << endl;
	}
	
	cv::initModule_nonfree(); // required for SURF
	
	// uses ORB because default parameters bring lesser keypoints than SURF (faster)
	// uses SURF because flann kdtree support real descriptors (default)
	auto opencvFinder = createOpenCVFeaturesFinder<PointXYZRGBA, PointWithScale>("ORB", "SURF");
	// kdtree flann based matcher

	PointCloud<PointWithScale>::Ptr keypoints_src(new PointCloud<PointWithScale>);
	PointCloud<PointWithScale>::Ptr keypoints_tgt(new PointCloud<PointWithScale>);
	Descriptors descriptors_src, descriptors_tgt;

	// PointCloud<PointWithScale>::Ptr tmp_keypoints_src(new PointCloud<PointWithScale>);
	// PointCloud<PointWithScale>::Ptr tmp_keypoints_tgt(new PointCloud<PointWithScale>);

	auto matcher = DescriptorMatcher::create("FlannBased");

	CorrespondenceRejectorSampleConsensus<PointWithScale> rejector;
	rejector.setMaximumIterations(500);

	PointCloud<PointXYZRGBA>::Ptr transformed_cloud (new PointCloud<PointXYZRGBA>);
	PointCloud<PointXYZRGBA>::Ptr final_cloud (new PointCloud<PointXYZRGBA>(*vec_cloud[0]));	
	Eigen::Matrix4f final_transformation = Eigen::Matrix4f::Identity(), pairTransformation;
	
	opencvFinder->setInputCloud(vec_cloud[0]);
	opencvFinder->computeKeypointsAndDescriptors(*keypoints_src, descriptors_src);
	
	// opencvFinder->computeKeypoints(*tmp_keypoints_src);
	// opencvFinder->computeDescriptors(*tmp_keypoints_src, descriptors_src, *keypoints_src);

	for (int idx = 1; idx < vec_cloud.size(); ++idx) 
	{
		opencvFinder->setInputCloud(vec_cloud[idx]);
		opencvFinder->computeKeypointsAndDescriptors(*keypoints_tgt, descriptors_tgt);

		// opencvFinder->computeKeypoints(*tmp_keypoints_tgt);
		// opencvFinder->computeDescriptors(*tmp_keypoints_tgt, descriptors_tgt, *keypoints_tgt);
		
		cout << " registering pair : " << idx << endl;
		cout << "# keypoints src : " << keypoints_src->size() << endl;
		cout << "# descriptors src : " << descriptors_src.rows << endl;
		cout << "# keypoints tgt : " << keypoints_tgt->size() << endl;
		cout << "# descriptors tgt : " << descriptors_tgt.rows << endl;

		std::vector<DMatch> cv_matches;
		matcher->match(descriptors_src, descriptors_tgt, cv_matches);

	 	Correspondences matches, cleaned_matches;
	 	for (const auto & cv_match : cv_matches)
	 	{
	 		matches.emplace_back(cv_match.queryIdx, cv_match.trainIdx, cv_match.distance);
	 	}

		cout << "# initial matches : " << matches.size() << endl;

	 	rejector.setInputSource(keypoints_src);
		rejector.setInputTarget(keypoints_tgt);	
		rejector.getRemainingCorrespondences(matches, cleaned_matches);		

		cout << "# filtered matches with ransac : " << cleaned_matches.size() << endl;
		
		// TransformationEstimationSVD<PointWithScale, PointWithScale> pose_refiner;
		TransformationEstimationLM<PointWithScale, PointWithScale> pose_refiner;

		pose_refiner.estimateRigidTransformation(*keypoints_src, *keypoints_tgt, 
												 cleaned_matches, pairTransformation);

		cout << "estimated rigid transformation with LM" << endl;

		// inverse pairTransformation because cloud 0 is the reference
		final_transformation = final_transformation * pairTransformation.inverse();

		transformPointCloud<PointXYZRGBA, float>(*vec_cloud[idx], *transformed_cloud, final_transformation);

		// downsamples cloud to save space and reduce visualizer  	
	 	VoxelGrid<PointXYZRGBA> grid;
	 	// create 3d box with size leaf_size x leaf_size x leaf_size
	 	// and replace its values with one weighted value
	 	float leaf_size = 0.005;	
	    grid.setLeafSize (leaf_size, leaf_size, leaf_size);
	    grid.setInputCloud (transformed_cloud);
	    
		PointCloud<PointXYZRGBA> downsampled_transformed_cloud;
	    grid.filter (downsampled_transformed_cloud);

		cout  << "# total points after downsampled : " 
			  << downsampled_transformed_cloud.size() << endl;

 		*final_cloud += downsampled_transformed_cloud;

 		keypoints_src.swap(keypoints_tgt);
 		descriptors_src = descriptors_tgt;
 		descriptors_tgt = Descriptors();
 	}

	cout << "# total points final cloud : " << final_cloud->size() << endl ;

 	// save cloud to disk
 	savePCDFile<PointXYZRGBA>(argv[1], *final_cloud, true);

	// visualization setup
	PCLVisualizer viewer;
	// add rgb cloud    
    PointCloudColorHandlerRGBField<PointXYZRGBA> handler (final_cloud);
	viewer.addPointCloud<PointXYZRGBA>(final_cloud, handler, "cloud");
	// improves point of view
    viewer.updatePointCloudPose("cloud", viewer.getViewerPose());
 	// displays cloud until a key is pressed
	while (!viewer.wasStopped())
	{
		viewer.spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}

	return 0;
}