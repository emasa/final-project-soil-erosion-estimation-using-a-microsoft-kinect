
#ifndef CLOUD_GENERATOR_H
#define CLOUD_GENERATOR_H

#include <memory>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/grabber.h>

#include "Common/Status.h"

class CloudGenerator
{
public:
	typedef std::shared_ptr<CloudGenerator> Ptr;
	typedef std::shared_ptr<const CloudGenerator> ConstPtr;

	typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudOut;
	typedef PointCloudOut::Ptr PointCloudOutPtr;
	typedef PointCloudOut::ConstPtr PointCloudOutConstPtr;	

	typedef std::shared_ptr<pcl::Grabber> GrabberPtr;

public:
	CloudGenerator ()
	  : grabber_ ()
	  , most_recent_frame_ ()
	  , trigger_ (false)
	  , fresh_frame_ (false)
	  , grabber_on_(false)	
	{}
	
	Status
	startGenerating();

	Status
	stopGenerating();

	Status
	generate(PointCloudOutPtr &cloud);

	Status 
	setGrabberImplementation(const std::shared_ptr<pcl::Grabber> &grabber);
private:
	void
	onNewFrame (const PointCloudOutConstPtr &cloud);

	std::shared_ptr<pcl::Grabber> grabber_;

	boost::mutex mutex_;
	
	PointCloudOutPtr most_recent_frame_;	

	bool trigger_, fresh_frame_, grabber_on_;
};

#endif // CLOUD_GENERATOR_H