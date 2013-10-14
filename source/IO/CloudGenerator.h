
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
	  , new_frame_captured_(false)
	  , is_running_(false)
	  {}
	
	Status
	startGenerating();

	Status
	stopGenerating();

	Status
	generate(PointCloudOutPtr &cloud);

	bool
	isRunning() { return is_running_; }

	void 
	setGrabber(const GrabberPtr &grabber) { grabber_ = grabber; }

	GrabberPtr
	getGrabber() { return grabber_; }

private:
	void
	onNewFrame (const PointCloudOutConstPtr &cloud);

	std::shared_ptr<pcl::Grabber> grabber_;

	PointCloudOutPtr most_recent_frame_;	
	
	bool new_frame_captured_, is_running_;

	boost::mutex mutex_;
};

#endif // CLOUD_GENERATOR_H