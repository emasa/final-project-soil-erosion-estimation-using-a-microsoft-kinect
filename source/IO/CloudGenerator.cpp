
#include <memory>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/exceptions.h>
#include <pcl/io/grabber.h>

#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>

#include "Common/Status.h"
#include "IO/CloudGenerator.h"

const int DEFAULT_SLEEP_TIME = 30; // 30 ms
const int DEFAULT_TOTAL_TIME = 2000; // 2 s

Status
CloudGenerator::setGrabberImplementation(const GrabberPtr &grabber)
{		
	try {
		// Register a callback function to the grabber...
		boost::function<void (const PointCloudOutConstPtr&)> frame_cb = 
			boost::bind (&CloudGenerator::onNewFrame, this, _1);		
		
		grabber->registerCallback (frame_cb);
		grabber_ = grabber;
	} catch (pcl::IOException &e) {
		return DEVICE_NOT_WORKING;
	}

	return SUCCESS;	
}

Status
CloudGenerator::startGenerating()
{
	try {
		if (grabber_->getFramesPerSecond() > 0) // streaming based
	  	{
	  		grabber_->start ();
		}
		grabber_on_ = true;
	} catch(pcl::IOException &e) {
		return DEVICE_NOT_WORKING;
	}
	return SUCCESS;
}

Status
CloudGenerator::stopGenerating()
{
	grabber_on_ = false;
	try {
		grabber_->stop ();
	} catch(pcl::IOException &e) {
		return DEVICE_NOT_WORKING;
	}
	return SUCCESS;
}

bool
CloudGenerator::isRunning()
{
	return grabber_on_;
}

void
CloudGenerator::onNewFrame (const PointCloudOutConstPtr &cloud)
{
	mutex_.lock ();
	if (trigger_)
	{
		trigger_ = false;
		most_recent_frame_ = cloud->makeShared(); // Make a copy of the frame
		fresh_frame_ = true;
	}
	mutex_.unlock ();
}

Status
CloudGenerator::generate(PointCloudOutPtr &cloud)
{
	// TODO : proteger con otro lock ?
	if ( !grabber_on_ )
	{
		return DEVICE_NOT_STARTED;
	}

	trigger_ = true;
	
	if (grabber_->getFramesPerSecond() == 0) grabber_->start(); // trigger based

	int sleep_time = 0;
	while (!fresh_frame_ && sleep_time < DEFAULT_TOTAL_TIME) 
	{
		boost::this_thread::sleep (boost::posix_time::milliseconds (DEFAULT_SLEEP_TIME));
		sleep_time += DEFAULT_SLEEP_TIME;
	}
	
	if (fresh_frame_)
	{
		cloud = most_recent_frame_->makeShared();
		most_recent_frame_.reset();
		fresh_frame_ = false;
		return SUCCESS;
	} else {
		return CAPTURER_ERROR;
	} 
}
