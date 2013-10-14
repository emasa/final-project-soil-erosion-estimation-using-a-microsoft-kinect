
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
CloudGenerator::startGenerating()
{
	assert(grabber_);

	try {
		if (grabber_->getFramesPerSecond() > 0) // streaming based
	  	{
	  		grabber_->start ();
		}
		is_running_ = true;
	} catch(pcl::IOException &e) {
		return DEVICE_NOT_WORKING;
	}
	return SUCCESS;
}

Status
CloudGenerator::stopGenerating()
{
	is_running_ = false;
	try {
		grabber_->stop ();
	} catch(pcl::IOException &e) {
		return DEVICE_NOT_WORKING;
	}
	return SUCCESS;
}

void
CloudGenerator::onNewFrame (const PointCloudOutConstPtr &cloud)
{
	mutex_.lock ();
	if (!new_frame_captured_)
	{
		most_recent_frame_ = cloud->makeShared(); // Make a copy of the frame
		new_frame_captured_ = true;
	}
	mutex_.unlock ();
}

Status
CloudGenerator::generate(PointCloudOutPtr &cloud)
{
	assert(grabber_);
	
	if ( !is_running_ )
	{
		return DEVICE_NOT_STARTED;
	}

	new_frame_captured_ = false;

	// Register a callback function to the grabber...
	boost::function<void (const PointCloudOutConstPtr&)> frame_cb = 
		boost::bind (&CloudGenerator::onNewFrame, this, _1);		
	
	auto connection = grabber_->registerCallback (frame_cb);
	if ( !connection.connected() ) 
	{
		return CAPTURER_ERROR;
	}

	if (grabber_->getFramesPerSecond() == 0) grabber_->start(); // trigger based

	int sleep_time = 0;
	while (!new_frame_captured_ && sleep_time < DEFAULT_TOTAL_TIME) 
	{
		boost::this_thread::sleep (boost::posix_time::milliseconds (DEFAULT_SLEEP_TIME));
		sleep_time += DEFAULT_SLEEP_TIME;
	}

	connection.disconnect();
	
	if (!new_frame_captured_ || !most_recent_frame_)
	{
		return CAPTURER_ERROR;
	} 

	cloud = most_recent_frame_;
	most_recent_frame_.reset();	
	
	return SUCCESS;
}
