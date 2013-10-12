
#ifndef EROSION_MODEL_ADJUSTMENT_TOOL
#define EROSION_MODEL_ADJUSTMENT_TOOL

#include <limits>
#include <memory>
#include <iostream>

#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/crop_box.h>

class ErosionModelAdjustmentTool
{

public:
	
	std::shared_ptr<ErosionModelAdjustmentTool> Ptr;
	std::shared_ptr<const ErosionModelAdjustmentTool> ConstPtr;

	ErosionModelAdjustmentTool()
	: border_percent_(0.025)
	, scale_(1)
	, model_z_(0)
	, real_z_(0)
 	, min_bounds_()
	, max_bounds_()	
	, cloud_in_()
	, crop_box_()
	{
		for (int idx = 0 ; idx < 3 ; ++idx)
		{
			min_bounds_(idx) =  std::numeric_limits<float>::lowest(); 
		 	max_bounds_(idx) =  std::numeric_limits<float>::max(); 
	 	}
	}

	void 
	setBorderPercent(float border_percent)
	{
		assert(0 <= border_percent); assert(border_percent <= 1); 
		border_percent_ = border_percent;	
	}

	void 
	setScale(float scale)
	{
		assert(scale > 0);
		scale_ = scale;
	}

	void 
	setModelFixedHeight(float model_z)
	{
		model_z_ = model_z;	
	}

	void 
	setRealFixedHeight(float real_z)
	{
		real_z_ = real_z;	
	}

	void
	setModelHeightBounds(float min_z, float max_z)
	{
		min_bounds_(2) = min_z;
		max_bounds_(2) = max_z;
	}

	void
	setInputCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_in)
	{
		cloud_in_ = cloud_in;	
	}

	void
	compute(pcl::PointCloud<pcl::PointXYZRGBA> &cloud_out)
	{
		assert( cloud_in_ );

		computeGroundBounds();
		
		crop_box_.setMin(min_bounds_);
		crop_box_.setMax(max_bounds_);		
		crop_box_.setInputCloud(cloud_in_);
		crop_box_.filter(cloud_out);

		transform(cloud_out, cloud_out);
	}

private:
	void
	computeGroundBounds()
	{
		pcl::PointXYZRGBA min_pt, max_pt;

		pcl::getMinMax3D(*cloud_in_, min_pt, max_pt);

		float x_delta = std::fabs(border_percent_ * (max_pt.x - min_pt.x));
		float y_delta = std::fabs(border_percent_ * (max_pt.y - min_pt.y));

		min_bounds_(0) = min_pt.x + x_delta; // min_x
		max_bounds_(0) = max_pt.x - x_delta; // max_x
		min_bounds_(1) = min_pt.y + y_delta; // min_y
		max_bounds_(1) = max_pt.y - y_delta; // max_y
	}

	void
	transform(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud_in, 
			  pcl::PointCloud<pcl::PointXYZRGBA> &cloud_out) 
	{
		pcl::copyPointCloud(cloud_in, cloud_out);
		for (auto &p : cloud_out.points)
		{
			p.x =   p.x * scale_;
			p.y = - p.y * scale_;
			p.z = (model_z_ - p.z) * scale_ + real_z_;
		}
	}

	float border_percent_;
	float scale_;
	float model_z_, real_z_;

	Eigen::Vector4f min_bounds_, max_bounds_;

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_in_;

	pcl::CropBox<pcl::PointXYZRGBA> crop_box_;

};

#endif // EROSION_MODEL_ADJUSTMENT_TOOL