
#include <vector>
#include <string>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/common/file_io.h>
#include <pcl/filters/filter.h>
#include <pcl/console/print.h>
#include <pcl/console/time.h>
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <boost/thread/thread.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/positional_options.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>

#include "Utils/Utils.h"
#include "Tools/ErosionModelAdjustmentTool.h"

using namespace std;
using namespace pcl;
using namespace pcl::console;
using namespace pcl::visualization;
namespace po = boost::program_options;

struct Helper 
{
	Helper() : x(0), y(0), z(0), finished(false){}

	void 
	callback(const PointPickingEvent &event, void* cookie)
	{
		if (event.getPointIndex() == -1)
		{
			PCL_INFO("Bad point. Select another one.\n"); 
			return;
		}
		event.getPoint(x, y, z);

		std::string answer;
		while (answer != "Y" && answer != "N")
		{
			PCL_INFO("Are you sure you want to set model_base : %f. Y/N ? ", z) ;	
			std::cin >> answer;
		}

		if (answer == "Y") 
		{
			finished = true;
		} else {
			PCL_INFO("Select another point.\n");
		}
	}

	float x, y, z;
	bool finished;
};

float 
getModelBaseUsingVisualization(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud)
{
	PCL_INFO("Creating visualizer... Select a point with shift + left click.\n");

	pcl::visualization::PCLVisualizer viewer;
	PointCloudColorHandlerRGBField<PointXYZRGBA> handler (cloud);	
	viewer.addPointCloud(cloud, handler, "cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_IMMEDIATE_RENDERING, 1.0, "cloud");

	viewer.updatePointCloudPose("cloud", pcl::getTransformation(0, 0, 0, M_PI, 0, 0));

	Helper helper;
	viewer.registerPointPickingCallback<Helper>(&Helper::callback, helper);

	while (!helper.finished)
	{
		viewer.spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::milliseconds (100));
	}
	viewer.close();

	return helper.z;
}
 
int main(int argc, char** argv)
{
	std::string output_path;
	std::vector<std::string> input_paths;
	float scale, real_base, model_base;
	float border_percent;
	float min_z, max_z;

	po::options_description opts("Options");
	opts.add_options() 
		("help,h", "produce help message")
		("input,i", po::value<std::vector<string>>(&input_paths)->multitoken(), "set cloud_1..cloud_n [required]")
		("output,o", po::value<string>(&output_path), "set output cloud [required]")
		("scale,s", po::value<float>(&scale), "set scale [required]")
		("real_base,r", po::value<float>(&real_base), "set real base [required]")
		("model_base,m", po::value<float>(&model_base), 
		 "set model base. [if not seted, use visualizer to pick a point]")
		 // "set model base. [required]")
		("drop_border,p", po::value<float>(&border_percent)->default_value(2.5), "border percent to be eliminated")
		("min_z,n", po::value<float>(&min_z), "set min z bound")
		("max_z,x", po::value<float>(&max_z), "set max z bound")
		;

	po::basic_command_line_parser<char> parser(argc, argv);
	parser.options(opts);
	parser.allow_unregistered();
	po::variables_map vm;
	po::store(parser.run(), vm);
	vm.notify();

	if (vm.count("help")) 
	{
    	std::cout << opts << std::endl; return 0;
	}

	if (  !vm.count("input") 
	   || !vm.count("output") 
	   || !vm.count("scale") 
	   || !vm.count("real_base")
	   || !vm.count("min_z")
	   || !vm.count("max_z")
	   ) 
	{
		PCL_ERROR("Set all required parameters.\n");
		std::cout << opts << std::endl; return -1;
	}
	
	std::vector<PointCloud<PointXYZRGBA>::Ptr> input_clouds;
	pcl::PCLPointCloud2 blob;
	
	for (const auto& path : input_paths)
	{
		if (loadCloud(path, blob))
		{
			// TODO : chequar por campos XYZRGBA
			PointCloud<pcl::PointXYZRGBA>::Ptr cloud_in(new PointCloud<pcl::PointXYZRGBA>);			
			fromPCLPointCloud2<pcl::PointXYZRGBA>(blob, *cloud_in);
			input_clouds.push_back(cloud_in);
		} else 
		{		
			PCL_ERROR("IO error found with %s. skipping cloud.\n", path.c_str());
		}
	}	
	
	bool use_visualization = !vm.count("model_base");
	if (use_visualization)
	{
		PointCloud<pcl::PointXYZRGBA>::Ptr concateneted_cloud_in(new PointCloud<pcl::PointXYZRGBA>);
		for (const auto& cloud_in : input_clouds)
		{
			*concateneted_cloud_in += *cloud_in;
		}
		model_base = getModelBaseUsingVisualization(concateneted_cloud_in);	
	}
	
	TicToc tt;
	print_highlight ("Computing ");
	tt.tic ();

	ErosionModelAdjustmentTool tool;
	tool.setScale(scale);
	tool.setRealFixedHeight(real_base);
	tool.setModelFixedHeight(model_base);
	tool.setModelHeightBounds(min_z, max_z);
	tool.setBorderPercent( border_percent / 100. );	

	// TODO: ver como manejamos los nan's en x,y,z

	PointCloud<PointXYZRGBA>::Ptr cloud_in(new PointCloud<PointXYZRGBA>);
	PointCloud<PointXYZRGBA> cloud_out, partial_cloud_out;

	for (const auto& cloud_in : input_clouds)
	{
		tool.setInputCloud(cloud_in);
		tool.compute(partial_cloud_out);
		cloud_out += partial_cloud_out;
	}	

	print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); 
	print_value ("%d", cloud_out.size()); print_info (" points]\n");
	
	pcl::toPCLPointCloud2<pcl::PointXYZRGBA>(cloud_out, blob);
	saveCloud(output_path, blob);

	return 0;
}
