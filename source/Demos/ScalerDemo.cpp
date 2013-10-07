
#include <vector>
#include <string>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/conversions.h>
#include <pcl/common/file_io.h>
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

using namespace std;
using namespace pcl;
using namespace pcl::console;
using namespace pcl::visualization;
namespace po = boost::program_options;

void
transformToRealWorld(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud_in, 
				pcl::PointCloud<pcl::PointXYZRGBA> &cloud_out, 
				float scale, float real_base, float model_base)
{
	pcl::copyPointCloud(cloud_in, cloud_out);

	scale /= 1000.; // km

	for (auto &p : cloud_out.points)
	{
		p.x =   p.x * scale;
		p.y = - p.y * scale;
		p.z = (model_base - p.z) * scale + real_base;
	}
}

struct Helper 
{
	Helper() : x(0), y(0), z(0), finished(false){}

	void 
	callback(const PointPickingEvent &event, void* cookie)
	{
		int idx = event.getPointIndex();
		if (idx == -1)
		{
			PCL_INFO("Bad point. Select another one.\n"); return;
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


int main(int argc, char** argv)
{
	std::string input_path, output_path;
	
	float scale, real_base, model_base;

	po::options_description opts("Options");
	opts.add_options() 
		("help,h", "produce help message")
		("input,i", po::value<string>(&input_path), "set input cloud [required]")
		("output,o", po::value<string>(&output_path), "set output cloud [required]")
		("scale,s", po::value<float>(&scale), "set scale [required]")
		("real_base,r", po::value<float>(&real_base), "set real base [required]")
		("model_base,m", po::value<float>(&model_base), 
		 "set model base . Use model scale. [if not seted, use visualizer to pick a point]")
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

	if (!vm.count("input") || !vm.count("output") || 
		!vm.count("scale") || !vm.count("real_base")) 
	{
		PCL_ERROR("Set all required parameters.\n");
		std::cout << opts << std::endl; return -1;
	}

	pcl::PCLPointCloud2 blob;
	PointCloud<PointXYZRGBA>::Ptr cloud_in(new PointCloud<PointXYZRGBA>), 
								  cloud_out(new PointCloud<PointXYZRGBA>);

	if (!loadCloud(input_path, blob))
	{
		return -1;
	}
	pcl::fromPCLPointCloud2<pcl::PointXYZRGBA>(blob, *cloud_in);

	
	if (!vm.count("model_base"))
	{
		PCL_INFO("Creating visualizer... Select a point with shift + left click.\n");

		pcl::visualization::PCLVisualizer viewer;
		PointCloudColorHandlerRGBField<PointXYZRGBA> handler (cloud_in);
		viewer.addPointCloud(cloud_in, handler, "cloud");
		viewer.updatePointCloudPose("cloud", pcl::getTransformation(0, 0, 0, M_PI, 0, 0));

		Helper helper;
		viewer.registerPointPickingCallback<Helper>(&Helper::callback, helper);

		while (!helper.finished)
		{
			viewer.spinOnce (100);
			boost::this_thread::sleep (boost::posix_time::milliseconds (100));
		}
		viewer.close();

		model_base = helper.z;
	}
	
	TicToc tt;
	print_highlight ("Computing ");
	tt.tic ();

	transformToRealWorld(*cloud_in, *cloud_out, scale, real_base, model_base);
		
	print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); 
	print_value ("%d", cloud_out->size()); print_info (" points]\n");
	
	pcl::toPCLPointCloud2<pcl::PointXYZRGBA>(*cloud_out, blob);
	
	saveCloud(output_path, blob);

	return 0;
}