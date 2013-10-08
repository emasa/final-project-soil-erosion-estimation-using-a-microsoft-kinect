
#include <vector>
#include <string>

#include <boost/thread/thread.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>

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
#include <pcl/search/kdtree.h>

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
findExtrema(const pcl::PointCloud<PointXYZRGBA> &cloud_in,
			pcl::PointCloud<PointXYZRGBA> &cloud_minima,
			pcl::PointCloud<PointXYZRGBA> &cloud_maxima,
			float radius)
{
	cloud_minima.clear();
	cloud_maxima.clear();

	std::vector<int> cloud_minima_indices, cloud_maxima_indices;

	int k_max = 100;
	pcl::search::KdTree<pcl::PointXYZRGBA> tree(true);
	tree.setInputCloud(cloud_in.makeShared());

	std::vector< int > k_indices(k_max); 
	std::vector< float > k_sqr_distances(k_max);
	for (int idx = 0 ; idx < static_cast<int>(cloud_in.size()) ; ++idx)
	{
		int nn_found = tree.radiusSearch(cloud_in[idx], radius, k_indices, k_sqr_distances, k_max);
		bool is_local_min = true, is_local_max = true;
		for (int tmp_idx = 0; tmp_idx < nn_found && ( is_local_max || is_local_min ); ++tmp_idx)
		{
			int ngb_index = k_indices[tmp_idx];
			if (idx == ngb_index) continue;

			if (cloud_in[idx].z < cloud_in[ngb_index].z) 
			{
				is_local_max = false;
			} else if (cloud_in[idx].z > cloud_in[ngb_index].z)
			{
				is_local_min = false;
			}
		}
		if (is_local_min && nn_found > 0) 
		{
			cloud_minima_indices.push_back(idx);
		}
		if (is_local_max && nn_found > 0) 
		{
			cloud_maxima_indices.push_back(idx);
		}
	}

	pcl::copyPointCloud(cloud_in, cloud_minima_indices, cloud_minima);	
	pcl::copyPointCloud(cloud_in, cloud_maxima_indices, cloud_maxima);
}

int main(int argc, char** argv)
{
	std::string input_path, output_maxima_path, output_minima_path;
	
	float radius;

	po::options_description opts("Options");
	opts.add_options() 
		("help,h", "produce help message")
		("input,i", po::value<string>(&input_path), "set input cloud [required]")
		("output-max,x", po::value<string>(&output_maxima_path), "set output maxima cloud [required]")
		("output-min,n", po::value<string>(&output_minima_path), "set output minima cloud [required]")
		("radius,r", po::value<float>(&radius), "set radius search [required]")
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

	if (!vm.count("input")      || 
		!vm.count("output-min") || 
		!vm.count("output-max") || 
		!vm.count("radius")) 
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
	
	TicToc tt;
	print_highlight ("Computing ");
	tt.tic ();

	PointCloud<PointXYZRGBA>::Ptr cloud_minima(new PointCloud<PointXYZRGBA>), 
								  cloud_maxima(new PointCloud<PointXYZRGBA>);

	findExtrema(*cloud_in, *cloud_minima, *cloud_maxima, radius);
		
	print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); 
	print_value ("%d", cloud_maxima->size() + cloud_minima->size()); print_info (" points]\n");
	
	pcl::toPCLPointCloud2<pcl::PointXYZRGBA>(*cloud_minima, blob);
	saveCloud(output_minima_path, blob);

	pcl::toPCLPointCloud2<pcl::PointXYZRGBA>(*cloud_maxima, blob);
	saveCloud(output_maxima_path, blob);

	PCLVisualizer viewer;
	displayKeypoints<PointXYZRGBA, PointXYZRGBA>(cloud_in, cloud_minima, viewer, 
                      							 pcl::PointRGB(0, 255, 0), 2, 
                   							     "cloud_1", "cloud_minima");

	displayKeypoints<PointXYZRGBA, PointXYZRGBA>(cloud_in, cloud_maxima, viewer, 
                      							 pcl::PointRGB(0, 0, 255), 2, 
                   							     "cloud_2", "cloud_maxima");

	while (!viewer.wasStopped())
	{
		viewer.spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::milliseconds (100));
	}

	return 0;
}