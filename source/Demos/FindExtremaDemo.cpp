
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
#include <pcl/search/kdtree.h>

#include <boost/program_options/options_description.hpp>
#include <boost/program_options/positional_options.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>

#include "Utils/Utils.h"

using namespace std;
using namespace pcl;
using namespace pcl::console;
namespace po = boost::program_options;

void
findExtrema(const pcl:PointCloud<PointXYZRGBA> &cloud_in,
			pcl:PointCloud<PointXYZRGBA> &cloud_minima,
			pcl:PointCloud<PointXYZRGBA> &cloud_maxima)
{
	cloud_minima.clear();
	cloud_maxima.clear();

	vector<int> cloud_minima_indixes, cloud_maxima_indixes;

	int k = 25;
	pcl::search::KdTree tree(false);
	tree.setInputCloud(cloud_in.makeShared());

	std::vector< int > k_indices(k); 
	std::vector< float > k_sqr_distances(k);
	for (int idx = 0 ; idx < static_cast<int>(cloud_in.size()) ; ++idx)
	{
		int nn_found = tree.nearestKSearch(cloud_in[idx], k, k_indices, k_sqr_distances);
		bool is_local_min = false, is_local_max = false;
		for (int ngb_idx = 0 ; ngb_idx < nn_found ; ++ngb_idx)
		{
			// TODO: COMPLETAR ACA
		}
	}
}

int main(int argc, char** argv)
{
	std::string input_path, output_maxima_path, output_minima_path;
	
	po::options_description opts("Options");
	opts.add_options() 
		("help,h", "produce help message")
		("input,i", po::value<string>(&input_path), "set input cloud [required]")
		("output-max,x", po::value<string>(&output_maxima_path), "set output maxima cloud [required]")
		("output-min,n", po::value<string>(&output_minima_path), "set output minima cloud [required]")
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

	if (!vm.count("input") || !vm.count("output")) 
	{
		PCL_ERROR("Set all required parameters.\n");
		std::cout << opts << std::endl; return -1;
	}

	pcl::PCLPointCloud2 blob;
	PointCloud<PointXYZRGBA> cloud_in, cloud_out;
	
	if (!loadCloud(input_path, blob))
	{
		return -1;
	}

	pcl::fromPCLPointCloud2<pcl::PointXYZRGBA>(blob, cloud_in);
	
	TicToc tt;
	print_highlight ("Computing ");
	tt.tic ();

	pcl::PointCloud<PointXYZRGBA> cloud_minima, cloud_maxima;
	findExtrema(cloud_in, cloud_minima, cloud_maxima);
		
	print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); 
	print_value ("%d", cloud_maxima.size() + cloud_minima.size()); print_info (" points]\n");
	
	pcl::toPCLPointCloud2<pcl::PointXYZRGBA>(cloud_minima, blob);
	saveCloud(output_minima, blob);

	pcl::toPCLPointCloud2<pcl::PointXYZRGBA>(cloud_maxima, blob);
	saveCloud(output_maxima_path, blob);

	return 0;
}