
#include <vector>
#include <string>
#include <cstdlib>

#include <boost/program_options/options_description.hpp>
#include <boost/program_options/positional_options.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>

#include "GlobalRegistration/GlobalRegistration.h"
#include "GlobalRegistration/GlobalRegistrationFactory.h"
#include "Tools/RegistrationTool.h"

using namespace std;
using namespace pcl;
namespace po = boost::program_options;

int main(int argc, char* argv[])
{
	vector<string> clouds;
	string dir, mode;

	po::options_description opts("Allowed options");
	opts.add_options() 
		("help,h", "produce help message")

		("output-dir,d", po::value<string>(&dir), "set output directory [required]")

		("mode,m", po::value<string>(&mode)->default_value("camera"), 
		 "set input mode. valid options : camera [default], files, camera_recovery")
		
		("input,i", po::value<vector<string>>(&clouds)->multitoken(), 
		 "set cloud_1 .. cloud_n [required on files and camera_recovery modes]")
		
		("backup,b", "enable original cloud backup [default : disabled]")
		;

	po::basic_command_line_parser<char> parser(argc, argv);
	parser.options(opts);
	parser.allow_unregistered();

	po::variables_map vm;
	po::store(parser.run(), vm);
	vm.notify();

	if (vm.count("help")) 
	{
    	std::cout << opts
    			  << "\n  on camera or camera-recovery modes press : \n" 
    	          << "    space : for capture and register a new cloud\n"
    	          << "    p, P  : for finish registration and save clouds\n"
    	          << std::endl;
    	return 0;
	}

	if (!vm.count("output-dir")) 
	{
		PCL_ERROR("Output directory was not set.\n"); return -1;
	}

	bool backup = vm.count("backup");

	auto algorithm = GlobalRegistrationFactory().ORBAndSURF();
	RegistrationTool<std::decltype(algorithm)::element_type> tool(backup);
	tool.setRegistrationAlgorithm(algorithm);
	
	if (!tool.setUpOutputDirectory(dir))
	{
		return -1;
	};

	if (mode == "camera")
	{
		tool.registerFromCamera();
	} else if (mode == "files") 
	{
		if (!vm.count("input") || clouds.size() < 2 )
		{
			PCL_ERROR("At least 2 input clouds are required\n"); return -1;
		} 
		
		tool.registerFromFiles(clouds);
	} else if (mode == "camera_recovery")
	{
		if (!vm.count("input") || clouds.size() == 0 )
		{
			PCL_ERROR("At least 1 input clouds is required\n"); return -1;
		}

		tool.registerFromFilesAndThenFromCamera(clouds);
	} else
	{
		PCL_ERROR("Invalid mode. Valid options are : camera files camera_recovery\n"); return -1;
	}

	return 0;
}