
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <cstdlib>

#include <boost/program_options/options_description.hpp>
#include <boost/program_options/positional_options.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/filesystem.hpp>

#include <pcl/common/file_io.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/exceptions.h>

#include "GlobalRegistration/GlobalRegistration.h"
#include "GlobalRegistration/GlobalRegistrationFactory.h"
#include "Tools/RegistrationTool.h"

using namespace std;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

const string CAM_PARAMS_FILE = "camera_parameters.txt";
const string CLOUDS_DIR = "backup";

bool
getCameraParametersFromDevice(double &fx, double &fy, double &cx, double &cy, 
							  const string& device_id="")
{
	try 
	{
		// ugly workaround : see pcl::OpenNIGrabber code  
		auto device = pcl::OpenNIGrabber(device_id).getDevice();
		// OpenNIGrabber::setupDevice from line 386
		float depth_width  = device->getDepthOutputMode().nXRes;
		float depth_height = device->getDepthOutputMode().nYRes;
		float image_width  = device->getImageOutputMode().nXRes;
		float image_height = device->getImageOutputMode().nYRes;

		// OpenNIGrabber::convertToXYZRGBPointCloud from line 637 
		float cloud_width  = std::max (image_width, depth_width);
		float cloud_height = std::max (image_height, depth_height);
		fx = device->getImageFocalLength (depth_width);
		fy = device->getImageFocalLength (depth_width);
		cx = ((float)cloud_width - 1.f) / 2.f;
		cy = ((float)cloud_height - 1.f) / 2.f;

		// doesn't works. it gives fx = fy = cx = cy = nan.
		// pcl::OpenNIGrabber().getRGBCameraIntrinsics(fx, fy, cx, cy);
	} catch(pcl::IOException &e)
	{
		return false;
	}

	return true;
}

bool
getCameraParametersFromFile(double &fx, double &fy, double &cx, double &cy, 
							const string& filename)
{
	std::fstream file(filename);	
	if ( file.is_open() )
	{
		file >> fx >> fy >> cx >> cy;	
	}

	return !( file.fail() || file.bad() );
}


int main(int argc, const char* argv[])
{
	string mode;
	string input_dir, output_dir;

	po::options_description opts("Allowed options");
	opts.add_options() 
		("help,h", "produce help message")

		("output-dir,o", po::value<string>(&output_dir), "set output directory [required]")

		("mode,m", po::value<string>(&mode)->default_value("camera"), 
		 "set input mode. valid options : camera [default], files, camera_recovery")

		("input-dir,i", po::value<string>(&input_dir), 
		 "set input dir. It must contains the backup folder[required on files and camera_recovery modes]")

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
    	          << "    s, S  : for a registration checkpoint (save aligned clouds)\n"
    	          << std::endl;
    	return 0;
	}

	if (mode != "camera" && mode != "files" && mode != "camera_recovery")
	{
		PCL_ERROR("Invalid mode. Valid options are : camera files camera_recovery\n"); 
		return -1;
	}

	fs::path input_dir_path;
	vector<string> clouds;

 	if ( mode == "files" || mode == "camera_recovery" ) 
	{	
		if ( !vm.count("input-dir") )
		{		
			PCL_ERROR("Set a valid input directory\n"); 
			return -1;
		}

		input_dir_path = fs::path(input_dir);
		fs::path clouds_dir_path = (input_dir_path / CLOUDS_DIR);
		pcl::getAllPcdFilesInDirectory (clouds_dir_path.c_str(), clouds);
		for (auto &cloud : clouds) 
		{
			cloud = (clouds_dir_path / cloud).c_str();
		}

		int min_clouds = mode == "files" ? 2 : 1;
		if ( clouds.size() < min_clouds )
		{
			PCL_ERROR("Min number of clouds required : %i\n", min_clouds); 
			return -1;
		}
	}

	double fx, fy, cx, cy;
	if ( mode == "files" )
	{
		fs::path input_cam_params = input_dir_path / CAM_PARAMS_FILE;
		if ( !getCameraParametersFromFile(fx, fy, cx, cy, input_cam_params.c_str()) )
		{
			PCL_ERROR("Couldn't set camera parameters from %s\n", input_cam_params.c_str());
			return -1;
		}
	} else 
	{
		if ( !getCameraParametersFromDevice(fx, fy, cx, cy) )
		{
			PCL_ERROR("Couldn't set camera parameters from camera\n");
			return -1;
		}
	}	

	PCL_INFO("Camera parameters fx=%f fy=%f cx=%f cy=%f\n", fx, fy, cx, cy);

	bool backup = vm.count("backup");

	if ( !vm.count("output-dir") ) 
	{
		PCL_ERROR("Set a valid output directory\n"); 
		return -1;
	}

	fs::path output_dir_path(output_dir);

	auto algorithm = GlobalRegistrationFactory().ORBAndSURF(fx, fy, cx, cy);	
	RegistrationTool<std::decltype(algorithm)::element_type> tool(backup);
	tool.setRegistrationAlgorithm(algorithm);
	
	if ( !tool.setUpOutputDirectory(output_dir_path.c_str()) ) 
	{
		return -1;
	}

	if ( backup )
	{
		fs::path output_cam_params = output_dir_path / CAM_PARAMS_FILE;	
		std::fstream file(output_cam_params.c_str(), ios_base::out | ios_base::trunc);
		
		if ( file.is_open() )
		{
			file << fx << " " << fy << " " << cx << " " << cy << std::endl;	
		} else 
		{
			PCL_ERROR("Couldn't write camera parameters on %s\n", output_cam_params.c_str());
			return -1;
		}
	}

	if ( mode == "camera" ) 
	{
		tool.registerFromCamera();
	}
	if ( mode == "files" )
	{
		tool.registerFromFiles(clouds);
	}
	if ( mode == "camera_recovery" )
	{
		tool.registerFromFilesAndThenFromCamera(clouds);
	}	

	return 0;
}