
#include <vector>
#include <string>

#include "Tools/RegistrationTool.h"

using namespace std;
using namespace pcl;

int main(int argc, char* argv[])
{
	if (argc < 3)
	{
		PCL_ERROR("use: program cloud_file0 .. cloud_fileN. Clouds must be organized\n");
		return -1;
	} 

	vector<string> filenames(argv + 1, argv + argc);
	
	RegistrationTool tool;
	tool.initRegistering(filenames);
	tool.run();

	return 0;
}