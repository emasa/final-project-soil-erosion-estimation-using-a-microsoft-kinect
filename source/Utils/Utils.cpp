
#include <string>

#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/time.h>
#include <pcl/recognition/color_gradient_dot_modality.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "Utils/Utils.h"

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

template<> void 
displayKeypoints<pcl::PointXYZRGBA, pcl::PointWithScale>
	(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, 
     const pcl::PointCloud<pcl::PointWithScale>::Ptr &keypoints, 
     pcl::visualization::PCLVisualizer &viewer, 
     const pcl::PointRGB &color,
     const unsigned int size, 
     const std::string &cloud_id,
     const std::string &keypoints_id, 
     const int viewport);

bool
loadCloud (const string &filename, pcl::PCLPointCloud2 &cloud)
{
  TicToc tt;
  print_highlight ("Loading "); print_value ("%s ", filename.c_str ());

  tt.tic ();
  if (loadPCDFile (filename, cloud) < 0)
    return (false);
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud.width * cloud.height); print_info (" points]\n");
  print_info ("Available dimensions: "); print_value ("%s\n", getFieldsList (cloud).c_str ());

  return (true);
}

void
saveCloud (const string &filename, const pcl::PCLPointCloud2 &output)
{
  TicToc tt;
  tt.tic ();

  print_highlight ("Saving "); print_value ("%s ", filename.c_str ());

  PCDWriter w;
  w.writeBinaryCompressed (filename, output);
  
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", output.width * output.height); print_info (" points]\n");
}