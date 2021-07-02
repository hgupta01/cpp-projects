#include <iostream>
#include <yaml-cpp/yaml.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/normal_refinement.h>
#include <pcl/visualization/pcl_visualizer.h>

std::vector<std::string> stringSplit(const std::string &str,
                                     const std::string &delim) {
  std::vector<std::string> tokens;
  size_t prev = 0, pos = 0;
  do {
    pos = str.find(delim, prev);
    if (pos == std::string::npos)
      pos = str.length();
    std::string token = str.substr(prev, pos - prev);
    if (!token.empty())
      tokens.push_back(token);
    prev = pos + delim.length();
  } while (pos < str.length() && prev < str.length());
  return tokens;
}

int main() {
  YAML::Node config = YAML::LoadFile(
      "/home/himanshu/Downloads/cpp_projects/cpp-projects/pcl_viewer/config.yaml");

  const std::string filename = config["filename"].as<std::string>();
  std::string delim = ".";
  std::vector<std::string> splits = stringSplit(filename, delim);
  std::string fext = splits.back();

  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(
  //     new pcl::PointCloud<pcl::PointXYZ>());

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr(
      new pcl::PointCloud<pcl::PointXYZRGBA>());

  if (fext.compare("pcd") == 0) {
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(filename, *cloud_ptr) ==
        -1) //* load the file
    {
      PCL_ERROR("Couldn't read file.\n");
      return (-1);
    }
  }

  else if (fext.compare("ply") == 0) {
    if (pcl::io::loadPLYFile<pcl::PointXYZRGBA>(filename, *cloud_ptr) ==
        -1) //* load the file
    {
      PCL_ERROR("Couldn't read file.\n");
      return (-1);
    }
  }

  // else if (fext.compare("ply") == 0) {
  //   if (pcl::io::loadPLYFile<pcl::PointXYZ>(filename, *cloud_ptr) ==
  //       -1) //* load the file
  //   {
  //     PCL_ERROR("Couldn't read file.\n");
  //     return (-1);
  //   }
  // }

  std::cout << cloud_ptr->points.size() << "\n";

  for (int i=0; i<5; ++i){
    std::cout << cloud_ptr->points[i].x << " " << cloud_ptr->points[i].y << " "
	<< cloud_ptr->points[i].z <<  cloud_ptr->points[i].rgba << "\n";
  }
  // ----------------------------------------------------------------
  // -----Calculate surface normals with a search radius of 0.05-----
  // ----------------------------------------------------------------
  // pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  // ne.setInputCloud (cloud_ptr);
  // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ()); 
  // ne.setSearchMethod (tree);
  // pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  // ne.setRadiusSearch (0.4); 
  // // ne.setKSearch(5);
  // ne.compute(*cloud_normals);

  // ----------------------------------------------------------------
  // -----Creating a filter to remove nan values from normal-----
  // -----PCL conditional library could also be used-------------
  // ----------------------------------------------------------------
  // for (int i=0; i<(int)cloud_normals->size(); ++i){
  //   float nx = cloud_normals->points[i].normal_x;
  //   float ny = cloud_normals->points[i].normal_y;
  //   float nz = cloud_normals->points[i].normal_z;

  //   if (std::isnan(nz)){
  //     cloud_normals->points[i].normal_x = 0.0;
  //     cloud_normals->points[i].normal_y = 0.0;
  //     cloud_normals->points[i].normal_z = 1.0;
  //   }
  // }

  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
  // pcl::visualization::PCLVisualizer::Ptr viewer(
  //     new pcl::visualization::PCLVisualizer("3D Viewer"));
  // viewer->setBackgroundColor(0, 0, 0);
  // viewer->addPointCloud<pcl::PointXYZ> (cloud_ptr, "sample cloud");
  // viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud_ptr, cloud_normals, 1, 0.05, "normals");
  // viewer->setPointCloudRenderingProperties(
  //     pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  // viewer->addCoordinateSystem(1.0);
  // viewer->initCameraParameters();

  // while (!viewer->wasStopped()) {
  //   viewer->spinOnce(100);
  // }

  //   pcl::visualization::PCLVisualizer::Ptr rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
  // {
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud_ptr);
    viewer->addPointCloud<pcl::PointXYZRGBA> (cloud_ptr, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();

    std::cout << cloud_ptr->points[0] << "\n" ;
    // std::cout << cloud_ptr->points[0].x << " y:"
              // << cloud_ptr->points[0].y << " z:"
              // << cloud_ptr->points[0].z << " r:"
              // << cloud_ptr->points[0].r << " g:"
              // << cloud_ptr->points[0].g << " b:"
              // << cloud_ptr->points[0].b << " a:"
              // << cloud_ptr->points[0].a << " "
              // << cloud_ptr->points[0].rgba
              // << "\n";

    while (!viewer->wasStopped())
      viewer->spinOnce(100);

    return 0;
  // }
}

// pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>rgb(cloud); 
// viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud"); 
// viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud,
// normals, 10, 0.05, "normals");
