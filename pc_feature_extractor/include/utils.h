#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>

#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <iterator>
#include <string>
#include <limits>
#include <array>

struct PointXYZR
{
  PCL_ADD_POINT4D
  uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

typedef pcl::PointXYZ Point;

float pointDistance(Point p)
{
    return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}

struct CloudInfo{
  std::vector<int> startRingIndex;
  std::vector<int> endRingIndex;

  std::vector<int>  pointColInd; // point column index in range image
  std::vector<float> pointRange; // point range 
};

struct smoothness_t {
  float value;
  size_t ind;
};

struct by_value {
  bool operator()(smoothness_t const &left, smoothness_t const &right) {
    return left.value < right.value;
  }
};


void splitString(std::string str, std::vector<std::string>& vec, std::string delimiter = " ")
{
    int start = 0;
    int end = str.find(delimiter);
    while (end != -1) {
        vec.push_back(str.substr(start, end - start));
        start = end + delimiter.size();
        end = str.find(delimiter, start);
    }
    vec.push_back(str.substr(start, end - start));
}

void readPCXYZR(std::string filename, pcl::PointCloud<PointXYZR>::Ptr &pc){
  PointXYZR pt;
  std::string line;
  std::ifstream infile(filename);
  while (std::getline(infile, line)){
    std::vector<std::string> vec;
    splitString(line, vec);
    pt.x = std::stof(vec[0]);
    pt.y = std::stof(vec[1]);
    pt.z = std::stof(vec[2]);
    pt.ring = std::stoi(vec[3]);
    pc->push_back(pt);
  }
}

void readPCXYZ(std::string filename, pcl::PointCloud<Point>::Ptr &pc){
  Point pt;
  std::string line;
  std::ifstream infile(filename);
  while (std::getline(infile, line)){
    std::vector<std::string> vec;
    splitString(line, vec);
    pt.x = std::stof(vec[0]);
    pt.y = std::stof(vec[1]);
    pt.z = std::stof(vec[2]);
    pc->push_back(pt);
  }
}

// template<class PointT>
void pclViewer(pcl::PointCloud<Point>::ConstPtr cloud){
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<Point> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  while (!viewer->wasStopped()) {
    viewer->spinOnce(100);
  }
}

void removeGround(std::string filename, pcl::PointCloud<PointXYZR>::Ptr filtered){
  pcl::PointCloud<PointXYZR>::Ptr cloudR (new pcl::PointCloud<PointXYZR>);
  pcl::PointCloud<Point>::Ptr cloud (new pcl::PointCloud<Point>);
  // pcl::PointCloud<Point>::Ptr cloud_filtered (new pcl::PointCloud<Point>);
  pcl::PointIndices::Ptr ground (new pcl::PointIndices);

  readPCXYZ(filename, cloud);
  readPCXYZR(filename, cloudR);

  // Create the filtering object
  pcl::ProgressiveMorphologicalFilter<Point> pmf;
  pmf.setInputCloud (cloud);
  pmf.setMaxWindowSize (20);
  pmf.setSlope (1.0f);
  pmf.setInitialDistance (0.5f);
  pmf.setMaxDistance (3.0f);
  pmf.extract (ground->indices);

  int cloud_index = 0;
  int indice_index = 0;
  while(cloud_index < cloud->size()){
    if (cloud_index!=ground->indices[indice_index]){
      // cloud_filtered->push_back(cloud->points[cloud_index]);
      filtered->push_back(cloudR->points[cloud_index]);
      cloud_index++;
    }
    else{
      cloud_index++;
      indice_index++;
    }
  }

  // pclViewer(cloud_filtered);
}

#endif