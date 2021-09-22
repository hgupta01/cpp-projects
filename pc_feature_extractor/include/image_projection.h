#include "utils.h"

class ImageProjection {
protected:
  int N_SCAN = 16;
  int Horizon_SCAN = 1800;
  float lidarMinRange = 3;
  float lidarMaxRange = 50;

  // LOAM
  float edgeThreshold = 1.0;
  float surfThreshold = 0.1;

  std::vector<smoothness_t> cloudSmoothness;
  float *cloudCurvature;
  int *cloudNeighborPicked;
  int *cloudLabel;

  cv::Mat rangeMat;//, x, y, z;
  CloudInfo cloudInfo;
  pcl::PointCloud<Point>::Ptr fullCloud;
  pcl::PointCloud<PointXYZR>::Ptr laserCloudIn;
  pcl::PointCloud<Point>::Ptr extractedCloud;
  pcl::PointCloud<Point>::Ptr cornerCloud;
  pcl::PointCloud<Point>::Ptr surfaceCloud;

public:
  ImageProjection(std::string filename);
  ~ImageProjection(){};

  void projectPointCloud();
  void cloudExtraction();
  void calculateSmoothness();
  void markOccludedPoints();
  void extractFeatures();
};

ImageProjection::ImageProjection(std::string filename){
  fullCloud.reset(new pcl::PointCloud<Point>());
  laserCloudIn.reset(new pcl::PointCloud<PointXYZR>());
  extractedCloud.reset(new pcl::PointCloud<Point>());
  cornerCloud.reset(new pcl::PointCloud<Point>());
  surfaceCloud.reset(new pcl::PointCloud<Point>());

  fullCloud->points.resize(N_SCAN*Horizon_SCAN);

  cloudInfo.startRingIndex.resize(N_SCAN);
  cloudInfo.endRingIndex.resize(N_SCAN);

  cloudInfo.pointColInd.resize(N_SCAN*Horizon_SCAN);
  cloudInfo.pointRange.resize(N_SCAN*Horizon_SCAN);

  cloudSmoothness.resize(N_SCAN * Horizon_SCAN);
  cloudCurvature = new float[N_SCAN * Horizon_SCAN];
  cloudNeighborPicked = new int[N_SCAN * Horizon_SCAN];
  cloudLabel = new int[N_SCAN * Horizon_SCAN];

  laserCloudIn->clear();
  extractedCloud->clear();
  rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
  // x = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(0.0));
  // y = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(0.0));
  // z = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(0.0));

  // readPCXYZR(filename, laserCloudIn);
  removeGround(filename, laserCloudIn);
  projectPointCloud();
  cloudExtraction();
  calculateSmoothness();
  markOccludedPoints();
  extractFeatures();
}

void ImageProjection::projectPointCloud() {
  int cloudSize = laserCloudIn->points.size();
  // range image projection
  for (int i = 0; i < cloudSize; ++i) {
    Point thisPoint;
    thisPoint.x = laserCloudIn->points[i].x;
    thisPoint.y = laserCloudIn->points[i].y;
    thisPoint.z = laserCloudIn->points[i].z;

    float range = pointDistance(thisPoint);
    if (range < lidarMinRange || range > lidarMaxRange)
      continue;

    int rowIdn = laserCloudIn->points[i].ring;
    if (rowIdn < 0 || rowIdn >= N_SCAN)
      continue;

    float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI; // in degrees atan2(y/x) not x/y

    static float ang_res_x = 360.0 / float(Horizon_SCAN); // in degrees
    int columnIdn =
        -round((horizonAngle - 90.0) / ang_res_x) + Horizon_SCAN / 2;
    if (columnIdn >= Horizon_SCAN)
      columnIdn -= Horizon_SCAN;

    if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
      continue;

    if (rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX)
      continue;

    rangeMat.at<float>(rowIdn, columnIdn) = range;
    x.at<float>(rowIdn, columnIdn) = thisPoint.x;
    y.at<float>(rowIdn, columnIdn) = thisPoint.y;
    z.at<float>(rowIdn, columnIdn) = thisPoint.z;

    int index = columnIdn + rowIdn * Horizon_SCAN;
    fullCloud->points[index] = thisPoint;
  }

  cv::FileStorage file("/home/himanshu/image.txt", cv::FileStorage::WRITE);
  file << "range" <<rangeMat;
  file << "x" << x;
  file << "y" << y;
  file << "z" << z;
}

void ImageProjection::cloudExtraction() {
  int count = 0;
  // extract segmented cloud for lidar odometry
  for (int i = 0; i < N_SCAN; ++i) {
    cloudInfo.startRingIndex[i] = count - 1 + 5; // count + 4 // 4

    for (int j = 0; j < Horizon_SCAN; ++j) {
      if (rangeMat.at<float>(i, j) != FLT_MAX) {
        // mark the points' column index for marking occlusion later
        cloudInfo.pointColInd[count] = j;
        // save range info
        cloudInfo.pointRange[count] = rangeMat.at<float>(i, j);
        // save extracted cloud
        extractedCloud->push_back(fullCloud->points[j + i * Horizon_SCAN]);
        // size of extracted cloud
        ++count;
      }
    }
    cloudInfo.endRingIndex[i] = count - 1 - 5; // count - 6 // 1800-6=1794
  }
  // pclViewer(extractedCloud);
}

void ImageProjection::calculateSmoothness() { 
  int cloudSize = extractedCloud->points.size();
  for (int i = 5; i < cloudSize - 5; i++) {
    float diffRange =
        cloudInfo.pointRange[i - 5] + cloudInfo.pointRange[i - 4] +
        cloudInfo.pointRange[i - 3] + cloudInfo.pointRange[i - 2] +
        cloudInfo.pointRange[i - 1] - cloudInfo.pointRange[i] * 10 +
        cloudInfo.pointRange[i + 1] + cloudInfo.pointRange[i + 2] +
        cloudInfo.pointRange[i + 3] + cloudInfo.pointRange[i + 4] +
        cloudInfo.pointRange[i + 5];

    cloudCurvature[i] =
        diffRange * diffRange; // diffX * diffX + diffY * diffY + diffZ * diffZ;

    cloudNeighborPicked[i] = 0;
    cloudLabel[i] = 0;
    // cloudSmoothness for sorting
    cloudSmoothness[i].value = cloudCurvature[i];
    cloudSmoothness[i].ind = i;
  }
}

void ImageProjection::markOccludedPoints() {
  int cloudSize = extractedCloud->points.size();
  // mark occluded points and parallel beam points
  for (int i = 5; i < cloudSize - 6; ++i) {
    // occluded points
    float depth1 = cloudInfo.pointRange[i];
    float depth2 = cloudInfo.pointRange[i + 1];
    int columnDiff =
        std::abs(int(cloudInfo.pointColInd[i + 1] - cloudInfo.pointColInd[i]));

    if (columnDiff < 10) {
      // 10 pixel diff in range image
      if (depth1 - depth2 > 0.3) {
        cloudNeighborPicked[i - 5] = 1;
        cloudNeighborPicked[i - 4] = 1;
        cloudNeighborPicked[i - 3] = 1;
        cloudNeighborPicked[i - 2] = 1;
        cloudNeighborPicked[i - 1] = 1;
        cloudNeighborPicked[i] = 1;
      } else if (depth2 - depth1 > 0.3) {
        cloudNeighborPicked[i + 1] = 1;
        cloudNeighborPicked[i + 2] = 1;
        cloudNeighborPicked[i + 3] = 1;
        cloudNeighborPicked[i + 4] = 1;
        cloudNeighborPicked[i + 5] = 1;
        cloudNeighborPicked[i + 6] = 1;
      }
    }
    // parallel beam
    float diff1 =
        std::abs(float(cloudInfo.pointRange[i - 1] - cloudInfo.pointRange[i]));
    float diff2 =
        std::abs(float(cloudInfo.pointRange[i + 1] - cloudInfo.pointRange[i]));

    if (diff1 > 0.02 * cloudInfo.pointRange[i] &&
        diff2 > 0.02 * cloudInfo.pointRange[i])
      cloudNeighborPicked[i] = 1;
  }
}

void ImageProjection::extractFeatures() {
  cornerCloud->clear();
  surfaceCloud->clear();

  pcl::PointCloud<Point>::Ptr surfaceCloudScan(
      new pcl::PointCloud<Point>());

  for (int i = 0; i < N_SCAN; i++) {
    surfaceCloudScan->clear();

    for (int j = 0; j < 6; j++) {

      int sp = (cloudInfo.startRingIndex[i] * (6 - j) +
                cloudInfo.endRingIndex[i] * j) /
               6; // (4*6 + 1794*0)/6 = 4// start point
      int ep = (cloudInfo.startRingIndex[i] * (5 - j) +
                cloudInfo.endRingIndex[i] * (j + 1)) /
                   6 -
               1; // (4*5+1794)/5 = 362// end point

      if (sp >= ep)
        continue;

      std::sort(cloudSmoothness.begin() + sp, cloudSmoothness.begin() + ep,
                by_value());

      int largestPickedNum = 0;
      for (int k = ep; k >= sp; k--) {
        int ind = cloudSmoothness[k].ind;
        if (cloudNeighborPicked[ind] == 0 &&
            cloudCurvature[ind] > edgeThreshold) {
          largestPickedNum++;
          if (largestPickedNum <= 20) {
            cloudLabel[ind] = 1;
            cornerCloud->push_back(extractedCloud->points[ind]);
          } else {
            break;
          }

          cloudNeighborPicked[ind] = 1;
          for (int l = 1; l <= 5; l++) {
            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] -
                                          cloudInfo.pointColInd[ind + l - 1]));
            if (columnDiff > 10)
              break;
            cloudNeighborPicked[ind + l] = 1;
          }
          for (int l = -1; l >= -5; l--) {
            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] -
                                          cloudInfo.pointColInd[ind + l + 1]));
            if (columnDiff > 10)
              break;
            cloudNeighborPicked[ind + l] = 1;
          }
        }
      }

      for (int k = sp; k <= ep; k++) {
        int ind = cloudSmoothness[k].ind;
        if (cloudNeighborPicked[ind] == 0 &&
            cloudCurvature[ind] < surfThreshold) {

          cloudLabel[ind] = -1;
          cloudNeighborPicked[ind] = 1;

          for (int l = 1; l <= 5; l++) {

            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] -
                                          cloudInfo.pointColInd[ind + l - 1]));
            if (columnDiff > 10)
              break;

            cloudNeighborPicked[ind + l] = 1;
          }
          for (int l = -1; l >= -5; l--) {

            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] -
                                          cloudInfo.pointColInd[ind + l + 1]));
            if (columnDiff > 10)
              break;

            cloudNeighborPicked[ind + l] = 1;
          }
        }
      }

      for (int k = sp; k <= ep; k++) {
        if (cloudLabel[k] <= 0) {
          surfaceCloudScan->push_back(extractedCloud->points[k]);
        }
      }
    }
    *surfaceCloud += *surfaceCloudScan;
  }
  // pclViewer(cornerCloud);
  // pclViewer(surfaceCloud);
}

