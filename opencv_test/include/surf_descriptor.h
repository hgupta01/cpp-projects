#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"

#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

using namespace boost::python;

typedef std::vector<int> MyList;

class SURFDescriptor{
private:
	cv::Mat src_img, trg_img;
  MyList src_pnts, trg_pnts;

public:
  SURFDescriptor(){};
  SURFDescriptor(std::string src, std::string trg);
  void calculateMatchingPoints();
  // inline void setSrcImg(std::string file){src_img = cv::imread(file, cv::IMREAD_GRAYSCALE);};
  // inline void setTrgImg(std::string file){trg_img = cv::imread(file, cv::IMREAD_GRAYSCALE);};
  inline MyList getSrcPnts(){return src_pnts;};
  inline MyList getTrgPnts(){return trg_pnts;};
};

SURFDescriptor::SURFDescriptor(std::string src, std::string trg){
  src_img = cv::imread(src, cv::IMREAD_GRAYSCALE);
  trg_img = cv::imread(trg, cv::IMREAD_GRAYSCALE);
}

void SURFDescriptor::calculateMatchingPoints(){
  if ( src_img.empty() || trg_img.empty() )
    {
        std::cout << "Could not open or find the image!\n" << std::endl;
    }
    //-- Step 1: Detect the keypoints using SURF Detector, compute the descriptors
    int minHessian = 400;
    cv::Ptr<cv::xfeatures2d::SURF> descriptor = cv::xfeatures2d::SURF::create( minHessian );
    cv::Ptr<cv::AgastFeatureDetector> detector = cv::AgastFeatureDetector::create();

    std::vector<cv::KeyPoint> keypoints1, keypoints2;
    cv::Mat descriptors1, descriptors2;
    detector->detect(src_img, keypoints1);
    detector->detect(trg_img, keypoints2);

    descriptor->compute(src_img, keypoints1, descriptors1);
    descriptor->compute(trg_img, keypoints2, descriptors2);
    // detector->detectAndCompute( img1, noArray(), keypoints1, descriptors1 );
    // detector->detectAndCompute( img2, noArray(), keypoints2, descriptors2 );
    
    // Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE);
    std::vector< std::vector<cv::DMatch> > knn_matches;
    matcher->knnMatch( descriptors1, descriptors2, knn_matches, 2);
    //-- Filter matches using the Lowe's ratio test
    const float ratio_thresh = 0.5f;
    std::vector<cv::DMatch> good_matches;
    for (size_t i = 0; i < knn_matches.size(); i++)
    {
        if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
        {
            good_matches.push_back(knn_matches[i][0]);
        }
    }

		int count = 0;
    for (auto match: good_matches){
      int imgsrc_idx = match.queryIdx;
      int imgtrg_idx = match.trainIdx;

      cv::Point2f ptsrc = keypoints1[imgsrc_idx].pt;
      cv::Point2f pttrg = keypoints2[imgtrg_idx].pt;

			src_pnts.push_back(ptsrc.x);
			src_pnts.push_back(ptsrc.y);
			trg_pnts.push_back(pttrg.x);
			trg_pnts.push_back(pttrg.y);
    }
}