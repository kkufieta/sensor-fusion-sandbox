#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;

void detKeypoints1() {
  // load image from file and convert to grayscale
  cv::Mat imgGray;
  cv::Mat img = cv::imread("../images/img1.png");
  cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

  // Shi-Tomasi detector
  int blockSize = 6; //  size of a block for computing a derivative covariation
                     //  matrix over each pixel neighborhood
  double maxOverlap = 0.0; // max. permissible overlap between two features in %
  double minDistance = (1.0 - maxOverlap) * blockSize;
  int maxCorners =
      img.rows * img.cols / max(1.0, minDistance); // max. num. of keypoints
  double qualityLevel = 0.01; // minimal accepted quality of image corners
  double k = 0.04;
  bool useHarris = false;

  vector<cv::KeyPoint> kptsShiTomasi;
  vector<cv::Point2f> corners;
  double t = (double)cv::getTickCount();
  cv::goodFeaturesToTrack(imgGray, corners, maxCorners, qualityLevel,
                          minDistance, cv::Mat(), blockSize, useHarris, k);
  t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
  cout << "Shi-Tomasi with n= " << corners.size() << " keypoints in "
       << 1000 * t / 1.0 << " ms" << endl;

  for (auto it = corners.begin(); it != corners.end();
       ++it) { // add corners to result vector

    cv::KeyPoint newKeyPoint;
    newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
    newKeyPoint.size = blockSize;
    kptsShiTomasi.push_back(newKeyPoint);
  }

  // visualize results
  cv::Mat visImage = img.clone();
  cv::drawKeypoints(img, kptsShiTomasi, visImage, cv::Scalar::all(-1),
                    cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
  string windowName = "Shi-Tomasi Results";
  cv::namedWindow(windowName, 1);
  imshow(windowName, visImage);

  // Use the OpenCV library to add the FAST detector
  // in addition to the already implemented Shi-Tomasi
  // detector and compare both algorithms with regard to
  // (a) number of keypoints, (b) distribution of
  // keypoints over the image and (c) processing speed.
  int threshold = 45; // difference between intensity of central pixel and
                      // pixels of a circle around this pixel

  vector<cv::KeyPoint> kptsFast;
  t = (double)cv::getTickCount();
  cv::FAST(imgGray, kptsFast, threshold);
  t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
  cout << "FAST with n= " << kptsFast.size() << " keypoints in "
       << 1000 * t / 1.0 << " ms" << endl;

  // Visualize results
  cv::Mat visFastImage = img.clone();
  cv::drawKeypoints(img, kptsFast, visFastImage, cv::Scalar::all(-1),
                    cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
  windowName = "FAST Results";
  cv::namedWindow(windowName, 2);
  imshow(windowName, visFastImage);

  // Compare this detector to the one above
  cv::Ptr<cv::FastFeatureDetector> fast_detector =
      cv::FastFeatureDetector::create(threshold);
  vector<cv::KeyPoint> kptsFastDetector;
  t = (double)cv::getTickCount();
  fast_detector->detect(imgGray, kptsFastDetector);
  t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
  cout << "FAST Detector with n= " << kptsFastDetector.size()
       << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

  // Visualize results
  cv::Mat visFastDetectorImage = img.clone();
  cv::drawKeypoints(img, kptsFastDetector, visFastDetectorImage,
                    cv::Scalar::all(-1),
                    cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
  windowName = "FAST Detector Results";
  cv::namedWindow(windowName, 3);
  imshow(windowName, visFastDetectorImage);
  cv::waitKey(0);
}

int main() { detKeypoints1(); }