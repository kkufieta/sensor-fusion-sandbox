#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;

void cornernessHarris() {
  // load image from file
  cv::Mat img;
  img = cv::imread("../images/img1.png");
  cv::cvtColor(img, img, cv::COLOR_BGR2GRAY); // convert to grayscale

  // Detector parameters
  int blockSize =
      2; // for every pixel, a blockSize × blockSize neighborhood is considered
  int apertureSize = 3; // aperture parameter for Sobel operator (must be odd)
  int minResponse =
      100; // minimum value for a corner in the 8bit scaled response matrix
  double k = 0.04; // Harris parameter (see equation for details)

  // Detect Harris corners and normalize output
  cv::Mat dst, dst_norm, dst_norm_scaled;
  dst = cv::Mat::zeros(img.size(), CV_32FC1);
  cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
  cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
  cv::convertScaleAbs(dst_norm, dst_norm_scaled);

  // visualize results
  string windowName = "Harris Corner Detector Response Matrix";
  cv::namedWindow(windowName, 4);
  cv::imshow(windowName, dst_norm_scaled);

  // Locate local maxima in the Harris response matrix
  // and perform a non-maximum suppression (NMS) in a local neighborhood around
  // each maximum. The resulting coordinates are stored in a list of
  // keypoints of the type `vector<cv::KeyPoint>`.
  std::vector<cv::KeyPoint> keypoints;
  double maxOverlap = 0.0; // max. permissible overlap
  for (size_t r = 0; r < dst_norm_scaled.rows; r++) {
    for (size_t c = 0; c < dst_norm_scaled.cols; c++) {
      int response = (int)dst_norm.at<float>(r, c);
      if (response > minResponse) {
        cv::KeyPoint keypoint;
        keypoint.pt = cv::Point2f(c, r);
        keypoint.size = 2 * apertureSize;
        keypoint.response = response;

        // Perform non-maximum suppression (NMS) in local neighborhood around
        // new keypoint
        bool bOverlap = false;
        for (auto it = keypoints.begin(); it != keypoints.end(); it++) {
          float kptOverlap = cv::KeyPoint::overlap(keypoint, *it);
          if (kptOverlap > maxOverlap) {
            bOverlap = true;
            if (keypoint.response > (*it).response) {
              *it = keypoint; // replace old keypoint with new one
              // break;
            }
          }
        }
        // Only add new keypoint if overlap has not been found in NMS
        if (!bOverlap) {
          keypoints.push_back(keypoint);
        }
      }
    }
  }
  // visualize results
  windowName = "Harris Corner Detector with NMS";
  cv::namedWindow(windowName, 5);
  cv::Mat vis_image = dst_norm_scaled.clone();
  cv::drawKeypoints(dst_norm_scaled, keypoints, vis_image);
  cv::imshow(windowName, vis_image);
  cv::waitKey(0);
}

int main() { cornernessHarris(); }