#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <string>

#include "structIO.hpp"

using namespace std;

void showLidarTopview() {
  std::vector<LidarPoint> lidarPoints;
  readLidarPts("../dat/C51_LidarPts_0000.dat", lidarPoints);

  cv::Size worldSize(10.0, 20.0); // width and height of sensor field in m
  cv::Size imageSize(1000, 2000); // corresponding top view image in pixel

  // create topview image
  cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(0, 0, 0));

  // plot Lidar points into image
  for (auto it = lidarPoints.begin(); it != lidarPoints.end(); ++it) {
    float xw = (*it).x; // world position in m with x facing forward from sensor
    float yw = (*it).y; // world position in m with y facing left from sensor

    int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
    int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

    // Remove all Lidar points on the road surface while preserving
    // measurements on the obstacles in the scene.
    // Naive approach to remove the lidar points on the road surface. Should be
    // done using RANSAC
    float zmin = -1.4;
    if ((*it).z >= zmin) {
      // Change the color of the Lidar points such that
      // X=0.0m corresponds to red while X=20.0m is shown as green.
      cv::circle(topviewImg, cv::Point(x, y), 5,
                 cv::Scalar(0, (int)255 * (xw / worldSize.height),
                            (int)255 * (1 - xw / worldSize.height)),
                 -1);
      // Alternatively: Color code the intensity value of the lidar point
      // TODO: Take the intensity distribution into account
      //   cv::circle(topviewImg, cv::Point(x, y), 5,
      //              cv::Scalar(0, (int)255 * (*it).r, (int)255 * (1 -
      //              (*it).r)), -1);
    }
  }

  // plot distance markers
  float lineSpacing = 2.0; // gap between distance markers
  int nMarkers = floor(worldSize.height / lineSpacing);
  for (size_t i = 0; i < nMarkers; ++i) {
    int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) +
            imageSize.height;
    cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y),
             cv::Scalar(255, 150, 150));
    cv::putText(topviewImg,                             // target image
                to_string((int)lineSpacing * i) + " m", // text
                cv::Point(4, y - 8),                    // top-left position
                cv::FONT_HERSHEY_DUPLEX, 1.0,
                CV_RGB(150, 150, 255), // font color
                2);
  }

  // display image
  string windowName = "Top-View Perspective of LiDAR data";
  cv::namedWindow(windowName, 2);
  cv::imshow(windowName, topviewImg);
  cv::waitKey(0); // wait for key to be pressed
}

int main() { showLidarTopview(); }