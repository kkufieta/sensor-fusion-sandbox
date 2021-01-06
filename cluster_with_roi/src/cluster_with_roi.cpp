#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "dataStructures.h"
#include "structIO.hpp"

using namespace std;

void loadCalibrationData(cv::Mat &P_rect_00, cv::Mat &R_rect_00, cv::Mat &RT) {
  RT.at<double>(0, 0) = 7.533745e-03;
  RT.at<double>(0, 1) = -9.999714e-01;
  RT.at<double>(0, 2) = -6.166020e-04;
  RT.at<double>(0, 3) = -4.069766e-03;
  RT.at<double>(1, 0) = 1.480249e-02;
  RT.at<double>(1, 1) = 7.280733e-04;
  RT.at<double>(1, 2) = -9.998902e-01;
  RT.at<double>(1, 3) = -7.631618e-02;
  RT.at<double>(2, 0) = 9.998621e-01;
  RT.at<double>(2, 1) = 7.523790e-03;
  RT.at<double>(2, 2) = 1.480755e-02;
  RT.at<double>(2, 3) = -2.717806e-01;
  RT.at<double>(3, 0) = 0.0;
  RT.at<double>(3, 1) = 0.0;
  RT.at<double>(3, 2) = 0.0;
  RT.at<double>(3, 3) = 1.0;

  R_rect_00.at<double>(0, 0) = 9.999239e-01;
  R_rect_00.at<double>(0, 1) = 9.837760e-03;
  R_rect_00.at<double>(0, 2) = -7.445048e-03;
  R_rect_00.at<double>(0, 3) = 0.0;
  R_rect_00.at<double>(1, 0) = -9.869795e-03;
  R_rect_00.at<double>(1, 1) = 9.999421e-01;
  R_rect_00.at<double>(1, 2) = -4.278459e-03;
  R_rect_00.at<double>(1, 3) = 0.0;
  R_rect_00.at<double>(2, 0) = 7.402527e-03;
  R_rect_00.at<double>(2, 1) = 4.351614e-03;
  R_rect_00.at<double>(2, 2) = 9.999631e-01;
  R_rect_00.at<double>(2, 3) = 0.0;
  R_rect_00.at<double>(3, 0) = 0;
  R_rect_00.at<double>(3, 1) = 0;
  R_rect_00.at<double>(3, 2) = 0;
  R_rect_00.at<double>(3, 3) = 1;

  P_rect_00.at<double>(0, 0) = 7.215377e+02;
  P_rect_00.at<double>(0, 1) = 0.000000e+00;
  P_rect_00.at<double>(0, 2) = 6.095593e+02;
  P_rect_00.at<double>(0, 3) = 0.000000e+00;
  P_rect_00.at<double>(1, 0) = 0.000000e+00;
  P_rect_00.at<double>(1, 1) = 7.215377e+02;
  P_rect_00.at<double>(1, 2) = 1.728540e+02;
  P_rect_00.at<double>(1, 3) = 0.000000e+00;
  P_rect_00.at<double>(2, 0) = 0.000000e+00;
  P_rect_00.at<double>(2, 1) = 0.000000e+00;
  P_rect_00.at<double>(2, 2) = 1.000000e+00;
  P_rect_00.at<double>(2, 3) = 0.000000e+00;
}

void showLidarTopview(std::vector<LidarPoint> &lidarPoints, cv::Size worldSize,
                      cv::Size imageSize) {
  // create topview image
  cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(0, 0, 0));

  // plot Lidar points into image
  for (auto it = lidarPoints.begin(); it != lidarPoints.end(); ++it) {
    float xw = (*it).x; // world position in m with x facing forward from sensor
    float yw = (*it).y; // world position in m with y facing left from sensor

    int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
    int x = (-yw * imageSize.height / worldSize.height) + imageSize.width / 2;

    float zw = (*it).z; // world position in m with y facing left from sensor
    if (zw > -1.40) {

      float val = it->x;
      float maxVal = worldSize.height;
      int red = min(255, (int)(255 * abs((val - maxVal) / maxVal)));
      int green = min(255, (int)(255 * (1 - abs((val - maxVal) / maxVal))));
      cv::circle(topviewImg, cv::Point(x, y), 5, cv::Scalar(0, green, red), -1);
    }
  }

  // plot distance markers
  float lineSpacing = 2.0; // gap between distance markers
  int nMarkers = floor(worldSize.height / lineSpacing);
  for (size_t i = 0; i < nMarkers; ++i) {
    int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) +
            imageSize.height;
    cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y),
             cv::Scalar(255, 0, 0));
  }

  // display image
  string windowName = "Top-View Perspective of LiDAR data";
  cv::namedWindow(windowName, 2);
  cv::imshow(windowName, topviewImg);
  cv::waitKey(0); // wait for key to be pressed
}

void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes,
                         std::vector<LidarPoint> &lidarPoints) {
  // store calibration data in OpenCV matrices
  cv::Mat P_rect_xx(
      3, 4,
      cv::DataType<double>::type); // 3x4 projection matrix after rectification
  cv::Mat R_rect_xx(4, 4,
                    cv::DataType<double>::type); // 3x3 rectifying rotation to
                                                 // make image planes co-planar
  cv::Mat RT(
      4, 4,
      cv::DataType<double>::type); // rotation matrix and translation vector
  loadCalibrationData(P_rect_xx, R_rect_xx, RT);

  // loop over all Lidar points and associate them to a 2D bounding box
  cv::Mat X(4, 1, cv::DataType<double>::type);
  cv::Mat Y(3, 1, cv::DataType<double>::type);

  for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1) {
    // assemble vector for matrix-vector-multiplication
    X.at<double>(0, 0) = it1->x;
    X.at<double>(1, 0) = it1->y;
    X.at<double>(2, 0) = it1->z;
    X.at<double>(3, 0) = 1;

    // project Lidar point into camera
    Y = P_rect_xx * R_rect_xx * RT * X;
    cv::Point pt;
    pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2); // pixel coordinates
    pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

    double shrinkFactor = 0.10;
    vector<vector<BoundingBox>::iterator>
        enclosingBoxes; // pointers to all bounding boxes which enclose the
    // current Lidar point

    auto createBoundingBox = [](vector<BoundingBox>::iterator it,
                                double shrinkFactor) {
      cv::Rect smallerBox;
      smallerBox.x = it->roi.x + shrinkFactor * it->roi.width / 2.0;
      smallerBox.y = it->roi.y + shrinkFactor * it->roi.height / 2.0;
      smallerBox.width = it->roi.width * (1 - shrinkFactor);
      smallerBox.height = it->roi.height * (1 - shrinkFactor);

      return smallerBox;
    };

    for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin();
         it2 != boundingBoxes.end(); ++it2) {
      // shrink current bounding box slightly to avoid having too many outlier
      // points around the edges
      cv::Rect smallerBox = createBoundingBox(it2, shrinkFactor);

      // check wether point is within current bounding box
      if (smallerBox.contains(pt)) {
        // check whether point is in any other box as well
        bool containedInTwoBoxes = false;
        for (vector<BoundingBox>::iterator it3 = it2 + 1;
             it3 != boundingBoxes.end(); ++it3) {
          // shrink current bounding box slightly to avoid having too many
          // outlier points around the edges
          cv::Rect smallerBox = createBoundingBox(it3, shrinkFactor);
          if (smallerBox.contains(pt)) {
            containedInTwoBoxes = true;
            break;
          }
        }
        if (!containedInTwoBoxes) {
          it2->lidarPoints.push_back(*it1);
        }
        lidarPoints.erase(it1);
        it1--;
        break;
      }
    } // eof loop over all bounding boxes

  } // eof loop over all Lidar points
}

void show3DObjects(std::vector<BoundingBox> boundingBoxes, cv::Size worldSize,
                   cv::Size imageSize, bool bWait = true) {
  cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

  // loop over bounding boxes and plot lidar points into topviewImg, enclosed in
  // a bounding box and with stats
  for (auto it1 = boundingBoxes.begin(); it1 != boundingBoxes.end(); ++it1) {
    cv::RNG rng(it1->boxID);
    cv::Scalar color = cv::Scalar(rng.uniform(0, 150), rng.uniform(0, 150),
                                  rng.uniform(0, 150));

    // plot lidar points into image
    int top = 1e8, left = 1e8, bottom = 0, right = 0;
    float xwmin = 1e8, ywmin = 1e8, ywmax = -1e8;
    for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end();
         ++it2) {
      float xw =
          it2->x; // world position in m with x facing forward from sensor
      float yw =
          it2->y; // world position in m with y facing forward from sensor

      // find closest point in x direction and width
      xwmin = xw < xwmin ? xw : xwmin;
      ywmin = yw < ywmin ? yw : ywmin;
      ywmax = yw > ywmax ? yw : ywmax;

      int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
      int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

      // find bounding box dimensions
      top = y < top ? y : top;
      bottom = y > bottom ? y : bottom;
      left = x < left ? x : left;
      right = x > right ? x : right;

      // draw individual point
      cv::circle(topviewImg, cv::Point(x, y), 4, color, -1);
    }
    // draw enclosing rectangle
    cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),
                  color, 2);

    // augment object with stats
    char str1[200], str2[200];
    sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
    cv::putText(topviewImg, str1, cv::Point(left - 100, bottom + 50),
                cv::FONT_ITALIC, 0.75, color);
    sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax - ywmin);
    cv::putText(topviewImg, str2, cv::Point(left - 100, bottom + 100),
                cv::FONT_ITALIC, 0.75, color);

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
  }
  // display image
  string windowName = "3D objects";
  cv::namedWindow(windowName, 2);
  cv::imshow(windowName, topviewImg);
  if (bWait)
    cv::waitKey(0); // wait for key to be pressed
}

int main() {
  std::vector<LidarPoint> lidarPoints;
  readLidarPts("../dat/C53A3_currLidarPts.dat", lidarPoints);

  std::vector<BoundingBox> boundingBoxes;
  readBoundingBoxes("../dat/C53A3_currBoundingBoxes.dat", boundingBoxes);

  clusterLidarWithROI(boundingBoxes, lidarPoints);
  for (auto it = boundingBoxes.begin(); it != boundingBoxes.end(); ++it) {
    if (it->lidarPoints.size() > 0) {
      showLidarTopview(it->lidarPoints, cv::Size(10.0, 25.0),
                       cv::Size(1000, 2000));
    }
  }
  cv::Size worldSize(25.0, 25.0); // width and height of sensor field in m
  cv::Size imageSize(1000, 2000); // corresponding top view image in pixel
  bool bWait = true;

  show3DObjects(boundingBoxes, worldSize, imageSize, bWait);

  return 0;
}