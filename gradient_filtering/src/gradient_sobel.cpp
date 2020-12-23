#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;

void gradientSobel() {
  // TODO: Based on the image gradients in both x and y, compute an image
  // which contains the gradient magnitude according to the equation at the
  // beginning of this section for every pixel position. Also, apply different
  // levels of Gaussian blurring before applying the Sobel operator and compare
  // the results. load image from file

  // load image from file
  cv::Mat img = cv::imread("../images/img1.png");

  // convert image to grayscale
  cv::Mat img_gray;
  cv::cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);

  cv::Mat img_blurred;
  cv::GaussianBlur(img_gray, img_blurred, cv::Size(3, 3), 0, 0,
                   cv::BORDER_DEFAULT);

  // create filter kernel
  float sobel_x[9] = {-1, 0, 1, -2, 0, 2, -1, 0, 1};
  float sobel_y[9] = {-1, -2, -1, 0, 0, 0, 1, 2, 1};

  cv::Mat kernel_x = cv::Mat(3, 3, CV_32F, sobel_x);
  cv::Mat kernel_y = cv::Mat(3, 3, CV_32F, sobel_y);

  // apply filter
  cv::Mat result_x, result_y;
  cv::filter2D(img_blurred, result_x, -1, kernel_x, cv::Point(-1, -1), 0,
               cv::BORDER_DEFAULT);
  cv::filter2D(img_blurred, result_y, -1, kernel_y, cv::Point(-1, -1), 0,
               cv::BORDER_DEFAULT);

  // show results
  string windowName = "Original image";
  cv::namedWindow(windowName, 1); // create window
  cv::imshow(windowName, img);

  windowName = "Grayscale image";
  cv::namedWindow(windowName, 2); // create window
  cv::imshow(windowName, img_gray);

  windowName = "Sobel operator (x-direction)";
  cv::namedWindow(windowName, 3); // create window
  cv::imshow(windowName, result_x);

  windowName = "Sobel operator (y-direction)";
  cv::namedWindow(windowName, 3); // create window
  cv::imshow(windowName, result_y);
  cv::waitKey(0); // wait for keyboard input before continuing
}

int main() { gradientSobel(); }