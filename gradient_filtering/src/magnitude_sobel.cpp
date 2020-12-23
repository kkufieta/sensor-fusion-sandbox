#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;

void magnitudeSobel() {
  // load image from file
  cv::Mat img;
  img = cv::imread("../images/img1.png");

  // convert image to grayscale
  cv::Mat img_gray;
  cv::cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);

  // apply smoothing using the GaussianBlur() function from the OpenCV
  cv::Mat img_blurred;
  int kernel_size = 7;
  float std_dev = 3.0;
  cv::GaussianBlur(img_gray, img_blurred, cv::Size(kernel_size, kernel_size),
                   std_dev, std_dev);

  // create filter kernels using the cv::Mat datatype both for x and y
  // create filter kernel
  float sobel_x[9] = {-1, 0, 1, -2, 0, 2, -1, 0, 1};
  float sobel_y[9] = {-1, -2, -1, 0, 0, 0, 1, 2, 1};

  cv::Mat kernel_x = cv::Mat(3, 3, CV_32F, sobel_x);
  cv::Mat kernel_y = cv::Mat(3, 3, CV_32F, sobel_y);

  // apply filter using the OpenCv function filter2D()
  // apply filter
  cv::Mat result_x, result_y;
  cv::filter2D(img_blurred, result_x, -1, kernel_x, cv::Point(-1, -1), 0,
               cv::BORDER_DEFAULT);
  cv::filter2D(img_blurred, result_y, -1, kernel_y, cv::Point(-1, -1), 0,
               cv::BORDER_DEFAULT);

  cv::Mat img_magnitude = result_x.clone();
  // compute magnitude image based on the equation presented in the lesson
  for (int r = 0; r < result_x.rows; r++) {
    for (int c = 0; c < result_x.cols; c++) {
      img_magnitude.at<unsigned char>(r, c) =
          sqrt(pow(result_x.at<unsigned char>(r, c), 2) +
               pow(result_y.at<unsigned char>(r, c), 2));
    }
  }

  // show result
  string windowName = "Gaussian Blurring";
  cv::namedWindow(windowName, 1); // create window
  cv::imshow(windowName, img_magnitude);
  cv::waitKey(0); // wait for keyboard input before continuing
}

int main() { magnitudeSobel(); }