#pragma once

#include <opencv2/opencv.hpp>
#include <utility>


cv::Mat convert(const cv::Mat& img, int cvt) {
  cv::Mat out;
  cv::cvtColor(img, out, cvt);
  return std::move(out);
}

inline cv::Mat rgb2gray(const cv::Mat& img) {return convert(img, cv::COLOR_BGR2GRAY);}
inline cv::Mat gray2rgb(const cv::Mat& img) {return convert(img, cv::COLOR_GRAY2BGR);}

class Window {
  public:
  ~Window() {
    cv::destroyAllWindows();
  }

  char imshow(const cv::Mat& img, int delay=25) {
    cv::imshow( "Frame", img );

    return (char) cv::waitKey(delay);
  }
};