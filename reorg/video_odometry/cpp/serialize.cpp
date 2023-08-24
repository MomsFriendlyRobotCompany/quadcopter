#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

using namespace std;
using namespace cv;


inline Mat rgb2gray(const Mat& img) {
  Mat out;
  cvtColor(img, out, COLOR_BGR2GRAY);
  return out;
}

inline Mat gray2rgb(const Mat& img) {
  Mat out;
  cvtColor(img, out, COLOR_GRAY2BGR);
  return out;
}


int main() {
  cout << "hello" << endl;

  VideoCapture cap("../hench.mp4");
  // VideoCapture cap(0);

  if(!cap.isOpened()){
    cout << "Error opening video stream or file" << endl;
    return -1;
  }

  // int frame_width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
  // int frame_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
  // bool record = false;

  Mat frame;

  cap >> frame;
  Mat gray = rgb2gray(frame);

  cap.release();

  vector<uchar> buffer;
  imencode(".png", frame, buffer);
  cout << "frame size: " << frame.rows * frame.cols << " " << frame.dims << endl;
  cout << "color buffer size: " << buffer.size() << endl;
  cout << "channels: " << 1 + (frame.type() >> CV_CN_SHIFT) << endl;

  vector<uchar> buffer2;
  imencode(".png", gray, buffer2);
  cout << "frame size: " << gray.rows * gray.cols << endl;
  cout << "gray buffer size: " << buffer2.size() << endl;
  cout << "channels: " << 1 + (gray.type() >> CV_CN_SHIFT) << endl;


  return 0;
}