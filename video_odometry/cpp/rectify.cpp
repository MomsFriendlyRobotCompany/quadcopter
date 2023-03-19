#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

using namespace std;
using namespace cv;

Mat convert(const Mat& img, int cvt) {
  Mat out;
  cvtColor(img, out, cvt);
  return out;
}

inline Mat rgb2gray(const Mat& img) {return convert(img, COLOR_BGR2GRAY);}
inline Mat gray2rgb(const Mat& img) {return convert(img, COLOR_GRAY2BGR);}

class Undistort {
  public:
  /*
  K = camera matrix
  d = distortion matrix
  size = (width, height)
  */
  Undistort() {}
  Undistort(Mat K, Mat d, Size s): K(K), d(d), s(s) {
    calc(K,d,s,0.0);
  }

  void calc(Mat K, Mat d, Size s, float alpha=0) {
    this->K = K;
    this->d = d;
    this->s = s;
    optCamMat = getOptimalNewCameraMatrix(K,d,s,alpha);
    Mat R = Mat::eye(3,3,CV_64F);
    initUndistortRectifyMap(K,d,R,optCamMat,s,CV_32FC1,mapx,mapy);
  }

  void calc(double alpha) {
    optCamMat = getOptimalNewCameraMatrix(K,d,s,alpha);
    Mat R = Mat::eye(3,3,CV_64F);
    initUndistortRectifyMap(K,d,R,optCamMat,s,CV_32FC1,mapx,mapy);
  }

  Mat undistort(Mat& img) {
    Mat out;
    remap(img,out,mapx,mapy,INTER_LINEAR);
    return out;
  }

  protected:
  Mat K; // camera matrix
  Mat d; // distortion coefficients from calibration
  Mat optCamMat; // new Optimal K
  Size s; // (width,height)
  Mat mapx,mapy; // distortion maps
};

void print(Mat& img, string name) {
  int dims = 1 + (img.type() >> CV_CN_SHIFT);
  cout << name << " size[h x w x d]: " << img.rows << "x" << img.cols << "x" << dims << endl;
}


int main() {
  cout << "hello" << endl;

  // these numbers come from calibration
  Mat K = (Mat_<double>(3,3) << 532.8, 0.0, 342.5, 0.0, 532.9, 233.9, 0.0, 0.0, 1.0);
  Mat d = (Mat_<double>(5,1) << -0.281,  0.025,  0.001, 0.0,  0.163);
  Undistort ud;
  ud.calc(K, d, Size(640,480), 1.0);

  Mat frame = imread("../left01.jpg");
  Mat gray = rgb2gray(frame);
  Mat fix = ud.undistort(gray);
  print(gray, "gray");
  print(fix, "fix");

  Mat compare;
  hconcat(gray, fix, compare);

  imshow( "Frame", compare );
  waitKey(0);

  return 0;
}