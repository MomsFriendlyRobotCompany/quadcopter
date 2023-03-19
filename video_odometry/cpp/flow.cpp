#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

#include "common.hpp"
#include <unistd.h>

using namespace std;
using namespace cv;

using cv::Ptr;

// they have an opencv example, but don't use much of opencv
// https://github.com/PX4/PX4-OpticalFlow
// https://github.com/ethz-ait/klt_feature_tracker

// different detectors
// https://github.com/oreillymedia/Learning-OpenCV-3_examples/blob/master/example_16-02.cpp

#include <string>
#include <chrono>

namespace sc = std::chrono;

double time_now(){
    sc::microseconds ms = sc::duration_cast< sc::microseconds >(
    sc::system_clock::now().time_since_epoch());
    return (double)(ms.count())/1000000.0;
}

class OpticalFlow {
  public:
  OpticalFlow(){}

  void init(Mat& K, uint min_num_feat=10) {
    this->Rf = Mat::eye(3,3,CV_64F);
    this->tf = Mat::zeros(3,1,CV_64F);
    this->fc = K.at<double>(0,0);
    this->pc = Point2f(K.at<double>(0,2), K.at<double>(1,2)); // (x,y)
    this->min_num_feat = min_num_feat;

    // int threshold = 25;
    // bool nonmaxSuppression = true;
    // this->detector = FastFeatureDetector::create(threshold, nonmaxSuppression);
  }

  bool featureTracking(const Mat& img_1, const Mat& img_2) {
    // check we have our minimum of features, if not, find new ones
    if (p1.size() < min_num_feat) {
      cout << "p1 has only: " << p1.size() << " ";

      if (0) {
        Size subPixWinSize(10,10);
        int maxCorners = 300;
        double quality = 0.3;
        double minDistance = 20;
        int blockSize = 20;
        TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);
        goodFeaturesToTrack(img_1, p1, maxCorners, quality, minDistance, Mat(), blockSize);
        cornerSubPix(img_1, p1, subPixWinSize, Size(-1,-1), termcrit);
      }
      else if (0) {
        vector<KeyPoint> kpts;
        int threshold = 25;
        bool nonmaxSuppression = true;
        Ptr<FastFeatureDetector> detector = FastFeatureDetector::create(threshold, nonmaxSuppression);
        detector->detect(img_1, kpts);
        KeyPoint::convert(kpts, p1);
      }
      else if (0) {
        vector<KeyPoint> kpts;
        Ptr<FeatureDetector> detector = ORB::create(352);
        detector->detect(img_1, kpts);
        KeyPointsFilter::removeDuplicated(kpts); // value??
        KeyPoint::convert(kpts, p1);
      }
      else if (1) {
        int minHessian = 40;
        Ptr<FeatureDetector> detector = SIFT::create( minHessian );
        std::vector<KeyPoint> kpts;
        detector->detect( img_1, kpts );
        KeyPoint::convert(kpts, p1);
      }

      cout << "re-found features: " << p1.size() << endl;
    }

    if (p1.size() <= 0) return false;

    // do optical flow .. update p2 features from new image
    Size winSize(21,21);
    int maxLevel = 3;
    TermCriteria criteria = TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 30, 0.01);
    vector<uchar> status; // track if feature found in BOTH images
    vector<float> err;
    calcOpticalFlowPyrLK(img_1, img_2, p1, p2, status, err, winSize, maxLevel, criteria); // find p2

    // if (status.size() < 100) {
    //   cout << "calcOpticalFlow failed" << endl;
    //   return false;
    // }

    // remove bad feature points ... if feature not
    // found in BOTH images, it is bad
    vector<Point2f> new_p1, new_p2;
    for(uint i = 0; i < status.size(); ++i){
        // Select good feature points
        if(status[i] == 1) {
          new_p1.push_back(p1[i]);
          new_p2.push_back(p2[i]);
        }
    }
    // std::swap(p1, new_p1);
    // std::swap(p2, new_p2);
    p1 = new_p1;
    p2 = new_p2;

    // if (p1.size() != p2.size()) {
    //   cout << "size error" << endl;
    //   return false;
    // }

    return true;
  }

  bool calcuatePose() {
    try {
      double prob = 0.999;
      double threshold = 1.0;
      Mat E;
      // if unique, E = 3x3, if not unique, E = 3x(3n) where n
      // is the number of possible matricies
      if ((p1.size() < 8) || (p2.size() < 8)) {
        cout << "too few pts for unique solution: " << p1.size() << " and " << p2.size() << endl;
        // throw cv::Exception("no features");
        return false;
      }

      Mat eE = findEssentialMat(p1,p2,fc,pc,RANSAC,prob,threshold);
      if (eE.rows > 3) { // not unique, take first solution
        Rect r(0,0,3,3);
        E = eE(r);
      }
      else { // unique
        E = eE;
      }

      if (E.empty()) return false;

      Mat R(3,3,CV_64F), t(3,1,CV_64F);
      recoverPose(E,p1,p2,R,t,fc,pc);

      double scale = 0.1; // FIXME: include altitude

      // update position and rotation
      tf = tf - scale * R * t;
      Rf = R * Rf;
    }
    catch (cv::Exception e) {
      cout << "Crap ------------------------------------" << endl;
      cout << e.code << ": " << e.err << " "<< e.msg << endl;
      // cout << "E: " << E << endl;
      cout << "size error?: " << p1.size() << " != " << p2.size() << endl;
      throw e;
      // return false;
    }
    catch (...) {
      cout<< "Crap: Oops ..." << endl;
      return false;
    }

    return true;
  }

  // img1: old image, class keeps track of this
  // img2: new image
  void once(Mat& img2) {
    // if img1 is empty (first time through loop)
    if (img1.empty()) img1 = img2.clone();

    if (featureTracking(img1, img2)) {
      calcuatePose();
      // std::swap(p1,p2); // set p1 = p2
      p1 = p2;
    }

    // std::swap(p1,p2); // set p1 = p2
    // cv::swap(img1,img2);
    img1 = img2.clone();
  }

  Mat get_R() const { return Rf; }
  Mat get_t() const { return tf; }

  protected:
  Mat Rf; // final rotation of camera
  Mat tf; // final translation of camera
  Mat img1; // old image
  // Ptr<FastFeatureDetector> detector;
  vector<Point2f> p1,p2; // feature points found in image
  double fc; // focal length from camera matrix
  Point2f pc; // principle point from camera matrix
  uint min_num_feat; // min number of features to maintain
};



int main() {
  // vector<int> vect1{1, 2, 3, 4};

  //   // Declaring new vector
  //   vector<int> vect2{9,8,7};

  //   // Using assignment operator to copy one
  //   // vector to other
  //   vect2 = vect1;
  //   for (int i=0; i < vect2.size(); i++) cout << vect2[i] << " ";
  //   cout << endl;

  //   return 0;

  cout << "hello" << endl;

  // VideoCapture cap("../floor-loop-2.mp4");
  VideoCapture cap(1);
  cap.grab();
  cap.grab();
  cap.set(cv::CAP_PROP_FPS, 120);
  usleep(100000);
  cap.set(CAP_PROP_FRAME_WIDTH, 800);
  usleep(100000);
  cap.set(CAP_PROP_FRAME_HEIGHT, 600);
  usleep(100000);
  cap.grab();
  cap.grab();

  if(!cap.isOpened()){
    cout << "Error opening video stream or file" << endl;
    return -1;
  }

  int w = cap.get(cv::CAP_PROP_FRAME_WIDTH);
  int h = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
  float s = cap.get(cv::CAP_PROP_FPS);
  cout << "video: " << w << "x" << h << "@" << s << endl;
  // bool record = false;

  // Mat K = (Mat_<double>(3,3) << 532.8, 0.0, 320./2.0, 0.0, 532.9, 240./2.0, 0.0, 0.0, 1.0);
  // OpticalFlow of;
  // of.init(K);

  Mat frame;
  Window win;

  double epoch = time_now();
  int count = 0;

  while (1) {

    cap >> frame;
    if (frame.empty()) {
      cout << "crap" << endl;
      continue;
    }

    Mat tmp = rgb2gray(frame);
    Mat gray = tmp;

    // gray = tmp(Rect(0,0,1024,1024));
    // resize(tmp, gray, Size(640, 480), 0, 0, INTER_CUBIC);
    // resize(tmp, gray, Size(320, 240), 0, 0, INTER_CUBIC);

    // of.once(gray);
    // cout << "t: " << of.get_t() << endl;

    char c = win.imshow(gray,1);
    if(c==27)
      break;

    // cout << count++ /(time_now() - epoch) << "\n";
  }

  // cout << "done R: " << of.get_R() << endl;
  // cout << "done t: " << of.get_t() << endl;

  cap.release();

  return 0;
}