#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

using namespace std;
using namespace cv;


class SaveVideo {
  public:
  SaveVideo(){}

  void push(Mat& frame) {
    buffer.push_back(frame);
  }

  void write(int w, int h, string fname="outcpp.mp4", int fps=20) {
    VideoWriter video(
    fname,
    // VideoWriter::fourcc('M', 'P', '4', 'V'),
    VideoWriter::fourcc('a', 'v', 'c', '1'),
    fps,
    Size(w,h));

    for(Mat frame: buffer) video.write(frame);

    video.release();
  }

  // VideoWriter* video;
  vector<Mat> buffer;
};

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

  SaveVideo sv;

  VideoCapture cap("../hench.mp4");
  // VideoCapture cap(0);

  if(!cap.isOpened()){
    cout << "Error opening video stream or file" << endl;
    return -1;
  }

  int frame_width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
  int frame_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
  bool record = false;

  while(1){
    Mat frame;
    // Capture frame-by-frame
    cap >> frame;
    frame = rgb2gray(frame);
    frame = gray2rgb(frame);

    // If the frame is empty, break immediately
    if (frame.empty())
      break;

    // if (record) video.write(frame);
    if (record) sv.push(frame);

    // Display the resulting frame
    imshow( "Frame", frame );

    // Press  ESC on keyboard to exit
    char c=(char)waitKey(25);
    if(c==27)
      break;
    else if (c=='s')
      record = !record;
  }

  // When everything done, release the video capture object
  cap.release();
  sv.write(frame_width,frame_height);

  // Closes all the frames
  destroyAllWindows();

  return 0;
}