// #include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

using namespace cv;
using namespace std;

void display(Mat &im, Mat &bbox)
{
  //circle(im, Point2i(bbox.at<float>(0), bbox.at<float>(1)), 5, Scalar(0, 0, 255), -1);
  //circle(im, Point2i(bbox.at<float>(2), bbox.at<float>(3)), 5, Scalar(0, 255, 0), -1);
  //circle(im, Point2i(bbox.at<float>(4), bbox.at<float>(5)), 5, Scalar(255, 0, 0), -1);
  //circle(im, Point2i(bbox.at<float>(6), bbox.at<float>(7)), 5, Scalar(255,255,255), -1);
  int n = bbox.cols;
  for(int i = 0 ; i < n; i++) {
    line(im,
      Point2i(bbox.at<float>(2*i), bbox.at<float>(2*i + 1)),
      Point2i(bbox.at<float>((2*i + 2) % (2*n)), bbox.at<float>((2*i + 3) % (2*n))),
      Scalar(255,0, 0),
      2);
  }
  imshow("Result", im);
}

int main(int argc, char* argv[])
{
  // Read image
  
  Mat inputImage;
  /*
  if(argc>1)
    inputImage = imread(argv[1]);
  else
    inputImage = imread("qrcode-learnopencv.jpg");
  */

  VideoCapture camera = cv::VideoCapture(2);
  if (!camera.isOpened()) {
      cout << "error opening the camera" << endl;
  }

  QRCodeDetector qrDecoder;

  while (true) {
    Mat bbox, rectifiedImage;

    camera >> inputImage;

    std::string data = qrDecoder.detectAndDecode(inputImage, bbox, rectifiedImage);
    if(data.length()>0)
    {
      cout << "Decoded Data : " << data << endl;

      display(inputImage, bbox);
      rectifiedImage.convertTo(rectifiedImage, CV_8UC3);
      imshow("Rectified QRCode", rectifiedImage);

      waitKey(10);
    }
    else
      cout << "QR Code not detected" << endl;
  }
}