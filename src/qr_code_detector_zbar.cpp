

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include <math.h>

#include <opencv2/opencv.hpp>

//#include <opencv2/wechat_qrcode.hpp>
//for the cv::Mat
#include <opencv2/core/mat.hpp>
// for Size
#include <opencv2/core/types.hpp>
// for CV_8UC3
#include <opencv2/core/hal/interface.h>
// for compressing the image
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/objdetect.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>

#include <zbar.h>


using std::placeholders::_1;


typedef struct
{
  std::string type;
  std::string data;
  std::vector<cv::Point> location;
} decodedObject;


class QRDetectorNode : public rclcpp::Node
{
public:
  QRDetectorNode()
  : Node("qr_code_detector_node")
  {
    camera_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      "/camera/image_compressed", 10,
      std::bind(&QRDetectorNode::topic_callback, this, _1));
    centroid_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      "/centroid/rel_pos", 10);

    this->declare_parameter("side_threshold", 5.0);
    this->declare_parameter("display_gui", true);
    
    rclcpp::Parameter side_threshold_param_ = this->get_parameter("side_threshold");
    rclcpp::Parameter display_gui_param_ = this->get_parameter("display_gui");
    
    side_threshold_ = side_threshold_param_.as_double();
    display_gui_ = display_gui_param_.as_bool();

    qrDecoder_ = cv::QRCodeDetector();
  }

private:

  void draw_bbox(cv::Mat &im, cv::Mat &bbox)
  {
    int n = bbox.cols;
    for(int i = 0 ; i < n; i++) {
      cv::line(im,
        cv::Point2i(bbox.at<float>(2*i), bbox.at<float>(2*i + 1)),
        cv::Point2i(bbox.at<float>((2*i + 2) % (2*n)), bbox.at<float>((2*i + 3) % (2*n))),
        cv::Scalar(255, 0, 0),
        2);
    }
    //two opposite corners to see the rotation.
    cv::circle(im,
      cv::Point2i(bbox.at<float>(0), bbox.at<float>(1)),
      5,
      cv::Scalar(0, 0, 255),
      -1);
    cv::circle(im,
      cv::Point2i(bbox.at<float>(4), bbox.at<float>(5)),
      5,
      cv::Scalar(0, 255, 0),
      -1);
  }

  float get_centroid_and_size(cv::Mat &bbox, std::vector<int> &centroid) {
    int n = bbox.cols;
    int sum_x = 0;
    int sum_y = 0;
    for(int i = 0 ; i < n; i++) {
      //one point and the next one:
      int x = bbox.at<float>(2*i), y = bbox.at<float>(2*i + 1);
      sum_x += x;
      sum_y += y;
    }

    centroid[0] = sum_x / n;
    centroid[1] = sum_y / n;
    

    //test if the bbox is valid (comparing the sides length ~= squared or romboid):
    int x0 = bbox.at<float>(0), y0 = bbox.at<float>(1); // i = 0
    int x2 = bbox.at<float>(4), y2 = bbox.at<float>(5); // i = 2
    int x1 = bbox.at<float>(2), y1 = bbox.at<float>(3); // i = 1
    int x3 = bbox.at<float>(6), y3 = bbox.at<float>(7); // i = 3
    float dist01 = sqrt((abs(x1-x0)^2) + (abs(y1-y0)^2)); //distance from 0 to 1
    float dist12 = sqrt((abs(x2-x1)^2) + (abs(y2-y1)^2)); //distance from 1 to 2
    float dist23 = sqrt((abs(x3-x2)^2) + (abs(y3-y2)^2)); //distance from 2 to 3
    float dist30 = sqrt((abs(x0-x3)^2) + (abs(y0-y3)^2)); //distance from 3 to 0
    
    bool is_valid = (
      fabs(dist01 - dist12) < side_threshold_ &&
      fabs(dist12 - dist23) < side_threshold_ &&
      fabs(dist23 - dist30) < side_threshold_ &&
      fabs(dist30 - dist01) < side_threshold_);

    float size = -1.0;
    if (is_valid) {
      float diag02 = sqrt((abs(x2-x0)^2) + (abs(y2-y0)^2)); //diagonal from 0 to 2
      float diag13 = sqrt((abs(x3-x1)^2) + (abs(y3-y1)^2)); //diagonal from 1 to 3
      size = (diag02 + diag13) / 2.0; //average of the diagonals
    }
    return size;
  }

// Find and decode barcodes and QR codes
  void decode(cv::Mat &im, std::vector<decodedObject> &decodedObjects)
  {
    // Create zbar scanner and configure it to only detect QR codes:
    zbar::ImageScanner scanner;
    scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 0); //first we deactivate all.
    scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
  
    // Convert image to grayscale and to zbar image:
    cv::Mat imGray;
    cv::cvtColor(im, imGray, CV_BGR2GRAY);
    zbar::Image image(im.cols, im.rows, "Y800", (uchar *) imGray.data, im.cols * im.rows);
  
    int n = scanner.scan(image); // Scan the image for QR codes.
  
    // Print results
    int counter = 0;
    for(zbar::Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol)
    {
      decodedObject obj;
  
      obj.type = symbol->get_type_name();
      obj.data = symbol->get_data();
  
      // Print type and data
      std::cout << "Code num: " << ++counter << std::endl;
      //std::cout << "Type    : " << obj.type  << std::endl; //always QR.
      std::cout << "Data    : " << obj.data  << std::endl << std::endl;
  
      // Obtain location
      for(int i = 0; i< symbol->get_location_size(); i++)
      {
        obj.location.push_back(cv::Point(symbol->get_location_x(i), symbol->get_location_y(i)));
      }
  
      decodedObjects.push_back(obj);
    }
  }

  //TODO:
  void get_bbox(cv::Mat &im, std::vector<decodedObject> &decodedObjects, std::vector<cv::Point> &bbox) {
    ;
  }

  //TODO:
  cv::Point get_centroid(std::vector<cv::Point> &bbox) {
    cv::Point centroid(-1, -1);
    return centroid;
  }

  // Display barcode and QR code location
  void display(cv::Mat &im, std::vector<decodedObject> &decodedObjects/*, std::vector<cv::Point> &centroids*/)
  {
    // Loop over all decoded objects
    for(int i = 0; i < decodedObjects.size(); i++)
    {
      std::vector<cv::Point> points = decodedObjects[i].location;
      std::vector<cv::Point> hull;
  
      // If the points do not form a quad, find convex hull
      if(points.size() > 4)
        convexHull(points, hull);
      else
        hull = points;
  
      // Number of points in the convex hull
      int n = hull.size();
      int centroid_x = 0;
      int centroid_y = 0;
      for(int j = 0; j < n; j++)
      {
        cv::line(im, hull[j], hull[(j+1) % n], cv::Scalar(255, 0, 0), 3);
        centroid_x += hull[j].x;
        centroid_y += hull[(j+1) % n].y;
      }
      /* //with the new functions we won't need them.
      cv::Point centroid = cv::Point(centroid_x /= n, centroid_y /= n);
      centroids.push_back(centroid);
      cv::circle(im, centroid, 5, cv::Scalar(0, 0, 255), -1);
      */
    }
  
    // Display results
    cv::imshow("Results", im);
    cv::waitKey(10);
  
  }

  void topic_callback(sensor_msgs::msg::CompressedImage::SharedPtr msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    //The empty string means that the returned image has the same encoding as the
    //source (we have to use this because in a sensor_msgs::msg::CompressedImage
    //we don have the msg->encoding attribute):
    cv_ptr = cv_bridge::toCvCopy(msg, ""); //this actually works as a decoding function because it decompress the image msg.
    
    //cv::Mat im = imread("zbar-test.jpg");
 
    // Variable for decoded objects
    std::vector<decodedObject> decodedObjects;
  
    // Find and decode barcodes and QR codes
    decode(cv_ptr->image, decodedObjects);
  
    // Display location
    //std::vector<cv::Point> centroids;
    display(cv_ptr->image, decodedObjects/*, centroids*/);
  }


  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr camera_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr centroid_pub_;
  cv::QRCodeDetector qrDecoder_;
  cv::Mat bbox_, rectifiedImage_;
  
  float side_threshold_;
  bool display_gui_;
  cv::Mat input_image_, qr_code_straight_;

};





int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QRDetectorNode>());
  rclcpp::shutdown();
  return 0;
}