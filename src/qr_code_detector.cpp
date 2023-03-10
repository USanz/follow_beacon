

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


using std::placeholders::_1;


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
    cv::circle(
      im, cv::Point2i(bbox.at<float>(0), bbox.at<float>(1)),
      5, cv::Scalar(0, 0, 255), -1);
    cv::circle(
      im, cv::Point2i(bbox.at<float>(4), bbox.at<float>(5)),
      5, cv::Scalar(0, 255, 0), -1);
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
    
    int x0 = bbox.at<float>(0), y0 = bbox.at<float>(1); // i = 0
    int x1 = bbox.at<float>(2), y1 = bbox.at<float>(3); // i = 1
    int x2 = bbox.at<float>(4), y2 = bbox.at<float>(5); // i = 2
    int x3 = bbox.at<float>(6), y3 = bbox.at<float>(7); // i = 3
    float dist01 = sqrt(pow(x1-x0, 2) + pow(y1-y0, 2)); //distance from 0 to 1
    float dist12 = sqrt(pow(x2-x1, 2) + pow(y2-y1, 2)); //distance from 1 to 2
    float dist23 = sqrt(pow(x3-x2, 2) + pow(y3-y2, 2)); //distance from 2 to 3
    float dist30 = sqrt(pow(x0-x3, 2) + pow(y0-y3, 2)); //distance from 3 to 0
    float diag02 = sqrt(pow(x2-x0, 2) + pow(y2-y0, 2)); //diagonal from 0 to 2
    float diag13 = sqrt(pow(x3-x1, 2) + pow(y3-y1, 2)); //diagonal from 1 to 3
    
    //test if the bbox is valid (comparing the sides length ~= squared or romboid):
    bool is_valid = (
      fabs(dist01 - dist12) < side_threshold_ &&
      fabs(dist12 - dist23) < side_threshold_ &&
      fabs(dist23 - dist30) < side_threshold_ &&
      fabs(dist30 - dist01) < side_threshold_);

    if (!is_valid) { //not squared.
      return -1;
    }
    return (diag02 + diag13) / 2.0; //diagonal avergae size.
  }

  void topic_callback(sensor_msgs::msg::CompressedImage::SharedPtr msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    //The empty string means that the returned image has the same encoding as the
    //source (we have to use this because in a sensor_msgs::msg::CompressedImage
    //we don have the msg->encoding attribute):
    cv_ptr = cv_bridge::toCvCopy(msg, ""); //this actually works as a decoding function because it decompress the image msg.
    
    //std::cout << "OpenCV version : " << CV_VERSION << std::endl; //version 4.7.0-dev
    //RCLCPP_INFO(this->get_logger(), "img received");
    
    //cv::QRCodeDetector qrDecoder_;
    //cv::Mat bbox_, rectifiedImage_;

    //cv::wechat_qrcode:: wc_qr_decoder_; // module not installed.
    //std::string data = wc_qr_decoder_.detectAndDecode(input_image_, bbox_, rectifiedImage_);
    

    //the decode part of this function is bugged and it does not work.
    //std::string data = qrDecoder_.detectAndDecode(input_image_, bbox_, rectifiedImage_);
    std_msgs::msg::Float32MultiArray centroid_per_msg = std_msgs::msg::Float32MultiArray();
    centroid_per_msg.data = {0.0, 0.0, -1.0};
    bool qr_detected = qrDecoder_.detect(cv_ptr->image, bbox_);
    if (qr_detected) {
      //This doesn't work neither:
      /*
      std::string qr_info = qrDecoder_.decode(cv_ptr->image, bbox_, qr_code_straight_);
      if(qr_info.length() > 0) {
        RCLCPP_INFO(this->get_logger(), "QR code information decoded: %s", qr_info.c_str());
      }
      */

      std::vector<int> centroid{0, 0};
      float size = get_centroid_and_size(bbox_, centroid);

      if (display_gui_) {
        draw_bbox(cv_ptr->image, bbox_);
        //RCLCPP_INFO(this->get_logger(), "centroid calculated: [%f, %f]", centroid_per_msg.data[0], centroid_per_msg.data[1]);
        
        cv::Scalar color = cv::Scalar(0, 0, 255); // red
        if (size > 0) {
          centroid_per_msg.data = {
            (( ( (float) centroid[0]) / cv_ptr->image.cols) - 0.5) * 2.0,
            (( ( (float) centroid[1]) / cv_ptr->image.rows) - 0.5) * 2.0,
            size};
          color = cv::Scalar(0, 255, 0); // green
        }
        cv::circle(cv_ptr->image,
          cv::Point2i(centroid[0], centroid[1]),
          15, color, -1);
      }
    }
    RCLCPP_INFO(this->get_logger(), "centroid vals: [%f, %f, %f]",
      centroid_per_msg.data[0],
      centroid_per_msg.data[1],
      centroid_per_msg.data[2]);

    centroid_pub_->publish(centroid_per_msg);

    if (display_gui_) {
      cv::imshow("QR code detector (OpenCV)", cv_ptr->image);
      cv::waitKey(10);
    }
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