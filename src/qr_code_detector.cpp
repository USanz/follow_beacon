

#include <memory>

#include "rclcpp/rclcpp.hpp"
//#include "sensor_msgs/msg/image.hpp"
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
#include <image_transport/image_transport.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/objdetect.hpp>
#include <opencv2/imgcodecs.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>



using namespace std::chrono_literals;
using std::placeholders::_1;


class QRDetectorNode : public rclcpp::Node
{
public:
  QRDetectorNode()
  : Node("qr_code_detector_node")
  {
    camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10,
      std::bind(&QRDetectorNode::topic_callback, this, _1));
    centroid_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      "/centroid/rel_pos", 10);

    //timer_ = this->create_wall_timer(
    //  200ms, std::bind(&QRDetectorNode::timer_callback, this));

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
        cv::Scalar(255,0, 0),
        2);
    }
    cv::imshow("Result", im);
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
    
    //diagonal:
    int x0 = bbox.at<float>(0), y0 = bbox.at<float>(1); // i = 0
    int x2 = bbox.at<float>(4), y2 = bbox.at<float>(5); // i = 2
    return sqrt(abs(x2-x0)^2 + abs(y2-y0)^2);
  }

  void topic_callback(sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);

    //cv::resize(cv_ptr->image, input_image_, cv::Size(640, 320), cv::INTER_LINEAR);
    
    //std::cout << "OpenCV version : " << CV_VERSION << std::endl; //version 4.7.0-dev
    //RCLCPP_INFO(this->get_logger(), "img received");
    
    //cv::QRCodeDetector qrDecoder_;
    //cv::Mat bbox_, rectifiedImage_;

    //cv::wechat_qrcode:: wc_qr_decoder_; // module not installed.
    //std::string data = wc_qr_decoder_.detectAndDecode(input_image_, bbox_, rectifiedImage_);
    

    //the decode part of this function is bugged and it does not work.
    //std::string data = qrDecoder_.detectAndDecode(input_image_, bbox_, rectifiedImage_);
    bool qr_detected = qrDecoder_.detect(cv_ptr->image, bbox_);
    if (qr_detected) {
      draw_bbox(cv_ptr->image, bbox_);

      cv::imshow("Result", cv_ptr->image);
      cv::waitKey(10);
    
    
      std_msgs::msg::Float32MultiArray centroid_per_msg = std_msgs::msg::Float32MultiArray();
      centroid_per_msg.data = {0.0, 0.0, 0.0};
      std::vector<int> centroid{0, 0};
      float size = get_centroid_and_size(bbox_, centroid);
      //RCLCPP_INFO(this->get_logger(), "centroid calculated: [%f, %f]", centroid_per_msg.data[0], centroid_per_msg.data[1]);
      if (size > 0) {
        centroid_per_msg.data = {
          (( ( (float) centroid[0]) / cv_ptr->image.cols) - 0.5) * 2.0,
          (( ( (float) centroid[1]) / cv_ptr->image.rows) - 0.5) * 2.0,
          size};
      }
      RCLCPP_INFO(this->get_logger(), "centroid vals: [%f, %f, %f]", centroid_per_msg.data[0], centroid_per_msg.data[1], centroid_per_msg.data[2]);
      centroid_pub_->publish(centroid_per_msg);
    }

  }


  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr centroid_pub_;
  //rclcpp::TimerBase::SharedPtr timer_;
  cv::QRCodeDetector qrDecoder_;
  cv::Mat bbox_, rectifiedImage_;
  
  cv::Mat input_image_;

};





int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QRDetectorNode>());
  rclcpp::shutdown();
  return 0;
}