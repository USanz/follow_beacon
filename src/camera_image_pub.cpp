
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>


using namespace std::chrono_literals;
using std::placeholders::_1;


class CameraImagePubNode : public rclcpp::Node
{
public:
  CameraImagePubNode()
  : Node("camera_image_pub_node")
  {
    cam_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10);
    timer_ = this->create_wall_timer(
      30ms, std::bind(&CameraImagePubNode::timer_callback, this));

    this->declare_parameter("cam_num", 0);
    this->declare_parameter("img_resize_width", 640);
    this->declare_parameter("img_resize_height", 320);

    rclcpp::Parameter img_resize_width_param = this->get_parameter("img_resize_width");
    rclcpp::Parameter img_resize_height_param = this->get_parameter("img_resize_height");
    cam_num_param_ = this->get_parameter("cam_num");

    img_resize_width_ = img_resize_width_param.as_int();
    img_resize_height_ = img_resize_height_param.as_int();

    // open the first webcam plugged in the computer:
    int cam_num = cam_num_param_.as_int();
    camera_ = cv::VideoCapture(cam_num);
    if (!camera_.isOpened()) {
      RCLCPP_INFO(this->get_logger(), "error opening the camera");
    }
  }

  void timer_callback()
  {     
    // capture the next frame from the webcam and publish it
    camera_ >> cam_frame_;
    cv::Mat rsz_img;
    cv::resize(cam_frame_, rsz_img, cv::Size(img_resize_width_, img_resize_height_), cv::INTER_LINEAR);
    
    RCLCPP_INFO(this->get_logger(), "img msg sent");

    sensor_msgs::msg::Image::SharedPtr cam_img_msg = cv_bridge::CvImage(
      std_msgs::msg::Header(), "bgr8", rsz_img).toImageMsg();
    cam_img_pub_->publish(*cam_img_msg.get());
  }

private:

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr cam_img_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Parameter cam_num_param_;

  int img_resize_width_;
  int img_resize_height_;
  
  cv::Mat cam_frame_;
  cv::VideoCapture camera_;
};

int main(int argc, char * argv[])
{
  /*
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraImagePubNode>();
  //TODO: change it to be a publisher with a timer to publish at 50fps.
  rclcpp::Rate loop_rate(10);
  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
    node->publish_img();
  }
  rclcpp::shutdown();
  return 0;
  */

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraImagePubNode>());
  rclcpp::shutdown();
  return 0;
}
