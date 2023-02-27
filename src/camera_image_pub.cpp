
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#define IMAGE_RSZ_WIDTH  640
#define IMAGE_RSZ_HEIGHT 320


//using namespace std::chrono_literals;
using std::placeholders::_1;


class CameraImagePubNode : public rclcpp::Node
{
public:
  CameraImagePubNode()
  : Node("camera_image_pub_node")
  {
    cam_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10);

    this->declare_parameter("cam_num", 0);
    this->declare_parameter("fps_pub", 15);
    this->declare_parameter("img_resize_width");
    this->declare_parameter("img_resize_height");

    rclcpp::Parameter cam_num_param = this->get_parameter("cam_num");
    rclcpp::Parameter fps_pub_param = this->get_parameter("fps_pub");
    rclcpp::Parameter img_resize_width_param = this->get_parameter("img_resize_width");
    rclcpp::Parameter img_resize_height_param = this->get_parameter("img_resize_height");

    img_resize_width_ = img_resize_width_param.as_int();
    img_resize_height_ = img_resize_height_param.as_int();


    fps_pub_ = fps_pub_param.as_int();
    int period = 1000 / fps_pub_;
    std::cout << "publish period: " << period << std::endl;
    std::cout << "publish fps: " << fps_pub_ << std::endl;
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period), std::bind(&CameraImagePubNode::timer_callback, this));

    // open the first webcam plugged in the computer:
    int cam_num = cam_num_param.as_int();
    camera_ = cv::VideoCapture(cam_num);
    if (!camera_.set(cv::CAP_PROP_FPS, fps_pub_)) {
      RCLCPP_INFO(this->get_logger(), "error setting the camera configuration");
    }
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
    
    sensor_msgs::msg::Image::SharedPtr cam_img_msg = cv_bridge::CvImage(
      std_msgs::msg::Header(), "bgr8", rsz_img).toImageMsg();
    cam_img_pub_->publish(*cam_img_msg.get());
  }

private:

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr cam_img_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  int fps_pub_;
  int img_resize_width_;
  int img_resize_height_;
  
  cv::Mat cam_frame_;
  cv::VideoCapture camera_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  /*
  auto node = std::make_shared<CameraImagePubNode>();
  rclcpp::Rate loop_rate(10);

  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
    node->publish_img();
  }
  */

  rclcpp::spin(std::make_shared<CameraImagePubNode>());
  rclcpp::shutdown();
  return 0;
}
