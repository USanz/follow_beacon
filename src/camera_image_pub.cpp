
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#define IMAGE_RSZ_WIDTH  640
#define IMAGE_RSZ_HEIGHT 320


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
    
    this->declare_parameter("cam_num", 0);
    cam_num_param_ = this->get_parameter("cam_num");

    // open the first webcam plugged in the computer:
    int cam_num = cam_num_param_.as_int();
    camera_ = cv::VideoCapture(cam_num);
    if (!camera_.isOpened()) {
      RCLCPP_INFO(this->get_logger(), "error opening the camera");
    }
  }

  void publish_img()
  {     
    // capture the next frame from the webcam and publish it
    camera_ >> cam_frame_;
    cv::Mat rsz_img;
    cv::resize(cam_frame_, rsz_img, cv::Size(IMAGE_RSZ_WIDTH, IMAGE_RSZ_HEIGHT), cv::INTER_LINEAR);
    
    sensor_msgs::msg::Image::SharedPtr cam_img_msg = cv_bridge::CvImage(
      std_msgs::msg::Header(), "bgr8", rsz_img).toImageMsg();
    cam_img_pub_->publish(*cam_img_msg.get());
  }

private:

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr cam_img_pub_;
  rclcpp::Parameter cam_num_param_;

  cv::Mat cam_frame_;
  cv::VideoCapture camera_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraImagePubNode>();
  rclcpp::Rate loop_rate(10);
  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
    node->publish_img();
  }
  rclcpp::shutdown();
  return 0;
}


/*
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ImageColorFilterNode>();

  std::string windowName = "hsv image filter";
  int hh = 70, hl = 30, sh = 255, sl = 220, vh = 205, vl = 75;
  if (node->use_gui()) {
    cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("hh", windowName, &hh, 255);
    cv::createTrackbar("hl", windowName, &hl, 255);
    cv::createTrackbar("sh", windowName, &sh, 255);
    cv::createTrackbar("sl", windowName, &sl, 255);
    cv::createTrackbar("vh", windowName, &vh, 255);
    cv::createTrackbar("vl", windowName, &vl, 255);
  }

  rclcpp::Rate loop_rate(10);
  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);

    //the resize of the window doesnt work at all so i resized the image:
    //cv::resizeWindow(windowName, 500, 300);
    node->set_filter(hh, hl, sh, sl, vh, vl);
    RCLCPP_INFO(node->get_logger(), " applying filter: [%d, %d, %d, %d, %d, %d\n]", hh, hl, sh, sl, vh, vl);

    if (node->use_gui()) {
      cv::imshow(windowName, node->get_filtered_img());
      cv::waitKey(10);
    }

    loop_rate.sleep();
  }
  cv::destroyAllWindows();
  return 0;
}
//*/