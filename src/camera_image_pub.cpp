
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>



class CameraImagePubNode : public rclcpp::Node
{
public:
  CameraImagePubNode()
  : Node("camera_image_pub_node")
  //This parameter is used to enable/disable internal communications using shared
  //memory when the nodes are running in the same process:
  //: Node("camera_image_pub_node", rclcpp::NodeOptions().use_intra_process_comms(false))
  {
    cam_img_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
      "/camera/image_compressed", 10);

    this->declare_parameter("cam_num", 0);
    this->declare_parameter("fps_pub", 15);
    this->declare_parameter("img_resize_width");
    this->declare_parameter("img_resize_height");
    this->declare_parameter("compression_format");

    rclcpp::Parameter cam_num_param = this->get_parameter("cam_num");
    rclcpp::Parameter fps_pub_param = this->get_parameter("fps_pub");
    rclcpp::Parameter img_resize_width_param = this->get_parameter("img_resize_width");
    rclcpp::Parameter img_resize_height_param = this->get_parameter("img_resize_height");
    rclcpp::Parameter compression_format_param = this->get_parameter("compression_format");

    img_resize_width_ = img_resize_width_param.as_int();
    img_resize_height_ = img_resize_height_param.as_int();

    fps_pub_ = fps_pub_param.as_int();
    int period = 1000 / fps_pub_;
    RCLCPP_INFO(this->get_logger(), "publish period: %d, fps: %d", period, fps_pub_);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period), std::bind(&CameraImagePubNode::timer_callback, this));

    // open the first webcam plugged in the computer:
    int cam_num = cam_num_param.as_int();
    camera_ = cv::VideoCapture(cam_num);
    if (!camera_.set(cv::CAP_PROP_FPS, fps_pub_)) {
      RCLCPP_ERROR(this->get_logger(), "error setting the camera configuration");
    }
    if (!camera_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "error opening the camera");
    }

    std::string format_key = compression_format_param.as_string();
    std::map<std::string, cv_bridge::Format> compress_format_map{
      {"BMP",  cv_bridge::Format::BMP},
      {"DIB",  cv_bridge::Format::DIB},
      {"JPG",  cv_bridge::Format::JPG},
      {"JPEG", cv_bridge::Format::JPEG},
      {"JPE",  cv_bridge::Format::JPE},
      {"JP2",  cv_bridge::Format::JP2},
      {"PNG",  cv_bridge::Format::PNG},
      {"PBM",  cv_bridge::Format::PBM},
      {"PGM",  cv_bridge::Format::PGM},
      {"PPM",  cv_bridge::Format::PPM},
      {"SR",   cv_bridge::Format::SR},
      {"RAS",  cv_bridge::Format::RAS},
      {"TIFF", cv_bridge::Format::TIFF},
      {"TIF",  cv_bridge::Format::TIF}
    };

    auto search_iter = compress_format_map.find(format_key);
    if (search_iter != compress_format_map.end()) {
      RCLCPP_INFO(this->get_logger(), "Image compression format: %s, enum: %d", search_iter->first.c_str(), search_iter->second);
      compression_format_ = search_iter->second;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Image compression format not found");
        
    }
  }

  void timer_callback()
  {     
    // capture the next frame from the webcam and publish it
    camera_ >> cam_frame_;
    if (cam_frame_.rows > img_resize_height_ || cam_frame_.cols > img_resize_width_) {
      cv::resize(cam_frame_, cam_frame_, cv::Size(img_resize_width_, img_resize_height_), cv::INTER_LINEAR);
    }
    
    sensor_msgs::msg::CompressedImage::SharedPtr cam_img_msg =
      cv_bridge::CvImage(
        std_msgs::msg::Header(),
        "bgr8",
        cam_frame_
      ).toCompressedImageMsg(compression_format_);
      cam_img_pub_->publish(*cam_img_msg.get());
  }

private:

  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr cam_img_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  int fps_pub_;
  int img_resize_width_;
  int img_resize_height_;
  
  cv::Mat cam_frame_;
  cv::VideoCapture camera_;

  cv_bridge::Format compression_format_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraImagePubNode>());
  rclcpp::shutdown();
  return 0;
}
