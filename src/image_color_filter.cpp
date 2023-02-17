#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

#include <opencv2/opencv.hpp>
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


#define IMAGE_RSZ_WIDTH  640
#define IMAGE_RSZ_HEIGHT 320


using namespace std::chrono_literals;
using std::placeholders::_1;



class ImageColorFilterNode : public rclcpp::Node
{
public:
  ImageColorFilterNode()
  : Node("image_color_filter_node")
  {
    camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10,
      std::bind(&ImageColorFilterNode::topic_callback, this, _1));
    image_filter_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      "/image_filter/image", 10);

    this->declare_parameter("display_gui", true);

    hsv_img_ = cv::Mat::zeros(IMAGE_RSZ_HEIGHT, IMAGE_RSZ_WIDTH, CV_8UC3);
    filtered_img_ = cv::Mat::zeros(IMAGE_RSZ_HEIGHT, IMAGE_RSZ_WIDTH, CV_8UC1);
    disp_gui_param_ = this->get_parameter("display_gui");
  }

  cv::Mat get_filtered_img() { return filtered_img_; }

  bool use_gui() { return disp_gui_param_.as_bool(); }

  void set_filter(int hh, int hl, int sh, int sl, int vh, int vl) {
    //filter the image:
    cv::inRange(hsv_img_, cv::Scalar(hl, sl, vl), cv::Scalar(hh, sh, vh), filtered_img_);
    
    //this blur works fine with salt-and-peper noise:
    cv::medianBlur(filtered_img_, filtered_img_, 7);
    
    // TODO: test this noise reduction function better:
    // (at first sight it seems that it's not reducing the noise)
    //cv::fastNlMeansDenoising(filtered_img_, filtered_img_, 30, 7, 21);
    //threshold(filtered_img_, filtered_img_, 128, 255, cv::THRESH_BINARY);
    
    //publish the image:
    sensor_msgs::msg::Image::SharedPtr filtered_img_msg =
        cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", filtered_img_)
            .toImageMsg();
    image_filter_pub_->publish(*filtered_img_msg.get());
  }

private:
  void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
  {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    cv::Mat rsz_img;

    //RCLCPP_INFO(this->get_logger(), "msg received, filtering image");
    
    cv::resize(cv_ptr->image, rsz_img, cv::Size(IMAGE_RSZ_WIDTH, IMAGE_RSZ_HEIGHT), cv::INTER_LINEAR);
    cv::cvtColor(rsz_img, hsv_img_, CV_BGR2HSV); //changing color space.
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_filter_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
  rclcpp::Parameter disp_gui_param_;

  cv::Mat hsv_img_;
  cv::Mat filtered_img_;
};



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ImageColorFilterNode>();

  std::string windowName = "hsv image filter";
  //int hh = 70, hl = 30, sh = 255, sl = 220, vh = 205, vl = 75; //for the green on the simulator
  int hh = 124, hl = 94, sh = 191, sl = 118, vh = 92, vl = 59; //for my dark blue umbrela
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