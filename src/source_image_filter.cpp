#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

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


using namespace std::chrono_literals;
using std::placeholders::_1;



class SourceImageFilterNode : public rclcpp::Node
{
public:
  SourceImageFilterNode()
  : Node("image_operator_node")
  {
    camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10, std::bind(&SourceImageFilterNode::topic_callback, this, _1));
    image_filter_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/sources/image_filter/image", 10);
    
    /*
    std::string windowName = "hsv image";
    cv::namedWindow(windowName);
    int rh = 255, rl = 100, gh = 255, gl = 0, bh = 70, bl = 0;
    cv::createTrackbar("rh", windowName, &rh, 255);
    cv::createTrackbar("rl", windowName, &rl, 255);
    cv::createTrackbar("gh", windowName, &gh, 255);
    cv::createTrackbar("gl", windowName, &gl, 255);
    cv::createTrackbar("bh", windowName, &bh, 255);
    cv::createTrackbar("bl", windowName, &bl, 255);

    cv::waitKey(0);
    */
  }

private:
  void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
  {
    //TODO: filter image
    RCLCPP_INFO(this->get_logger(), "Computing (filtering) received image...");
    

    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);


    //computing image filtering:

    //if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60) {
    cv::circle(cv_ptr->image, cv::Point(500, 500), 100, CV_RGB(255,0,0));
    //}
    cv::Mat hsv_image;
    cv::cvtColor(cv_ptr->image, hsv_image, CV_BGR2HSV);




    

    //cv::imshow("hsv image", hsv_image);



    //This also works fine:
    //sensor_msgs::msg::Image::SharedPtr new_msg =
    //    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", cv_ptr->image)
    //        .toImageMsg();

    
    sensor_msgs::msg::Image::SharedPtr new_msg =
        cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", hsv_image)
            .toImageMsg();
    image_filter_pub_->publish(*new_msg.get());


    //this works fine too:
    //image_filter_pub_->publish(*msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_filter_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SourceImageFilterNode>());
  rclcpp::shutdown();
  return 0;
}