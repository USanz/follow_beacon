

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"

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
  }

private:

  void display(cv::Mat &im, cv::Mat &bbox)
  {
    int n = bbox.rows;
    for(int i = 0 ; i < n ; i++)
    {
      cv::line(im,
        cv::Point2i(bbox.at<float>(i,0),bbox.at<float>(i,1)),
        cv::Point2i(bbox.at<float>((i+1) % n,0),
        bbox.at<float>((i+1) % n,1)),
        cv::Scalar(255,0,0),
        3);
    }
    cv::imshow("Result", im);
  }

  void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
  {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    //cv_ptr->image
    
  
    cv::QRCodeDetector qrDecoder = cv::QRCodeDetector();
  
    cv::Mat bbox, rectifiedImage;
  
    std::string data = qrDecoder.detectAndDecode(cv_ptr->image, bbox, rectifiedImage);
    if(data.length() > 0)
    {
      
      //display(cv_ptr->image, bbox);
      int n = bbox.rows;
      for(int i = 0 ; i < n ; i++) {
        cv::line(cv_ptr->image,
          cv::Point2i(bbox.at<float>(i,0),bbox.at<float>(i,1)),
          cv::Point2i(bbox.at<float>((i+1) % n,0),
          bbox.at<float>((i+1) % n,1)),
          cv::Scalar(255,0,0),
          3);
      }
      cv::imshow("Result", cv_ptr->image);
      
      rectifiedImage.convertTo(rectifiedImage, CV_8UC3);
      cv::imshow("Rectified QRCode", rectifiedImage);
  
      cv::waitKey(0);
    }
    else
      std::cout << "QR Code not detected" << std::endl;

  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
  

};





int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QRDetectorNode>());
  rclcpp::shutdown();
  return 0;
}