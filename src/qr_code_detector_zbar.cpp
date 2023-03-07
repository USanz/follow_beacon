

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
  cv::Point centroid;
  float diagonal_avg_size;
} decodedObject;


class QRDetectorNode : public rclcpp::Node
{
public:
  QRDetectorNode()
  : Node("qr_code_detector_node_zbar")
  {
    camera_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      "/camera/image_compressed", 10,
      std::bind(&QRDetectorNode::topic_callback, this, _1));
    centroid_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      "/centroid/rel_pos", 10);

    this->declare_parameter("qr_data_to_track", "");
    
    rclcpp::Parameter qr_data_to_track_param_ = this->get_parameter("qr_data_to_track");
    
    qr_data_to_track_ = qr_data_to_track_param_.as_string();
    if(qr_data_to_track_ == "") {
      RCLCPP_ERROR(this->get_logger(), "qr_data_to_track parameter not specified of void string given");
    }
    qrDecoder_ = cv::QRCodeDetector();
  }

private:

  void find_and_decode(cv::Mat &im, std::vector<decodedObject> &decodedObjects)
  {
    // Create zbar scanner and configure it to only detect QR codes:
    zbar::ImageScanner scanner;
    scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 0); //first we need to deactivate all.
    scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
    //scanner.enable_cache();

    // Convert image to grayscale and to zbar image:
    cv::Mat imGray;
    cv::cvtColor(im, imGray, CV_BGR2GRAY);
    zbar::Image image(im.cols, im.rows, "Y800", (uchar *) imGray.data, im.cols * im.rows);
  
    int n = scanner.scan(image); // Scan the image for QR codes.
  
    // Print results
    int counter = 0;
    for(zbar::Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol) {
      counter++;
      decodedObject obj;
  
      obj.type = symbol->get_type_name();
      obj.data = symbol->get_data();
  
      // Print type and data
      /*
      std::cout << "Code num: " << counter << std::endl;
      //std::cout << "Type    : " << obj.type  << std::endl; //always QR.
      std::cout << "Data    : " << obj.data  << std::endl << std::endl;
      */

      // Obtain location
      std::vector<cv::Point> symbol_points;
      for(int i = 0; i < symbol->get_location_size(); i++) {
        symbol_points.push_back(cv::Point(symbol->get_location_x(i), symbol->get_location_y(i)));
      }

      std::vector<cv::Point> bbox_points = symbol_points;
      std::vector<cv::Point> hull;
      // If the points do not form a quad, find convex hull
      if(bbox_points.size() > 4) {
        convexHull(bbox_points, hull);
      } else {
        hull = bbox_points;
      }
      
      int n = hull.size();
      obj.centroid.x = 0;
      obj.centroid.y = 0;
      for(int i = 0; i < n; i++) {
        int x = hull[i].x;
        int y = hull[i].y;
        obj.location.push_back(cv::Point(x, y));
        obj.centroid.x += x;
        obj.centroid.y += y;
      }
      obj.centroid.x /= n;
      obj.centroid.y /= n;

      obj.diagonal_avg_size = 0.0;
      if (n == 4) { //squared
        int x0 = obj.location[0].x, y0 = obj.location[0].y; // point 0
        int x1 = obj.location[1].x, y1 = obj.location[1].y; // point 1
        int x2 = obj.location[2].x, y2 = obj.location[2].y; // point 2
        int x3 = obj.location[3].x, y3 = obj.location[3].y; // point 3
        obj.diagonal_avg_size += sqrt((abs(x2-x0)^2) + (abs(y2-y0)^2)); //sum diag 0-2
        obj.diagonal_avg_size += sqrt((abs(x3-x1)^2) + (abs(y3-y1)^2)); //sum diag 1-3
        obj.diagonal_avg_size /= 2.0; //average
      }

      decodedObjects.push_back(obj);
    }
  }

  // Display barcode and QR code location
  void display(cv::Mat &im, std::vector<decodedObject> &decodedObjects)
  {
    // Loop over all decoded objects
    for(int i = 0; i < decodedObjects.size(); i++) {
      //Draw bbox:
      std::vector<cv::Point> points = decodedObjects[i].location;
      int n = points.size();
      for(int j = 0; j < n; j++) {
        cv::line(im, points[j], points[(j+1) % n], cv::Scalar(0, 0, 255), 3);
      }

      //Draw two corners:
      cv::circle(im, points[0], 5, cv::Scalar(255, 0, 0), -1);
      cv::circle(im, points[2], 5, cv::Scalar(255, 255, 0), -1);

      //Draw centroid:
      cv::Scalar centroid_color(0, 0, 255); //red
      if(decodedObjects[i].data == qr_data_to_track_) {
        centroid_color = cv::Scalar(0, 255, 0); // green
      }
      cv::circle(im, decodedObjects[i].centroid, 5, centroid_color, -1);
    }
  
    // Display results:
    cv::imshow("QR code detector (Zbar)", im);
    cv::waitKey(10);
  }

  void publish_biggest_code(std::vector<decodedObject> &decodedObjects, cv::Size abs_img_size) {
    float max_diag_size = 0;
    decodedObject selected_obj;
    for (int i = 0; i < decodedObjects.size(); i++) {
      if(decodedObjects[i].diagonal_avg_size > max_diag_size &&
          decodedObjects[i].data == qr_data_to_track_) {
        selected_obj = decodedObjects[i];
      }
    }

    //Create and send the message:
    std_msgs::msg::Float32MultiArray centroid_per_msg = std_msgs::msg::Float32MultiArray();
    centroid_per_msg.data = {0.0, 0.0, -1.0};
    if (decodedObjects.size() > 0 && selected_obj.diagonal_avg_size > 0.0) {
      centroid_per_msg.data = {
        (( ((float) selected_obj.centroid.x) / abs_img_size.width) - 0.5) * 2.0,
        (( ((float) selected_obj.centroid.y) / abs_img_size.height) - 0.5) * 2.0,
        selected_obj.diagonal_avg_size};
    }
    RCLCPP_INFO(this->get_logger(), "centroid vals: [%f, %f, %f]",
      centroid_per_msg.data[0],
      centroid_per_msg.data[1],
      centroid_per_msg.data[2]);

    centroid_pub_->publish(centroid_per_msg);
    

  }

  void topic_callback(sensor_msgs::msg::CompressedImage::SharedPtr msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    //The empty string means that the returned image has the same encoding as the
    //source (we have to use this because in a sensor_msgs::msg::CompressedImage
    //we don have the msg->encoding attribute):
    cv_ptr = cv_bridge::toCvCopy(msg, ""); //this actually works as a decoding function because it decompress the image msg.
 
    // Variable for decoded objects
    std::vector<decodedObject> decodedObjects;
  
    // Find and decode barcodes and QR codes
    find_and_decode(cv_ptr->image, decodedObjects);
    
    // Display location
    display(cv_ptr->image, decodedObjects);

    // Send only the biggest code position:
    publish_biggest_code(decodedObjects, cv_ptr->image.size());
    /*
    if (decodedObjects.size() > 0) {
      publish_biggest_code(decodedObjects, cv_ptr->image.size());
    }
    */
  }


  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr camera_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr centroid_pub_;
  cv::QRCodeDetector qrDecoder_;
  cv::Mat bbox_, rectifiedImage_;
  
  std::string qr_data_to_track_;
};





int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QRDetectorNode>());
  rclcpp::shutdown();
  return 0;
}