// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <opencv2/opencv.hpp>
// for Size
#include <opencv2/core/types.hpp>
// for CV_8UC3
#include <opencv2/core/hal/interface.h>
// for compressing the image
#include <image_transport/image_transport.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std::chrono_literals;
using std::placeholders::_1;



void get_pixel(const sensor_msgs::msg::Image::SharedPtr msg, int i, int j, int *img_pix)
{
  int channel = 0;
  img_pix[0] = msg->data[i*msg->step + (j + channel++)];
  img_pix[1] = msg->data[i*msg->step + (j + channel++)];
  img_pix[2] = msg->data[i*msg->step + (j + channel)];
  return;
}

void get_centroid(const sensor_msgs::msg::Image::SharedPtr msg, int *pixel_pos)
{ // for now it only gives the first green pixel it finds :D
  pixel_pos[0] = -1;
  pixel_pos[1] = -1;
  int pixel_color[3];
  for (int i = 0; i < (int) msg->height; i++) // rows
  {
    for (int j = 0; j < (int) msg->width; j++) // columns
    {
      get_pixel(msg, i, j, pixel_color);
      /*
      if (pixel_color[0] > 0  && pixel_color[0] <= 5   &&
          pixel_color[1] > 95 && pixel_color[1] <= 105 &&
          pixel_color[2] > 0  && pixel_color[2] <= 5) 
        {
        pixel_pos[0] = j;
        pixel_pos[1] = i;
        return;
      }
      //*/
      if (pixel_color[1] - pixel_color[0] >= 80 &&
          pixel_color[1] - pixel_color[2] >= 80) //if the green component is strong than the others
      {
        pixel_pos[0] = j;
        pixel_pos[1] = i;
        return;
      }
    }
    
  }
  return;
  
}



class OperatorImageNode : public rclcpp::Node
{
public:
  OperatorImageNode()
  : Node("image_operator_node")
  {
    raw_image_debug_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10,
      std::bind(&OperatorImageNode::raw_img_callback, this, _1));
    filtered_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/sources/image_filter/image", 10,
      std::bind(&OperatorImageNode::topic_callback, this, _1));
    centroid_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      "/centroid/rel_pos", 10);
    image_debug_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      "/debug/image/pixel_pos", 10);

    centroid_pixel_ = cv::Point2i(-1, -1);
    pix_per_ = 0.0;
  }

private:
  void raw_img_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
  {
    cv_bridge::CvImagePtr cv_ptr;
    sensor_msgs::msg::Image::SharedPtr img_msg;

    cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);

    cv::circle(cv_ptr->image, centroid_pixel_, (int) (70.0 * pix_per_), cv::Scalar(0, 0, 255), -1);
    
    img_msg = cv_bridge::CvImage(
      std_msgs::msg::Header(), "bgr8", cv_ptr->image).toImageMsg();

    image_debug_pub_->publish(*img_msg.get());
  }

  void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv::Moments mnts;
    cv::Point2f centroidf;
    std_msgs::msg::Float32MultiArray centroid_per_msg;
    int pix_count = 0;
    
    cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);

    //get the moments of the filtered image to get the centroid:
    mnts = cv::moments(cv_ptr->image, true);
    centroidf = cv::Point2f(mnts.m10/mnts.m00, mnts.m01/mnts.m00); //floats
    centroid_pixel_ = cv::Point2i((int) centroidf.x, (int) centroidf.y); //ints
    
    //get the number of filtered pixels to make a percentage to have an idea of how far is the object:
    for (int i = 0; i < cv_ptr->image.rows; i++) {
      for (int j = 0; j < cv_ptr->image.cols; j++) {
        if ((int) cv_ptr->image.at<uchar>(i, j) == 255) { // use Vec3b type for 3 channel imgs instead.
          pix_count++;
        }
      }
    }
    pix_per_ = ((float) pix_count) / ((float) cv_ptr->image.total());

    RCLCPP_INFO(this->get_logger(), "centroid pix: [%d, %d], dist: %f", centroid_pixel_.x, centroid_pixel_.y, pix_per_);
    
    // Centroid distance percentage from the center of the image, being right and
    // down directions from the center, the positive values of x and y respectively.
    centroid_per_msg = std_msgs::msg::Float32MultiArray();
    
    centroid_per_msg.data = {0.0, 0.0, 0.0};
    if (centroidf.x >= 0 && centroidf.y >= 0) {
      centroid_per_msg.data = {
        ((centroidf.x / (float) msg->width) - 0.5) * 2.0,
        ((centroidf.y / (float) msg->height) - 0.5) * 2.0,
        pix_per_};
    }
    centroid_pub_->publish(centroid_per_msg);
    
  }

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr centroid_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_debug_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr filtered_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_image_debug_sub_;

  cv::Point2i centroid_pixel_;
  float pix_per_;
};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OperatorImageNode>());
  rclcpp::shutdown();
  return 0;
}