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
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"


using std::placeholders::_1;




class SinkMotorsNode : public rclcpp::Node
{
public:
  SinkMotorsNode()
  : Node("sink_motors_node")
  {
    centroid_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/centroid/rel_pos", 10, std::bind(&SinkMotorsNode::topic_callback, this, _1));
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10);

    this->declare_parameter("only_rotation_mode", false);
    this->declare_parameter("only_display_mode", false);

    this->declare_parameter("goal_x_dist");
    this->declare_parameter("goal_pix_per");
    this->declare_parameter("goal_x_threshold");
    this->declare_parameter("goal_pix_per_threshold");
    this->declare_parameter("kp_w");
    this->declare_parameter("kp_v");
    
    rclcpp::Parameter only_rotation_mode_param = this->get_parameter("only_rotation_mode");
    rclcpp::Parameter only_display_mode_param = this->get_parameter("only_display_mode");
    
    goal_x_dist_param_ = this->get_parameter("goal_x_dist");
    goal_pix_per_param_ = this->get_parameter("goal_pix_per");
    goal_x_threshold_param_ = this->get_parameter("goal_x_threshold");
    goal_pix_per_threshold_param_ = this->get_parameter("goal_pix_per_threshold");
    kp_w_param_ = this->get_parameter("kp_w");
    kp_v_param_ = this->get_parameter("kp_v");
    
    only_rotation_mode_ = only_rotation_mode_param.as_bool();
    only_display_mode_ = only_display_mode_param.as_bool();
  }

private:
  void topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) const
  {
    float current_x_dist = msg->data[0];
    //float current_y_dist = msg->data[1]; //unused at the moment.
    float current_pix_per = msg->data[2];
    float goal_x_dist = goal_x_dist_param_.as_double();
    float goal_pix_per = goal_pix_per_param_.as_double();
    float goal_x_threshold = goal_x_threshold_param_.as_double();
    float goal_pix_per_threshold = goal_pix_per_threshold_param_.as_double(); //10% to each side and 3% forward or backward.

    float error_x_dist = goal_x_dist - current_x_dist;
    float kp_w = kp_w_param_.as_double(); //proportional constant of the controller.
    float wanted_w = kp_w * error_x_dist; //angular vel.

    float error_pix_per = goal_pix_per - current_pix_per;
    float kp_v = kp_v_param_.as_double();
    float wanted_v = kp_v * error_pix_per;

    geometry_msgs::msg::Twist vel_msg = geometry_msgs::msg::Twist();
    
    vel_msg.linear.x = wanted_v;
    if ((fabs(error_pix_per) < goal_pix_per_threshold) ||
        only_rotation_mode_) {
      vel_msg.linear.x = 0.0;
    }
    vel_msg.linear.y = 0.0;
    vel_msg.linear.z = 0.0;

    vel_msg.angular.x = 0.0;
    vel_msg.angular.y = 0.0;
    vel_msg.angular.z = wanted_w;
    if (fabs(error_x_dist) < goal_x_threshold) {
      vel_msg.angular.z = 0.0;
    }

    if (only_display_mode_) {
      vel_msg.linear.x = 0.0;
      vel_msg.angular.z = 0.0;
    }
    
    RCLCPP_INFO(this->get_logger(), "calculated vels [v, w]: [%f, %f], sending [%f, %f]]", wanted_v, wanted_w, vel_msg.linear.x, vel_msg.angular.z);
    
    vel_pub_->publish(vel_msg);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr centroid_sub_;
  
  rclcpp::Parameter goal_x_dist_param_;
  rclcpp::Parameter goal_pix_per_param_;
  rclcpp::Parameter goal_x_threshold_param_;
  rclcpp::Parameter goal_pix_per_threshold_param_;
  rclcpp::Parameter kp_w_param_;
  rclcpp::Parameter kp_v_param_;
  
  bool only_rotation_mode_;
  bool only_display_mode_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SinkMotorsNode>());
  rclcpp::shutdown();
  return 0;
}
