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
  : Node("minimal_subscriber")
  {
    centroid_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/centroid/rel_pos", 10, std::bind(&SinkMotorsNode::topic_callback, this, _1));
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10);
  }

private:
  void topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) const
  {
    float current_x_dist = msg->data[0], current_y_dist = msg->data[1], current_z_dist = msg->data[2];
    float goal_x_dist = 0.0, goal_z_dist = 0.3;
    float goal_x_threshold = 0.1, goal_z_threshold = 0.03; //10% to each side and 3% forward or backward.

    float error_x_dist = goal_x_dist - current_x_dist;
    float kp_w = 0.5; //proportional constant of the controller.
    float wanted_w = kp_w * error_x_dist; //angular vel.

    float error_z_dist = goal_z_dist - current_z_dist;
    float kp_v = 1.0;
    float wanted_v = kp_v * error_z_dist;

    geometry_msgs::msg::Twist vel_msg = geometry_msgs::msg::Twist();
    
    vel_msg.linear.x = wanted_v;
    if (fabs(error_z_dist) < goal_z_threshold) {
      vel_msg.linear.x = 0.0;
    }
    vel_msg.linear.x=0.0; ///SECURITY LOCK, COMMENT THIS LINE TO LET THE ROBOT MOVE.
    vel_msg.linear.y = 0.0;
    vel_msg.linear.z = 0.0;

    vel_msg.angular.x = 0.0;
    vel_msg.angular.y = 0.0;
    vel_msg.angular.z = wanted_w;
    if (fabs(error_x_dist) < goal_x_threshold) {
      vel_msg.angular.z = 0.0;
    }
    
    RCLCPP_INFO(this->get_logger(), "sending [v, w]: [%f, %f]", vel_msg.linear.x, vel_msg.angular.z);
    
    vel_pub_->publish(vel_msg);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr centroid_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SinkMotorsNode>());
  rclcpp::shutdown();
  return 0;
}
