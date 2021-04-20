// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#include <iostream>

// include ROS 1
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"

// include ROS 2
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>


ros::Publisher pub;

void twistCallback(const geometry_msgs::msg::Twist::SharedPtr ros2_msg)
{
  // Declarations
  geometry_msgs::Twist ros1_msg;
  geometry_msgs::Vector3 ros1_linear;
  geometry_msgs::Vector3 ros1_angular;
  
  // Linear
  ros1_linear.x = ros2_msg->linear.x;
  ros1_linear.y = ros2_msg->linear.y;
  ros1_linear.z = ros2_msg->linear.z;
  
  ros1_msg.linear = ros1_linear;
  
  // Angular
  ros1_angular.x = ros2_msg->angular.x;
  ros1_angular.y = ros2_msg->angular.y;
  ros1_angular.z = ros2_msg->angular.z;

  ros1_msg.angular = ros1_angular;
  
  //Publish ROS 1 Message
  pub.publish(ros1_msg);
}

int main(int argc, char * argv[])
{ 
  
  // ROS 2 node and subscriber
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("Twist_Bridge");
  node->declare_parameter("topic","cmd_vel");
  std::string topic_;
  node->get_parameter("topic",topic_);
  auto ros2_publisher_qos = rclcpp::QoS(rclcpp::KeepLast(10));
  auto sub = node->create_subscription<geometry_msgs::msg::Twist>(
    topic_, ros2_publisher_qos, twistCallback);
  std::cout << "twist_bridge: Initialized ROS 2 node" << std::endl;
  
  // ROS 1 node and publisher
  ros::init(argc, argv, node->get_name());
  ros::NodeHandle n;
  pub = n.advertise<geometry_msgs::Twist>(topic_, 1);
  std::cout << "twist_bridge: Initialized ROS 1 node" << std::endl;

  // ROS 2 spinning loop
  rclcpp::executors::SingleThreadedExecutor executor;
  while (n.ok() && rclcpp::ok()) {
    executor.spin_node_once(node);
  }
  rclcpp::shutdown();
  return 0;
}
