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
#include <memory>
#include <utility>

// include ROS 1
#include "ros/ros.h"
#include "sensor_msgs/CompressedImage.h"

// include ROS 2
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/compressed_image.hpp>

std::shared_ptr<rclcpp::Node> node;
rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub;

void compImageCallback(const sensor_msgs::CompressedImage::ConstPtr & ros1_msg)
{
  // Declarations
  auto ros2_msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
  std_msgs::msg::Header ros2_msg_header;
  builtin_interfaces::msg::Time ros2_msg_stamp;
  
  //Header
  ros2_msg_header.stamp = node->now();
  ros2_msg_header.frame_id = ros1_msg->header.frame_id;
  
  //Build ROS 2 Message
  ros2_msg->header = ros2_msg_header;
  ros2_msg->format = ros1_msg->format;
  ros2_msg->data = ros1_msg->data;
  
  //Publish ROS 2 Message
  pub->publish(std::move(ros2_msg));
}

int main(int argc, char * argv[])
{ 
  // ROS 2 node and publisher
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("Comp_Image_Bridge_Publisher");
  node->declare_parameter("image_topic","zed2/left/image_rect_color/compressed");
  std::string image_topic_;
  node->get_parameter("image_topic",image_topic_);
  pub = node->create_publisher<sensor_msgs::msg::CompressedImage>(image_topic_, 1);
  std::cout << "comp_image_bridge: Initialized ROS 2 node" << std::endl;
  
  // ROS 1 node and subscriber
  ros::init(argc, argv, node->get_name());
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe(image_topic_, 1, compImageCallback);
  std::cout << "comp_image_bridge: Initialized ROS 1 node" << std::endl;
  // ros::spin();
  
  // ROS 1 asynchronous spinner
  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();

  // ROS 2 spinning loop
  rclcpp::executors::SingleThreadedExecutor executor;
  while (n.ok() && rclcpp::ok()) {
    executor.spin_node_once(node);
  }

  return 0;
}
