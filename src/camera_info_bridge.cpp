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
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/RegionOfInterest.h"


// include ROS 2
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>

std::shared_ptr<rclcpp::Node> node;
rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub;

void camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr & ros1_msg)
{
  // Declarations
  auto ros2_msg = std::make_unique<sensor_msgs::msg::CameraInfo>();
  std_msgs::msg::Header ros2_msg_header;
  builtin_interfaces::msg::Time ros2_msg_stamp;
  sensor_msgs::msg::RegionOfInterest ros2_msg_roi;

  // Header
  ros2_msg_header.stamp =  node->now();
  ros2_msg_header.frame_id = ros1_msg->header.frame_id;
  ros2_msg->header = ros2_msg_header;
  
  // Distortion Model
  ros2_msg->distortion_model = ros1_msg->distortion_model;
  
  // Height and Width
  ros2_msg->height = ros1_msg->height;
  ros2_msg->width = ros1_msg->width;
  
  //d, k, r, p
  ros2_msg->d = ros1_msg->D;
  
  for (unsigned long int i = 0; i < ros1_msg->K.size(); i++){
    ros2_msg->k[i] = ros1_msg->K[i];
  }
  
  for (unsigned long int i = 0; i < ros1_msg->R.size(); i++){
    ros2_msg->r[i] = ros1_msg->R[i];
  }
  
  for (unsigned long int i = 0; i < ros1_msg->P.size(); i++){
    ros2_msg->p[i] = ros1_msg->P[i];
  }
  
  // Binning
  ros2_msg->binning_x = ros1_msg->binning_x;
  ros2_msg->binning_y = ros1_msg->binning_y;
  
  // ROI
  ros2_msg_roi.x_offset = ros1_msg->roi.x_offset;
  ros2_msg_roi.y_offset = ros1_msg->roi.y_offset;
  ros2_msg_roi.height = ros1_msg->roi.height;
  ros2_msg_roi.width = ros1_msg->roi.width;
  ros2_msg_roi.do_rectify = ros1_msg->roi.do_rectify;
  ros2_msg->roi = ros2_msg_roi;
  
  //Publish ROS 2 Message
  pub->publish(std::move(ros2_msg));
}

int main(int argc, char * argv[])
{ 
  // ROS 2 node and publisher
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("Camera_Info_Bridge_Publisher");
  node->declare_parameter("topic","zed2/left/camera_info");
  std::string topic_;
  node->get_parameter("topic",topic_);
  pub = node->create_publisher<sensor_msgs::msg::CameraInfo>(topic_, 1);
  std::cout << "camera_info_bridge: Initialized ROS 2 node" << std::endl;

  // ROS 1 node and subscriber
  ros::init(argc, argv, node->get_name());
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe(topic_, 1, camInfoCallback);
  std::cout << "camera_info_bridge: Initialized ROS 1 node" << std::endl;
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
