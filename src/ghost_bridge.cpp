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


#include <cstring>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

// include ROS 1
#include "ros/ros.h"
#include "ros/this_node.h"
#include "ros/header.h"
#include "ros/service_manager.h"
#include "ros/transport/transport_tcp.h"

// include ROS 2
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/scope_exit.hpp"
#include "rcutils/get_env.h"
#include "ros1_bridge/bridge.hpp"



using bridge_mapping = std::tuple<std::string, std::string, std::string>;

int main(int argc, char * argv[])
{ 
  // ROS 2 node
  rclcpp::init(argc, argv);
  auto ros2_node = rclcpp::Node::make_shared("Robot_Bridge");
  std::cout << "robot_bridge: Initialized ROS 2 node" << std::endl;

  // ROS 1 node
  ros::init(argc, argv, ros2_node->get_name());
  ros::NodeHandle ros1_node;
  std::cout << "robot_bridge: Initialized ROS 1 node" << std::endl;
  
  constexpr std::size_t queue_size = 10;
  
  auto h = ros1_bridge::create_bridge_from_1_to_2(
    ros1_node, ros2_node, "geometry_msgs/Twist", "/ghost/cmd_vel", queue_size, "geometry_msgs/msg/Twist", "/mcu/command/manual_twist",
    queue_size);
  
  // ROS 1 asynchronous spinner
  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();

  // ROS 2 spinning loop
  rclcpp::executors::SingleThreadedExecutor executor;
  while (ros1_node.ok() && rclcpp::ok()) {
    executor.spin_node_once(ros2_node);
  }

  return 0;
}