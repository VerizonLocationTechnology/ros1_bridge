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
#include <cstring>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

// include ROS 1
#include "ros/ros.h"
#include "tf2_msgs/TFMessage.h"

// // include ROS 2
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/qos.hpp>
#include <tf2_ros/visibility_control.h>
#include <tf2_ros/static_transform_broadcaster.h>

using namespace std::chrono_literals;

std::shared_ptr<rclcpp::Node> node;
rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr pub;
std::vector<tf2_msgs::msg::TFMessage> memory; //timer

void timerCallback(){ //timer
  auto stamp = node->now();
  for (unsigned long int i = 0; i < memory.size(); i++){
    for (unsigned long int j = 0; j < memory[i].transforms.size(); j++){
      memory[i].transforms[j].header.stamp = stamp;
    }
    pub->publish(std::move(memory[i]));
  }  
}

void tfCallback(const tf2_msgs::TFMessage::ConstPtr & ros1_msg)
{
  auto ros2_msg = std::make_unique<tf2_msgs::msg::TFMessage>();
  std::cout << "IN ROS1 TF STATIC CALLBACK" << std::endl;
  for (unsigned long int i = 0; i < ros1_msg -> transforms.size(); i++){
    
    // Declarations
    auto ros1_tf = ros1_msg -> transforms[i];
    std_msgs::msg::Header ros2_tf_header;
    geometry_msgs::msg::TransformStamped ros2_tf_stamped;
    builtin_interfaces::msg::Time ros2_stamp;
    geometry_msgs::msg::Transform ros2_tf;
    geometry_msgs::msg::Quaternion ros2_quaternion;
    geometry_msgs::msg::Vector3 ros2_vector3;
    
    //Header
    ros2_tf_header.stamp = node->now();
    ros2_tf_header.frame_id = ros1_tf.header.frame_id;
    
    //Child Frame
    ros2_tf_stamped.child_frame_id = ros1_tf.child_frame_id;
    
    //    Vector3
    ros2_vector3.x = ros1_tf.transform.translation.x;
    ros2_vector3.y = ros1_tf.transform.translation.y;
    ros2_vector3.z = ros1_tf.transform.translation.z;
    
    //    Quaternion
    ros2_quaternion.x = ros1_tf.transform.rotation.x;
    ros2_quaternion.y = ros1_tf.transform.rotation.y;
    ros2_quaternion.z = ros1_tf.transform.rotation.z;
    ros2_quaternion.w = ros1_tf.transform.rotation.w;
    
    //Transform
    ros2_tf.translation = ros2_vector3;
    ros2_tf.rotation = ros2_quaternion;
    
    //Transform Stamped
    ros2_tf_stamped.header = ros2_tf_header;
    ros2_tf_stamped.transform = ros2_tf;
    
    //Push Stamped transform into vector
    ros2_msg->transforms.push_back(ros2_tf_stamped); //timer
  }
  
  memory.push_back(*ros2_msg); //timer
  //Publish ROS 2 Message
  // pub->publish(std::move(ros2_msg)); //MOVED TO TIMER CALLBACK
}

int main(int argc, char * argv[])
{ 
  
  // ROS 2 node and publisher
  rclcpp::init(argc, argv);
  
  node = rclcpp::Node::make_shared("TF2_Bridge");
  node->declare_parameter("topic","tf_static");
  std::string topic_;
  node->get_parameter("topic",topic_);
  std::cout << "STATIC TF TOPIC, SETTING QOS APPROPRIATELY" << std::endl;
  auto ros2_publisher_qos = rclcpp::QoS(10);
  ros2_publisher_qos.reliable();
  // ros2_publisher_qos.keep_all(); #don't need this since we repeatedly publish static transforms
  ros2_publisher_qos.transient_local();
  auto options = rclcpp::PublisherOptionsWithAllocator<std::allocator<void>>();
  pub = node->create_publisher<tf2_msgs::msg::TFMessage>(topic_, ros2_publisher_qos, options);
  auto timer_ = node->create_wall_timer(100ms, std::bind(&timerCallback)); //10 hz timer
  
  
  std::cout << "tf_bridge: Initialized ROS 2 node" << std::endl;
  
  // ROS 1 node and subscriber
  ros::init(argc, argv, node->get_name());
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe(topic_, 10, tfCallback);
  std::cout << "tf_bridge: Initialized ROS 1 node" << std::endl;
  
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
