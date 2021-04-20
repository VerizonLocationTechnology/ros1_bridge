# ros1_bridge - w/ vlt custom bridges

## Background
This is a port of the [ros1_bridge](https://github.com/ros2/ros1_bridge/tree/foxy) (foxy branch). A few custom bridges have built upon this repository that will be described below, as it seemed the dynamic_bridge was not able to bridge communications fast enough. The original README for the ros1_bridge can be found [here](ROS1_BRIDGE_README.md).

## Robot Bridge
This bridge bridges all necessary topics from ros1 to ros2, and vice versa, for a robot running a zed2. It's currently configured for the clearpath robots.

**ros1 -> ros2:**
- /tf: [TFMessage](http://docs.ros.org/en/jade/api/tf2_msgs/html/msg/TFMessage.html)
- /zed2/left/image_rect_color/compressed: [CompressedImage](https://docs.ros.org/en/api/sensor_msgs/html/msg/CompressedImage.html)
- /zed2/right/image_rect_color/compressed: [CompressedImage](https://docs.ros.org/en/api/sensor_msgs/html/msg/CompressedImage.html)
- /zed2/left/camera_info: [CameraInfo](https://docs.ros.org/en/api/sensor_msgs/html/msg/CameraInfo.html)
- /zed2/right/camera_info: [CameraInfo](https://docs.ros.org/en/api/sensor_msgs/html/msg/CameraInfo.html)


**ros2 -> ros1:**
- /cmd_vel: [Twist](https://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html)

**Note:** Because of behavior where, even with the correct qos settings, static transforms can sometimes not be found by Rtabmap and Nav2, we create a custom secondary bridge for these to go along with the robot bridge. This bridge constantly republishes the same static transforms, but with updated timestamps.
- /tf_static: [TFMessage](http://docs.ros.org/en/jade/api/tf2_msgs/html/msg/TFMessage.html)

## Launch Robot Bridge
To launch bridge (includes static_tf bridge) for a robot with a Zed2 camera (like the jackal), run the following command.

```
ros2 launch ros1_bridge robot_bridge.launch.py namespace:=jackal
```

## Adding New Topics to Robot Bridge
To add new topics to this bridge, simply add them to the file [here](https://git.vzbuilders.com/VLS/ros1_bridge/blob/feature/vlt-amr-797/src/robot_bridge.cpp#L44-L55).

The syntax is (all strings):
  ```
  {topic_name, ros1_msg_type, ros2_msg_type}
  ```
  
  So a bridge for transforms on the /tf topic would look like this.
  
  ```
  {"tf", "tf2_msgs/TFMessage", "tf2_msgs/msg/TFMessage"},
  ```
  
  **Note:** Make sure that, in order to namespace these properly for robots, you do no add the leading "/" to the topic names.

## Depracated

#### Custom Bridges
Workspace for ros1_bridge with custom build bridges as below.

**ros1 -> ros2:**
- tf_bridge: [TFMessage](http://docs.ros.org/en/jade/api/tf2_msgs/html/msg/TFMessage.html)
- tf_static_bridge: [TFMessage](http://docs.ros.org/en/jade/api/tf2_msgs/html/msg/TFMessage.html)
- comp_image_bridge: [CompressedImage](https://docs.ros.org/en/api/sensor_msgs/html/msg/CompressedImage.html)
- camera_info_bridge: [CameraInfo](https://docs.ros.org/en/api/sensor_msgs/html/msg/CameraInfo.html)

**ros2 -> ros1:**
- twist_bridge [Twist](https://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html)

#### Launch Bridges
You can launch a bridge individually, the launch files are located [here](launch/).

To launch bridges for a robot with a Zed2 camera (like the jackal), run the following command.

```
ros2 launch ros1_bridge jackal_bridges.launch.py namespace:=jackal
```

This will launch:
- **(2) comp_image_bridges**: one each for the left and right camera on the Zed2
- **(2) camera_info_bridge**: one each for the left and right camera on the Zed2
- **(1) tf_bridge**: bridging transforms
- **(1) tf_static_bridge**: bridging static transforms and repeatedly publishing them on the ros2 side
- **(1) twist_bridge**: for controlling (through twist messages) a ros1 robot from ros2