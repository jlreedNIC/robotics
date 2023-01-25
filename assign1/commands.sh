#!/bin/bash

ros2 action send_goal /create3_0620/undock irobot_create_msgs/action/Undock {}

ros2 action send_goal /create3_0620/drive_distance irobot_create_msgs/action/DriveDistance "{distance: 1.0}"

ros2 action send_goal /create3_0620/rotate_angle irobot_create_msgs/action/RotateAngle "{angle: .785}"

ros2 action send_goal /create3_0620/drive_distance irobot_create_msgs/action/DriveDistance "{distance: 0.5}"

ros2 action send_goal /create3_0620/rotate_angle irobot_create_msgs/action/RotateAngle "{angle: 2.36}"

ros2 action send_goal /create3_0620/drive_distance irobot_create_msgs/action/DriveDistance "{distance: 0.6}"

ros2 action send_goal /create3_0620/dock irobot_create_msgs/action/Dock {}