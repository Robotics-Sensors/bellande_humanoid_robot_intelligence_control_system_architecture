# Copyright (C) 2024 Bellande Robotics Sensors Research Innovation Center, Ronaldson Bellande
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import os
import sys
import subprocess
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def ros1_launch_description():
    # Get command-line arguments
    args = sys.argv[1:]

    # Construct the ROS 1 launch commandi
    roslaunch_command = ["roslaunch", "humanoid_robot_intelligence_control_system_bringup", "humanoid_robot_intelligence_control_system_bringup.launch"] + args

    # Execute the launch command
    subprocess.call(roslaunch_command)


def ros2_launch_description():
    # Create a list to hold all nodes to be launched
    nodes_to_launch = []
    
    # Add the HUMANOID_ROBOT Manager launch file
    nodes_to_launch.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            '$(find humanoid_robot_intelligence_control_system_manager)/launch/',
            'humanoid_robot_intelligence_control_system_manager.launch.py'
        ])
    ))
    
    # Add the UVC camera node
    nodes_to_launch.append(Node(
        package='usb_cam',
        executable='usb_cam_node',
        name='usb_cam_node',
        output='screen',
        parameters=[{
            'video_device': '/dev/video0',
            'image_width': 1280,
            'image_height': 720,
            'framerate': 30,
            'camera_frame_id': 'cam_link',
            'camera_name': 'camera'
        }]
    ))
    
    # Return the LaunchDescription containing all nodes
    return LaunchDescription(nodes_to_launch)


if __name__ == "__main__":
    ros_version = os.getenv("ROS_VERSION")
    if ros_version == "1":
        ros1_launch_description()
    elif ros_version == "2":
        ros2_launch_description()
    else:
        print("Unsupported ROS version. Please set the ROS_VERSION environment variable to '1' for ROS 1 or '2' for ROS 2.")
        sys.exit(1)
