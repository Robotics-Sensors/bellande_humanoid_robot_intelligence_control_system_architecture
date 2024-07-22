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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def ros1_launch_description():
    # Get command-line arguments
    args = sys.argv[1:]
    
    # Construct the ROS 1 launch command
    roslaunch_command = ["roslaunch", "humanoid_robot_intelligence_control_system_read_write_demo", "humanoid_robot_intelligence_control_system_read_write.launch"] + args
    
    # Add parameters
    roslaunch_command.extend([
        "gazebo:=false",
        "gazebo_robot_name:=humanoid_robot_intelligence_control_system",
        "offset_file_path:=$(find humanoid_robot_intelligence_control_system_manager)/config/offset.yaml",
        "robot_file_path:=$(find humanoid_robot_intelligence_control_system_manager)/config/HUMANOID_ROBOT.robot",
        "init_file_path:=$(find humanoid_robot_intelligence_control_system_manager)/config/dxl_init_HUMANOID_ROBOT.yaml",
        "device_name:=/dev/ttyUSB0",
        "/humanoid_robot_intelligence_control_system/direct_control/default_moving_time:=0.04",
        "/humanoid_robot_intelligence_control_system/direct_control/default_moving_angle:=90"
    ])
    
    # Add nodes
    roslaunch_command.extend([
        "humanoid_robot_intelligence_control_system_manager", "humanoid_robot_intelligence_control_system_manager", "angle_unit:=30",
        "humanoid_robot_intelligence_control_system_localization", "humanoid_robot_intelligence_control_system_localization",
        "humanoid_robot_intelligence_control_system_read_write_demo", "read_write"
    ])
    
    # Execute the launch command
    subprocess.call(roslaunch_command)


def ros2_launch_description():
    # Declare launch arguments
    gazebo_arg = DeclareLaunchArgument('gazebo', default_value='false')
    gazebo_robot_name_arg = DeclareLaunchArgument('gazebo_robot_name', default_value='humanoid_robot_intelligence_control_system')
    offset_file_path_arg = DeclareLaunchArgument('offset_file_path', default_value='$(find humanoid_robot_intelligence_control_system_manager)/config/offset.yaml')
    robot_file_path_arg = DeclareLaunchArgument('robot_file_path', default_value='$(find humanoid_robot_intelligence_control_system_manager)/config/HUMANOID_ROBOT.robot')
    init_file_path_arg = DeclareLaunchArgument('init_file_path', default_value='$(find humanoid_robot_intelligence_control_system_manager)/config/dxl_init_HUMANOID_ROBOT.yaml')
    device_name_arg = DeclareLaunchArgument('device_name', default_value='/dev/ttyUSB0')
    
    # Create a list to hold all nodes to be launched
    nodes_to_launch = []
    
    # Add HUMANOID_ROBOT Manager node
    nodes_to_launch.append(Node(
        package='humanoid_robot_intelligence_control_system_manager',
        executable='humanoid_robot_intelligence_control_system_manager',
        name='humanoid_robot_intelligence_control_system_manager',
        output='screen',
        parameters=[
            {'angle_unit': 30},
            {'gazebo': LaunchConfiguration('gazebo')},
            {'gazebo_robot_name': LaunchConfiguration('gazebo_robot_name')},
            {'offset_file_path': LaunchConfiguration('offset_file_path')},
            {'robot_file_path': LaunchConfiguration('robot_file_path')},
            {'init_file_path': LaunchConfiguration('init_file_path')},
            {'device_name': LaunchConfiguration('device_name')},
            {'/humanoid_robot_intelligence_control_system/direct_control/default_moving_time': 0.04},
            {'/humanoid_robot_intelligence_control_system/direct_control/default_moving_angle': 90}
        ]
    ))
    
    # Add HUMANOID_ROBOT Localization node
    nodes_to_launch.append(Node(
        package='humanoid_robot_intelligence_control_system_localization',
        executable='humanoid_robot_intelligence_control_system_localization',
        name='humanoid_robot_intelligence_control_system_localization',
        output='screen'
    ))
    
    # Add HUMANOID_ROBOT Read-Write demo node
    nodes_to_launch.append(Node(
        package='humanoid_robot_intelligence_control_system_read_write_demo',
        executable='read_write',
        name='humanoid_robot_intelligence_control_system_read_write',
        output='screen'
    ))
    
    # Return the LaunchDescription containing all nodes and arguments
    return LaunchDescription([
        gazebo_arg,
        gazebo_robot_name_arg,
        offset_file_path_arg,
        robot_file_path_arg,
        init_file_path_arg,
        device_name_arg
    ] + nodes_to_launch)


if __name__ == "__main__":
    ros_version = os.getenv("ROS_VERSION")
    if ros_version == "1":
        ros1_launch_description()
    elif ros_version == "2":
        ros2_launch_description()
    else:
        print("Unsupported ROS version. Please set the ROS_VERSION environment variable to '1' for ROS 1 or '2' for ROS 2.")
        sys.exit(1)
