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
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import FindExecutable, Command



def ros1_launch_description():
    # Get command-line arguments
    args = sys.argv[1:]
    
    # Construct the ROS 1 launch command
    roslaunch_command = ["roslaunch", "humanoid_robot_intelligence_control_system_bringup", "humanoid_robot_intelligence_control_system_bringup_visualization.launch"] + args
    
    # Execute the launch command
    subprocess.call(roslaunch_command)


def ros2_launch_description():
    # Create a list to hold all nodes to be launched
    nodes_to_launch = []
    
    # Add robot description
    nodes_to_launch.append(ExecuteProcess(
        cmd=[FindExecutable(name='xacro'), '$(find humanoid_robot_intelligence_control_system_description)/urdf/humanoid_robot_intelligence_control_system.urdf.xacro'],
        output='screen'
    ))
    
    # Add joint state publisher
    nodes_to_launch.append(Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_gui': True}],
        remappings=[('/robot/joint_states', '/humanoid_robot_intelligence_control_system/present_joint_states')]
    ))
    
    # Add robot state publisher
    nodes_to_launch.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        remappings=[('/joint_states', '/humanoid_robot_intelligence_control_system/present_joint_states')]
    ))
    
    # Add rviz
    nodes_to_launch.append(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', '$(find humanoid_robot_intelligence_control_system_bringup)/rviz/humanoid_robot_intelligence_control_system_bringup.rviz']
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
