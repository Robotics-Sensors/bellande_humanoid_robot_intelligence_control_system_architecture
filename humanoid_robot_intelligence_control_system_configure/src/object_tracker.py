#!/usr/bin/env python3

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
import math
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from geometry_msgs.msg import Point

from humanoid_robot_intelligence_control_system_detection.tracker import calculate_error, calculate_error_target, calculate_delta_time
from humanoid_robot_intelligence_control_system_detection.tracker_config import TrackerConfig, TrackerInitializeConfig

class ObjectTracker:
    def __init__(self, ros_version):
        self.ros_version = ros_version
        self.initialize_config = TrackerInitializeConfig()
        self.config = TrackerConfig()
        self.initialize_ros()
        self.setup_ros_communication()

    def initialize_ros(self):
        if self.ros_version == '1':
            rospy.init_node('object_tracker')
            self.get_param = rospy.get_param
            self.logger = rospy
        else:
            rclpy.init()
            self.node = rclpy.create_node('object_tracker')
            self.get_param = self.node.get_parameter
            self.logger = self.node.get_logger()
        
        self.config.update_from_params(self.get_param)
        self.initialize_config.update_from_params(self.get_param)

    def setup_ros_communication(self):
        if self.ros_version == '1':
            self.head_joint_offset_pub = rospy.Publisher(
                "/humanoid_robot_intelligence_control_system/head_control/set_joint_states_offset", JointState, queue_size=0)
            self.head_joint_pub = rospy.Publisher(
                "/humanoid_robot_intelligence_control_system/head_control/set_joint_states", JointState, queue_size=0)
            self.head_scan_pub = rospy.Publisher(
                "/humanoid_robot_intelligence_control_system/head_control/scan_command", String, queue_size=0)
            self.object_position_sub = rospy.Subscriber(
                "/object_detector_node/object_position", Point, self.object_position_callback)
            self.object_tracking_command_sub = rospy.Subscriber(
                "/object_tracker/command", String, self.object_tracker_command_callback)
            self.prev_time = rospy.Time.now()
        else:
            self.head_joint_offset_pub = self.node.create_publisher(
                JointState, "/humanoid_robot_intelligence_control_system/head_control/set_joint_states_offset", 10)
            self.head_joint_pub = self.node.create_publisher(
                JointState, "/humanoid_robot_intelligence_control_system/head_control/set_joint_states", 10)
            self.head_scan_pub = self.node.create_publisher(
                String, "/humanoid_robot_intelligence_control_system/head_control/scan_command", 10)
            self.object_position_sub = self.node.create_subscription(
                Point, "/object_detector_node/object_position", self.object_position_callback, 10)
            self.object_tracking_command_sub = self.node.create_subscription(
                String, "/object_tracker/command", self.object_tracker_command_callback, 10)
            self.prev_time = self.node.get_clock().now()

    def object_position_callback(self, msg):
        self.initialize_config.object_position = msg
        if self.initialize_config.on_tracking:
            self.process_tracking()

    def object_tracker_command_callback(self, msg):
        if msg.data == "start":
            self.start_tracking()
        elif msg.data == "stop":
            self.stop_tracking()
        elif msg.data == "toggle_start":
            if not self.initialize_config.on_tracking:
                self.start_tracking()
            else:
                self.stop_tracking()

    def start_tracking(self):
        self.initialize_config.on_tracking = True
        self.logger.info("Start Object tracking")

    def stop_tracking(self):
        self.go_init()
        self.initialize_config.on_tracking = False
        self.logger.info("Stop Object tracking")
        self.initialize_config.current_object_pan = 0
        self.initialize_config.current_object_tilt = 0
        self.initialize_config.x_error_sum = 0
        self.initialize_config.y_error_sum = 0

    def set_using_head_scan(self, use_scan):
        self.config.use_head_scan = use_scan

    def process_tracking(self):
        if not self.initialize_config.on_tracking:
            self.initialize_config.object_position.z = 0
            self.initialize_config.count_not_found = 0
            return "NotFound"

        if self.initialize_config.object_position.z <= 0:
            self.initialize_config.count_not_found += 1
            if self.initialize_config.count_not_found < self.config.WAITING_THRESHOLD:
                if self.initialize_config.tracking_status in ["Found", "Waiting"]:
                    tracking_status = "Waiting"
                else:
                    tracking_status = "NotFound"
            elif self.initialize_config.count_not_found > self.config.NOT_FOUND_THRESHOLD:
                self.scan_object()
                self.initialize_config.count_not_found = 0
                tracking_status = "NotFound"
            else:
                tracking_status = "NotFound"
        else:
            self.initialize_config.count_not_found = 0
            tracking_status = "Found"

        if tracking_status != "Found":
            self.initialize_config.tracking_status = tracking_status
            self.initialize_config.current_object_pan = 0
            self.initialize_config.current_object_tilt = 0
            self.initialize_config.x_error_sum = 0
            self.initialize_config.y_error_sum = 0
            return tracking_status

        x_error, y_error = calculate_error(self.initialize_config.object_position, self.config.FOV_WIDTH, self.config.FOV_HEIGHT)
        object_size = self.initialize_config.object_position.z

        curr_time = rospy.Time.now() if self.ros_version == '1' else self.node.get_clock().now()
        delta_time = calculate_delta_time(curr_time, self.prev_time)
        self.prev_time = curr_time

        x_error_diff = (x_error - self.initialize_config.current_object_pan) / delta_time
        y_error_diff = (y_error - self.initialize_config.current_object_tilt) / delta_time
        self.initialize_config.x_error_sum += x_error
        self.initialize_config.y_error_sum += y_error
        
        x_error_target = calculate_error_target(x_error, x_error_diff, self.initialize_config.x_error_sum, 
                                                self.config.p_gain, self.config.i_gain, self.config.d_gain)
        y_error_target = calculate_error_target(y_error, y_error_diff, self.initialize_config.y_error_sum, 
                                                self.config.p_gain, self.config.i_gain, self.config.d_gain)

        if self.config.DEBUG_PRINT:
            self.logger.info(f"Error: {x_error * 180 / math.pi} | {y_error * 180 / math.pi}")
            self.logger.info(f"Error diff: {x_error_diff * 180 / math.pi} | {y_error_diff * 180 / math.pi} | {delta_time}")
            self.logger.info(f"Error sum: {self.initialize_config.x_error_sum * 180 / math.pi} | {self.initialize_config.y_error_sum * 180 / math.pi}")
            self.logger.info(f"Error target: {x_error_target * 180 / math.pi} | {y_error_target * 180 / math.pi}")

        self.publish_head_joint(x_error_target, y_error_target)

        self.initialize_config.current_object_pan = x_error
        self.initialize_config.current_object_tilt = y_error
        self.initialize_config.current_object_bottom = object_size

        self.initialize_config.object_position.z = 0

        self.initialize_config.tracking_status = tracking_status
        return tracking_status

    def publish_head_joint(self, pan, tilt):
        min_angle = 1 * math.pi / 180
        if abs(pan) < min_angle and abs(tilt) < min_angle:
            return

        head_angle_msg = JointState()
        head_angle_msg.name = ["head_pan", "head_tilt"]
        head_angle_msg.position = [pan, tilt]

        self.head_joint_offset_pub.publish(head_angle_msg)

    def go_init(self):
        head_angle_msg = JointState()
        head_angle_msg.name = ["head_pan", "head_tilt"]
        head_angle_msg.position = [0.0, 0.0]

        self.head_joint_pub.publish(head_angle_msg)

    def scan_object(self):
        if not self.config.use_head_scan:
            return

        scan_msg = String()
        scan_msg.data = "scan"

        self.head_scan_pub.publish(scan_msg)

    def run(self):
        if self.ros_version == '1':
            rospy.spin()
        else:
            rclpy.spin(self.node)

def main(ros_version):
    try:
        tracker = ObjectTracker(ros_version)
        tracker.start_tracking()
        tracker.run()
    except Exception as e:
        if ros_version == '1':
            rospy.logerr(f"An error occurred: {e}")
        else:
            rclpy.shutdown()
            print(f"An error occurred: {e}")

if __name__ == '__main__':
    ros_version = os.getenv("ROS_VERSION")
    if ros_version == "1":
        import rospy
        try:
            main(ros_version)
        except rospy.ROSInterruptException:
            print("Error in ROS1 main")
    elif ros_version == "2":
        import rclpy
        try:
            main(ros_version)
        except KeyboardInterrupt:
            rclpy.shutdown()
            print("Error in ROS2 main")
    else:
        print("Unsupported ROS version. Please set the ROS_VERSION environment variable to '1' for ROS 1 or '2' for ROS 2.")
        sys.exit(1)
