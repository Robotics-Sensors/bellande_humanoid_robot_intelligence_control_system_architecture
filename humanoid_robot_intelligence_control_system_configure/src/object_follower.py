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
from humanoid_robot_intelligence_control_system_walking_module_msgs.msg import WalkingParam
from humanoid_robot_intelligence_control_system_walking_module_msgs.srv import GetWalkingParam

from humanoid_robot_intelligence_control_system_detection.follower import calculate_distance_to_object, calculate_footstep, calculate_delta_time
from humanoid_robot_intelligence_control_system_detection.follower_config import FollowerConfig, FollowerInitializeConfig

class ObjectFollower:
    def __init__(self, ros_version):
        self.ros_version = ros_version
        self.initialize_config = FollowerInitializeConfig()
        self.config = FollowerConfig()
        self.initialize_ros()
        self.setup_ros_communication()

    def initialize_ros(self):
        if self.ros_version == '1':
            rospy.init_node('object_follower')
            self.get_param = rospy.get_param
            self.logger = rospy
        else:
            rclpy.init()
            self.node = rclpy.create_node('object_follower')
            self.get_param = self.node.get_parameter
            self.logger = self.node.get_logger()
        
        self.config.update_from_params(self.get_param)
        self.initialize_config.update_from_params(self.get_param)

    def setup_ros_communication(self):
        if self.ros_version == '1':
            self.current_joint_states_sub = rospy.Subscriber(
                "/humanoid_robot_intelligence_control_system/goal_joint_states", JointState, self.current_joint_states_callback)
            self.set_walking_command_pub = rospy.Publisher(
                "/humanoid_robot_intelligence_control_system/walking/command", String, queue_size=1)
            self.set_walking_param_pub = rospy.Publisher(
                "/humanoid_robot_intelligence_control_system/walking/set_params", WalkingParam, queue_size=1)
            self.get_walking_param_client = rospy.ServiceProxy(
                "/humanoid_robot_intelligence_control_system/walking/get_params", GetWalkingParam)
            self.object_detection_sub = rospy.Subscriber(
                "/object_detection_result", Point, self.object_detection_callback)
            self.prev_time = rospy.Time.now()
        else:
            self.current_joint_states_sub = self.node.create_subscription(
                JointState, "/humanoid_robot_intelligence_control_system/goal_joint_states", self.current_joint_states_callback, 10)
            self.set_walking_command_pub = self.node.create_publisher(
                String, "/humanoid_robot_intelligence_control_system/walking/command", 1)
            self.set_walking_param_pub = self.node.create_publisher(
                WalkingParam, "/humanoid_robot_intelligence_control_system/walking/set_params", 1)
            self.get_walking_param_client = self.node.create_client(
                GetWalkingParam, "/humanoid_robot_intelligence_control_system/walking/get_params")
            self.object_detection_sub = self.node.create_subscription(
                Point, "/object_detection_result", self.object_detection_callback, 10)
            self.prev_time = self.node.get_clock().now()

    def start_following(self):
        self.initialize_config.on_tracking = True
        self.logger.info("Start Object following")
        self.set_walking_command("start")
        result = self.get_walking_param()
        if result:
            self.config.hip_pitch_offset = self.current_walking_param.hip_pitch_offset
            self.config.curr_period_time = self.current_walking_param.period_time
        else:
            self.config.hip_pitch_offset = 7.0 * math.pi / 180
            self.config.curr_period_time = 0.6

    def stop_following(self):
        self.initialize_config.on_tracking = False
        self.initialize_config.count_to_approach = 0
        self.logger.info("Stop Object following")
        self.set_walking_command("stop")

    def current_joint_states_callback(self, msg):
        for i, name in enumerate(msg.name):
            if name == "head_pan":
                self.initialize_config.current_pan = msg.position[i]
            elif name == "head_tilt":
                self.initialize_config.current_tilt = msg.position[i]

    def process_following(self, x_angle, y_angle, object_size):
        curr_time = rospy.Time.now() if self.ros_version == '1' else self.node.get_clock().now()
        delta_time = calculate_delta_time(curr_time, self.prev_time)
        self.prev_time = curr_time

        self.initialize_config.count_not_found = 0

        if self.initialize_config.current_tilt == -10 and self.initialize_config.current_pan == -10:
            self.logger.error("Failed to get current angle of head joints.")
            self.set_walking_command("stop")
            self.initialize_config.on_tracking = False
            self.initialize_config.approach_object_position = "NotFound"
            return False

        self.initialize_config.approach_object_position = "OutOfRange"

        distance_to_object = calculate_distance_to_object(
            self.config.CAMERA_HEIGHT, self.initialize_config.current_tilt, self.config.hip_pitch_offset, object_size)

        distance_to_approach = 0.22

        if (distance_to_object < distance_to_approach) and (abs(x_angle) < 25.0):
            self.initialize_config.count_to_approach += 1
            if self.initialize_config.count_to_approach > 20:
                self.set_walking_command("stop")
                self.initialize_config.on_tracking = False
                self.initialize_config.approach_object_position = "OnLeft" if x_angle > 0 else "OnRight"
                return True
            elif self.initialize_config.count_to_approach > 15:
                self.set_walking_param(self.config.IN_PLACE_FB_STEP, 0, 0)
                return False
        else:
            self.initialize_config.count_to_approach = 0

        distance_to_walk = distance_to_object - distance_to_approach
        fb_move, rl_angle = calculate_footstep(
            self.initialize_config.current_x_move, distance_to_walk, self.initialize_config.current_r_angle, self.initialize_config.current_pan, delta_time, self.config)
        self.set_walking_param(fb_move, 0, rl_angle)
        return False

    def decide_object_position(self, x_angle, y_angle):
        if self.initialize_config.current_tilt == -10 and self.initialize_config.current_pan == -10:
            self.initialize_config.approach_object_position = "NotFound"
            return
        object_x_angle = self.initialize_config.current_pan + x_angle
        self.initialize_config.approach_object_position = "OnLeft" if object_x_angle > 0 else "OnRight"

    def wait_following(self):
        self.initialize_config.count_not_found += 1
        if self.initialize_config.count_not_found > self.config.NOT_FOUND_THRESHOLD * 0.5:
            self.set_walking_param(0.0, 0.0, 0.0)

    def set_walking_command(self, command):
        if command == "start":
            self.get_walking_param()
            self.set_walking_param(self.config.IN_PLACE_FB_STEP, 0, 0, True)
        msg = String()
        msg.data = command
        self.set_walking_command_pub.publish(msg)

    def set_walking_param(self, x_move, y_move, rotation_angle, balance=False):
        self.current_walking_param.balance_enable = balance
        self.current_walking_param.x_move_amplitude = x_move + self.config.SPOT_FB_OFFSET
        self.current_walking_param.y_move_amplitude = y_move + self.config.SPOT_RL_OFFSET
        self.current_walking_param.angle_move_amplitude = rotation_angle + self.config.SPOT_ANGLE_OFFSET
        self.set_walking_param_pub.publish(self.current_walking_param)
        self.initialize_config.current_x_move = x_move
        self.initialize_config.current_r_angle = rotation_angle

    def get_walking_param(self):
        if self.ros_version == '1':
            try:
                response = self.get_walking_param_client()
                self.current_walking_param = response.parameters
                return True
            except rospy.ServiceException as e:
                self.logger.error("Failed to get walking parameters: %s" % e)
                return False
        else:
            future = self.get_walking_param_client.call_async(GetWalkingParam.Request())
            rclpy.spin_until_future_complete(self.node, future)
            if future.result() is not None:
                self.current_walking_param = future.result().parameters
                return True
            else:
                self.logger.error('Service call failed %r' % (future.exception(),))
                return False

    def object_detection_callback(self, msg):
        if self.initialize_config.on_tracking:
            x_angle = msg.x
            y_angle = msg.y
            object_size = msg.z
            self.process_following(x_angle, y_angle, object_size)
        else:
            self.wait_following()

    def run(self):
        if self.ros_version == '1':
            rospy.spin()
        else:
            rclpy.spin(self.node)

def main(ros_version):
    try:
        follower = ObjectFollower(ros_version)
        follower.start_following()
        follower.run()
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
