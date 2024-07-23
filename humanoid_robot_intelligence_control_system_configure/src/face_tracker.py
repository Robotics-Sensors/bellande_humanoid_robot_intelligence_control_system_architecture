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


import rospy
import math
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from geometry_msgs.msg import Point

class FaceTracker:
    def __init__(self):
        rospy.init_node('face_tracker')
        
        self.FOV_WIDTH = 35.2 * math.pi / 180
        self.FOV_HEIGHT = 21.6 * math.pi / 180
        self.NOT_FOUND_THRESHOLD = 50
        self.WAITING_THRESHOLD = 5
        self.use_head_scan = True
        self.count_not_found = 0
        self.on_tracking = False
        self.current_face_pan = 0
        self.current_face_tilt = 0
        self.x_error_sum = 0
        self.y_error_sum = 0
        self.tracking_status = "NotFound"
        self.DEBUG_PRINT = False

        self.p_gain = rospy.get_param('~p_gain', 0.4)
        self.i_gain = rospy.get_param('~i_gain', 0.0)
        self.d_gain = rospy.get_param('~d_gain', 0.0)

        rospy.loginfo(f"Face tracking Gain : {self.p_gain}, {self.i_gain}, {self.d_gain}")

        self.head_joint_pub = rospy.Publisher(
            "/humanoid_robot_intelligence_control_system/head_control/set_joint_states", JointState, queue_size=1)
        self.head_scan_pub = rospy.Publisher(
            "/humanoid_robot_intelligence_control_system/head_control/scan_command", String, queue_size=1)

        self.face_position_sub = rospy.Subscriber(
            "/face_detector/face_position", Point, self.face_position_callback)
        self.face_tracking_command_sub = rospy.Subscriber(
            "/face_tracker/command", String, self.face_tracker_command_callback)

        self.face_position = Point()
        self.prev_time = rospy.Time.now()

    def face_position_callback(self, msg):
        self.face_position = msg

    def face_tracker_command_callback(self, msg):
        if msg.data == "start":
            self.start_tracking()
        elif msg.data == "stop":
            self.stop_tracking()
        elif msg.data == "toggle_start":
            if not self.on_tracking:
                self.start_tracking()
            else:
                self.stop_tracking()

    def start_tracking(self):
        self.on_tracking = True
        rospy.loginfo("Start Face tracking")

    def stop_tracking(self):
        self.go_init()
        self.on_tracking = False
        rospy.loginfo("Stop Face tracking")
        self.current_face_pan = 0
        self.current_face_tilt = 0
        self.x_error_sum = 0
        self.y_error_sum = 0

    def process_tracking(self):
        if not self.on_tracking:
            self.face_position.z = 0
            self.count_not_found = 0
            return "NotFound"

        if self.face_position.z <= 0:
            self.count_not_found += 1
            if self.count_not_found < self.WAITING_THRESHOLD:
                tracking_status = "Waiting"
            elif self.count_not_found > self.NOT_FOUND_THRESHOLD:
                self.scan_face()
                self.count_not_found = 0
                tracking_status = "NotFound"
            else:
                tracking_status = "NotFound"
        else:
            self.count_not_found = 0
            tracking_status = "Found"

        if tracking_status != "Found":
            self.tracking_status = tracking_status
            return tracking_status

        x_error = -math.atan(self.face_position.x * math.tan(self.FOV_WIDTH))
        y_error = -math.atan(self.face_position.y * math.tan(self.FOV_HEIGHT))

        curr_time = rospy.Time.now()
        delta_time = (curr_time - self.prev_time).to_sec()
        self.prev_time = curr_time

        x_error_diff = (x_error - self.current_face_pan) / delta_time
        y_error_diff = (y_error - self.current_face_tilt) / delta_time
        self.x_error_sum += x_error
        self.y_error_sum += y_error
        x_error_target = x_error * self.p_gain + x_error_diff * self.d_gain + self.x_error_sum * self.i_gain
        y_error_target = y_error * self.p_gain + y_error_diff * self.d_gain + self.y_error_sum * self.i_gain

        if self.DEBUG_PRINT:
            rospy.loginfo(f"Error: {x_error * 180 / math.pi} | {y_error * 180 / math.pi}")
            rospy.loginfo(f"Error diff: {x_error_diff * 180 / math.pi} | {y_error_diff * 180 / math.pi} | {delta_time}")
            rospy.loginfo(f"Error sum: {self.x_error_sum * 180 / math.pi} | {self.y_error_sum * 180 / math.pi}")
            rospy.loginfo(f"Error target: {x_error_target * 180 / math.pi} | {y_error_target * 180 / math.pi}")

        self.publish_head_joint(x_error_target, y_error_target)

        self.current_face_pan = x_error
        self.current_face_tilt = y_error

        self.tracking_status = tracking_status
        return tracking_status

    def publish_head_joint(self, pan, tilt):
        min_angle = 1 * math.pi / 180
        if abs(pan) < min_angle and abs(tilt) < min_angle:
            return

        head_angle_msg = JointState()
        head_angle_msg.name = ["head_pan", "head_tilt"]
        head_angle_msg.position = [pan, tilt]

        self.head_joint_pub.publish(head_angle_msg)

    def go_init(self):
        head_angle_msg = JointState()
        head_angle_msg.name = ["head_pan", "head_tilt"]
        head_angle_msg.position = [0.0, 0.0]

        self.head_joint_pub.publish(head_angle_msg)

    def scan_face(self):
        if not self.use_head_scan:
            return

        scan_msg = String()
        scan_msg.data = "scan"

        self.head_scan_pub.publish(scan_msg)

    def run(self):
        rate = rospy.Rate(30)  # 30Hz
        while not rospy.is_shutdown():
            self.process_tracking()
            rate.sleep()

if __name__ == '__main__':
    try:
        tracker = FaceTracker()
        tracker.run()
    except rospy.ROSInterruptException:
        pass
