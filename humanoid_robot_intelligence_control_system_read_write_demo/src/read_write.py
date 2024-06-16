#!/usr/bin/env python
# Copyright (C) 2024 Bellande Robotics Sensors Research Innovation Center, Ronaldson Bellande
# 
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
# 
# http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from humanoid_robot_intelligence_control_system_controller_msgs.srv import SetModule
from humanoid_robot_intelligence_control_system_controller_msgs.msg import SyncWriteItem

def button_handler_callback(msg):
    global control_module
    if msg.data == "mode":
        control_module = ControlModule.Framework
        rospy.loginfo("Button : mode | Framework")
        ready_to_demo()
    elif msg.data == "start":
        control_module = ControlModule.DirectControlModule
        rospy.loginfo("Button : start | Direct control module")
        ready_to_demo()
    elif msg.data == "user":
        torque_on_all()
        control_module = ControlModule.None

def joint_states_callback(msg):
    if control_module == ControlModule.None:
        return

    write_msg = JointState()
    write_msg.header = msg.header

    for ix in range(len(msg.name)):
        joint_name = msg.name[ix]
        joint_position = msg.position[ix]

        if joint_name == "r_sho_pitch":
            write_msg.name.append("r_sho_pitch")
            write_msg.position.append(joint_position)
            write_msg.name.append("l_sho_pitch")
            write_msg.position.append(-joint_position)
        if joint_name == "r_sho_roll":
            write_msg.name.append("r_sho_roll")
            write_msg.position.append(joint_position)
            write_msg.name.append("l_sho_roll")
            write_msg.position.append(-joint_position)
        if joint_name == "r_el":
            write_msg.name.append("r_el")
            write_msg.position.append(joint_position)
            write_msg.name.append("l_el")
            write_msg.position.append(-joint_position)

    if control_module == ControlModule.Framework:
        write_joint_pub.publish(write_msg)
    elif control_module == ControlModule.DirectControlModule:
        write_joint_pub2.publish(write_msg)

def ready_to_demo():
    rospy.loginfo("Start Read-Write Demo")
    set_led(0x04)
    torque_on_all()
    rospy.loginfo("Torque on All joints")
    go_init_pose()
    rospy.loginfo("Go Init pose")
    rospy.sleep(4.0)
    set_led(control_module)
    if control_module == ControlModule.Framework:
        set_module("none")
        rospy.loginfo("Change module to none")
    elif control_module == ControlModule.DirectControlModule:
        set_module("direct_control_module")
        rospy.loginfo("Change module to direct_control_module")
    else:
        return
    torque_off("right")
    rospy.loginfo("Torque off")

def go_init_pose():
    init_msg = String()
    init_msg.data = "ini_pose"
    init_pose_pub.publish(init_msg)

def set_led(led):
    syncwrite_msg = SyncWriteItem()
    syncwrite_msg.item_name = "LED"
    syncwrite_msg.joint_name.append("open-cr")
    syncwrite_msg.value.append(led)
    sync_write_pub.publish(syncwrite_msg)

def check_manager_running(manager_name):
    node_list = rospy.get_published_topics()
    for node in node_list:
        if node[0] == manager_name:
            return True
    rospy.logerr("Can't find humanoid_robot_intelligence_control_system_manager")
    return False

def set_module(module_name):
    try:
        set_module_srv = rospy.ServiceProxy('/humanoid_robot_intelligence_control_system/set_present_ctrl_modules', SetModule)
        set_module_srv(module_name)
    except rospy.ServiceException as e:
        rospy.logerr("Failed to set module: %s" % e)

def torque_on_all():
    check_msg = String()
    check_msg.data = "check"
    dxl_torque_pub.publish(check_msg)

def torque_off(body_side):
    syncwrite_msg = SyncWriteItem()
    torque_value = 0
    syncwrite_msg.item_name = "torque_enable"
    if body_side == "right":
        syncwrite_msg.joint_name.extend(["r_sho_pitch", "r_sho_roll", "r_el"])
        syncwrite_msg.value.extend([torque_value] * 3)
    elif body_side == "left":
        syncwrite_msg.joint_name.extend(["l_sho_pitch", "l_sho_roll", "l_el"])
        syncwrite_msg.value.extend([torque_value] * 3)
    sync_write_pub.publish(syncwrite_msg)

class ControlModule:
    None = 0
    DirectControlModule = 1
    Framework = 2

if __name__ == '__main__':
    rospy.init_node('read_write', anonymous=True)
    
    init_pose_pub = rospy.Publisher('/humanoid_robot_intelligence_control_system/base/ini_pose', String, queue_size=10)
    sync_write_pub = rospy.Publisher('/humanoid_robot_intelligence_control_system/sync_write_item', SyncWriteItem, queue_size=10)
    dxl_torque_pub = rospy.Publisher('/humanoid_robot_intelligence_control_system/dxl_torque', String, queue_size=10)
    write_joint_pub = rospy.Publisher('/humanoid_robot_intelligence_control_system/set_joint_states', JointState, queue_size=10)
    write_joint_pub2 = rospy.Publisher('/humanoid_robot_intelligence_control_system/direct_control/set_joint_states', JointState, queue_size=10)

    read_joint_sub = rospy.Subscriber('/humanoid_robot_intelligence_control_system/present_joint_states', JointState, joint_states_callback)
    button_sub = rospy.Subscriber('/humanoid_robot_intelligence_control_system/open_cr/button', String, button_handler_callback)

    control_module = ControlModule.None
    demo_ready = False
    SPIN_RATE = 30
    DEBUG_PRINT = False
    
    loop_rate = rospy.Rate(SPIN_RATE)

    manager_name = "/humanoid_robot_intelligence_control_system_manager"
    while not rospy.is_shutdown():
        rospy.sleep(1.0)
        if check_manager_running(manager_name):
            rospy.loginfo("Succeed to connect")
            break
        rospy.logwarn("Waiting for humanoid_robot_intelligence_control_system manager")

    ready_to_demo()

    while not rospy.is_shutdown():
        rospy.spin()
        loop_rate.sleep()
