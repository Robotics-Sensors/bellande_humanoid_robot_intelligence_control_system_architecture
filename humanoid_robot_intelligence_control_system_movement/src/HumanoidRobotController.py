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

import time
import math
from typing import Dict, List, Tuple, Any

from humanoid_robot_intelligence_control_system_controller.PIDController import PIDController
from humanoid_robot_intelligence_control_system_detection.tracker_config import TrackerConfig
from humanoid_robot_intelligence_control_system_movement.MovementController import MovementController


class HumanoidRobotController:
    def __init__(self, config: TrackerConfig, controller_type: str = "PID"):
        self.config = config
        self.controller_type = controller_type
        self.controllers: Dict[str, Any] = {}
        self.joint_positions: Dict[str, float] = {}
        self.target_positions: Dict[str, float] = {}
        self.movement_controller = MovementController(config, controller_type)
        self.current_balance = 0.0
        self.current_speed = 0.0
        self.current_angle = 0.0
        self.current_position = (0.0, 0.0)
        self.last_joint_positions = {}
        self.control_loop_time = 0.01  # 100 Hz

        self.initialize_controllers()
        self.initialize_sensors()


    def initialize_controllers(self):
        joints = [
            "head_pan", "head_tilt",
            "left_shoulder_pitch", "left_shoulder_roll", "left_elbow",
            "right_shoulder_pitch", "right_shoulder_roll", "right_elbow",
            "left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle_pitch", "left_ankle_roll",
            "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle_pitch", "right_ankle_roll"
        ]

        for joint in joints:
            self.controllers[joint] = self._create_controller(joint)
            self.joint_positions[joint] = 0.0
            self.target_positions[joint] = 0.0

        self.controllers["balance"] = self._create_controller("balance")
        self.controllers["walk_forward"] = self._create_controller("walk_forward")
        self.controllers["turn"] = self._create_controller("turn")

        self.roll_controller = self._create_controller("roll")
        self.pitch_controller = self._create_controller("pitch")


    def _create_controller(self, controller_name: str) -> Any:
        if self.controller_type == "PID":
            return PIDController(
                getattr(self.config, f"get_{controller_name}_pid_gains")(),
                f"{controller_name}_controller",
                output_limits=self.config.get_joint_limits(controller_name)
            )
        elif self.controller_type == "BellandeController":
            return BellandeController(
                getattr(self.config, f"get_{controller_name}_bellande_params")(),
                f"{controller_name}_controller"
            )
        else:
            raise ValueError(f"Unsupported controller type: {self.controller_type}")


    def initialize_sensors(self):
        self.imu = None
        self.power_sensor = None
        self.joint_sensors = None


    def update_config(self, new_config: TrackerConfig):
        self.config = new_config
        for joint, controller in self.controllers.items():
            self._update_controller(controller, joint)
        self.movement_controller.update_config(new_config)


    def _update_controller(self, controller: Any, controller_name: str):
        if self.controller_type == "PID":
            controller.update_config(getattr(self.config, f"get_{controller_name}_pid_gains")())
        elif self.controller_type == "BellandeController":
            controller.update_config(getattr(self.config, f"get_{controller_name}_bellande_params")())


    def set_target_position(self, joint: str, position: float):
        if joint in self.target_positions:
            self.target_positions[joint] = position
        else:
            raise ValueError(f"Unknown joint: {joint}")


    def update_joint_position(self, joint: str, position: float):
        if joint in self.joint_positions:
            self.joint_positions[joint] = position
        else:
            raise ValueError(f"Unknown joint: {joint}")


    def compute_joint_control(self, joint: str) -> float:
        if joint not in self.controllers:
            raise ValueError(f"Unknown joint: {joint}")
        
        return self.controllers[joint].compute(
            self.target_positions[joint],
            self.joint_positions[joint]
        )


    def compute_all_joint_controls(self) -> Dict[str, float]:
        return {joint: self.compute_joint_control(joint) for joint in self.joint_positions.keys()}


    def compute_balance_control(self, current_balance: float) -> float:
        return self.controllers["balance"].compute(0, current_balance)


    def compute_walk_forward_control(self, current_speed: float, target_speed: float) -> float:
        return self.controllers["walk_forward"].compute(target_speed, current_speed)


    def compute_turn_control(self, current_angle: float, target_angle: float) -> float:
        return self.controllers["turn"].compute(target_angle, current_angle)


    def reset_all_controllers(self):
        for controller in self.controllers.values():
            controller.reset()


    def update_robot_state(self, joint_positions: Dict[str, float], balance: float, speed: float, angle: float, position: Tuple[float, float]):
        for joint, position in joint_positions.items():
            self.update_joint_position(joint, position)
        
        self.current_balance = balance
        self.current_speed = speed
        self.current_angle = angle
        self.current_position = position


    def compute_full_robot_control(self, target_speed: float, target_angle: float, target_position: Tuple[float, float]) -> Dict[str, float]:
        joint_controls = self.compute_all_joint_controls()
        balance_control = self.compute_balance_control(self.current_balance)
        walk_control = self.compute_walk_forward_control(self.current_speed, target_speed)
        turn_control = self.compute_turn_control(self.current_angle, target_angle)

        translation, rotation = self.movement_controller.compute_path_control(self.current_position, target_position)

        return {
            "joints": joint_controls,
            "balance": balance_control,
            "walk": walk_control,
            "turn": turn_control,
            "translation": translation,
            "rotation": rotation
        }


    def execute_control(self, control: Dict[str, float]):
        try:
            # Send joint control commands
            for joint, value in control['joints'].items():
                self.robot_interface.set_joint_position(joint, value)

            # Send balance control command
            self.robot_interface.set_balance_adjustment(control['balance'])

            # Send walk and turn commands
            self.robot_interface.set_walk_speed(control['walk'])
            self.robot_interface.set_turn_rate(control['turn'])

            # Send translation and rotation commands
            self.robot_interface.set_body_translation(control['translation'])
            self.robot_interface.set_body_rotation(control['rotation'])

            # Verify that commands were executed
            if not self.robot_interface.verify_last_command():
                raise Exception("Failed to execute control commands")

        except Exception as e:
            print(f"Error executing control: {e}")
            self.emergency_stop()
    

    def get_robot_state(self) -> Dict[str, Any]:
        joint_positions = self.joint_sensors.get_positions()
        imu_data = self.imu.get_data()
        return {
            "joint_positions": joint_positions,
            "balance": imu_data.acceleration.z,
            "speed": math.sqrt(imu_data.velocity.x**2 + imu_data.velocity.y**2),
            "angle": math.atan2(imu_data.orientation.y, imu_data.orientation.x),
            "position": (imu_data.position.x, imu_data.position.y)
        }


    def balance_adjustment(self):
        roll, pitch, yaw = self.imu.get_orientation()
        roll_rate, pitch_rate, yaw_rate = self.imu.get_angular_velocity()

        roll_correction = self.roll_controller.compute(0, roll)
        pitch_correction = self.pitch_controller.compute(0, pitch)

        self.set_target_position("left_hip_roll", self.joint_positions["left_hip_roll"] - roll_correction)
        self.set_target_position("right_hip_roll", self.joint_positions["right_hip_roll"] + roll_correction)
        self.set_target_position("left_ankle_roll", self.joint_positions["left_ankle_roll"] - roll_correction)
        self.set_target_position("right_ankle_roll", self.joint_positions["right_ankle_roll"] + roll_correction)
        self.set_target_position("left_ankle_pitch", self.joint_positions["left_ankle_pitch"] - pitch_correction)
        self.set_target_position("right_ankle_pitch", self.joint_positions["right_ankle_pitch"] - pitch_correction)

        if abs(pitch) > self.config.get_max_stable_pitch():
            hip_pitch_correction = self.pitch_controller.compute(0, pitch) * 0.5
            self.set_target_position("left_hip_pitch", self.joint_positions["left_hip_pitch"] + hip_pitch_correction)
            self.set_target_position("right_hip_pitch", self.joint_positions["right_hip_pitch"] + hip_pitch_correction)

        damping_factor = self.config.get_angular_velocity_damping_factor()
        self.set_target_position("left_hip_roll", self.joint_positions["left_hip_roll"] - roll_rate * damping_factor)
        self.set_target_position("right_hip_roll", self.joint_positions["right_hip_roll"] + roll_rate * damping_factor)
        self.set_target_position("left_ankle_roll", self.joint_positions["left_ankle_roll"] - roll_rate * damping_factor)
        self.set_target_position("right_ankle_roll", self.joint_positions["right_ankle_roll"] + roll_rate * damping_factor)
        self.set_target_position("left_ankle_pitch", self.joint_positions["left_ankle_pitch"] - pitch_rate * damping_factor)
        self.set_target_position("right_ankle_pitch", self.joint_positions["right_ankle_pitch"] - pitch_rate * damping_factor)


    def check_joint_status(self) -> bool:
        for joint, position in self.joint_positions.items():
            limits = self.config.get_joint_limits(joint)
            if position < limits[0] or position > limits[1]:
                print(f"Joint {joint} out of limits: {position}")
                return False
            
            if joint in self.last_joint_positions:
                velocity = (position - self.last_joint_positions[joint]) / self.control_loop_time
                if abs(velocity) > self.config.get_max_joint_velocity(joint):
                    print(f"Joint {joint} velocity too high: {velocity}")
                    return False
        
        self.last_joint_positions = self.joint_positions.copy()
        return True


    def check_power_status(self) -> bool:
        voltage = self.power_sensor.get_voltage()
        current = self.power_sensor.get_current()
        
        if voltage < self.config.get_min_voltage():
            print(f"Voltage too low: {voltage}V")
            return False
        if current > self.config.get_max_current():
            print(f"Current too high: {current}A")
            return False
        
        return True


    def check_sensor_status(self) -> bool:
        return self.imu.is_operational() and self.power_sensor.is_operational() and self.joint_sensors.is_operational()


    def self_diagnosis(self) -> Dict[str, bool]:
        return {
            "joints": self.check_joint_status(),
            "sensors": self.check_sensor_status(),
            "power": self.check_power_status(),
        }


    def emergency_stop(self):
        self.stop()
        self.reset_all_controllers()

        print("Emergency stop activated!")


    def stop(self):
        zero_control = {key: 0.0 for key in self.controllers.keys()}
        zero_control.update({"translation": 0.0, "rotation": 0.0})
        self.execute_control(zero_control)


    def run_control_loop(self):
        while True:
            try:
                loop_start_time = time.time()

                state = self.get_robot_state()
                self.update_robot_state(**state)
                self.balance_adjustment()

                diagnosis = self.self_diagnosis()
                if not all(diagnosis.values()):
                    print("System check failed. Initiating emergency stop.")
                    self.emergency_stop()
                    break

                control = self.compute_full_robot_control(self.current_speed, self.current_angle, self.current_position)
                self.execute_control(control)

                loop_end_time = time.time()
                loop_duration = loop_end_time - loop_start_time
                if loop_duration < self.control_loop_time:
                    time.sleep(self.control_loop_time - loop_duration)

            except Exception as e:
                print(f"Unexpected error: {e}")
                self.emergency_stop()
                break

        print("HumanoidRobotController shutting down.")
