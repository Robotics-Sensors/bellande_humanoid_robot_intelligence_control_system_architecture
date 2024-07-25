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

import math
from typing import Tuple

from humanoid_robot_intelligence_control_system_detection.PIDController import PIDController
from humanoid_robot_intelligence_control_system_detection.tracker_config import TrackerConfig


class MovementController:
    def __init__(self, config: TrackerConfig, controller_type: str = "PID"):
        self.config = config
        self.controller_type = controller_type
        self.translation_controller = self._create_controller("translation")
        self.rotation_controller = self._create_controller("rotation")
        self.path_controller = self._create_controller("path")

    def _create_controller(self, controller_name: str) -> Any:
        if self.controller_type == "PID":
            return PIDController(
                getattr(self.config, f"get_{controller_name}_pid_gains")(),
                f"{controller_name}_controller",
                output_limits=(-1, 1)
            )
        elif self.controller_type == "BellandeController":
            return BellandeController(
                getattr(self.config, f"get_{controller_name}_bellande_params")(),
                f"{controller_name}_controller"
            )
        else:
            raise ValueError(f"Unsupported controller type: {self.controller_type}")

    def update_config(self, new_config: TrackerConfig):
        self.config = new_config
        self._update_controller(self.translation_controller, "translation")
        self._update_controller(self.rotation_controller, "rotation")
        self._update_controller(self.path_controller, "path")

    def _update_controller(self, controller: Any, controller_name: str):
        if self.controller_type == "PID":
            controller.update_config(getattr(self.config, f"get_{controller_name}_pid_gains")())
        elif self.controller_type == "BellandeController":
            controller.update_config(getattr(self.config, f"get_{controller_name}_bellande_params")())

    def compute_translation_control(self, current_position: float, target_position: float) -> float:
        return self.translation_controller.compute(target_position, current_position)

    def compute_rotation_control(self, current_angle: float, target_angle: float) -> float:
        return self.rotation_controller.compute(target_angle, current_angle)

    def compute_path_control(self, current_position: Tuple[float, float], target_position: Tuple[float, float]) -> Tuple[float, float]:
        dx = target_position[0] - current_position[0]
        dy = target_position[1] - current_position[1]
        distance = (dx**2 + dy**2)**0.5
        angle = math.atan2(dy, dx)
        translation = self.translation_controller.compute(0, -distance)
        rotation = self.rotation_controller.compute(angle, 0)
        return translation, rotation
