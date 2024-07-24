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

def calculate_error(object_position, FOV_WIDTH, FOV_HEIGHT):
    x_error = -math.atan(object_position.x * math.tan(FOV_WIDTH))
    y_error = -math.atan(object_position.y * math.tan(FOV_HEIGHT))
    return x_error, y_error

def calculate_error_target(error, error_diff, error_sum, p_gain, i_gain, d_gain):
    return error * p_gain + error_diff * d_gain + error_sum * i_gain

def calculate_delta_time(curr_time, prev_time):
    return (curr_time - prev_time).to_sec()
