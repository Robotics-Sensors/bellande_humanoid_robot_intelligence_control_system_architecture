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

def calculate_distance_to_object(camera_height, tilt, hip_pitch_offset, object_size):
    distance = camera_height * math.tan(math.pi * 0.5 + tilt - hip_pitch_offset - object_size)
    return abs(distance)

def calculate_footstep(current_x_move, target_distance, current_r_angle, target_angle, delta_time, config):
    next_movement = current_x_move
    target_distance = max(0, target_distance)
    fb_goal = min(target_distance * 0.1, config.MAX_FB_STEP)
    config.accum_period_time += delta_time
    if config.accum_period_time > (config.curr_period_time / 4):
        config.accum_period_time = 0.0
        if (target_distance * 0.1 / 2) < current_x_move:
            next_movement -= config.UNIT_FB_STEP
        else:
            next_movement += config.UNIT_FB_STEP
    fb_goal = min(next_movement, fb_goal)
    fb_move = max(fb_goal, config.MIN_FB_STEP)

    rl_goal = 0.0
    if abs(target_angle) * 180 / math.pi > 5.0:
        rl_offset = abs(target_angle) * 0.2
        rl_goal = min(rl_offset, config.MAX_RL_TURN)
        rl_goal = max(rl_goal, config.MIN_RL_TURN)
        rl_angle = min(abs(current_r_angle) + config.UNIT_RL_TURN, rl_goal)
        if target_angle < 0:
            rl_angle *= -1
    else:
        rl_angle = 0

    return fb_move, rl_angle

def calculate_delta_time(curr_time, prev_time):
    return (curr_time - prev_time).to_sec()
