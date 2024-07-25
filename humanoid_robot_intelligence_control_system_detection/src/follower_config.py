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


class FollowerConfig:
    def __init__(self):
        self.FOV_WIDTH = 35.2 * math.pi / 180
        self.FOV_HEIGHT = 21.6 * math.pi / 180
        self.CAMERA_HEIGHT = 0.46
        self.NOT_FOUND_THRESHOLD = 50
        self.MAX_FB_STEP = 40.0 * 0.001
        self.MAX_RL_TURN = 15.0 * math.pi / 180
        self.IN_PLACE_FB_STEP = -3.0 * 0.001
        self.MIN_FB_STEP = 5.0 * 0.001
        self.MIN_RL_TURN = 5.0 * math.pi / 180
        self.UNIT_FB_STEP = 1.0 * 0.001
        self.UNIT_RL_TURN = 0.5 * math.pi / 180
        self.SPOT_FB_OFFSET = 0.0 * 0.001
        self.SPOT_RL_OFFSET = 0.0 * 0.001
        self.SPOT_ANGLE_OFFSET = 0.0
        self.hip_pitch_offset = 7.0
        self.curr_period_time = 0.6
        self.accum_period_time = 0.0
        self.DEBUG_PRINT = False

    def update_from_params(self, get_param):
        self.FOV_WIDTH = get_param('fov_width', self.FOV_WIDTH)
        self.FOV_HEIGHT = get_param('fov_height', self.FOV_HEIGHT)
        self.CAMERA_HEIGHT = get_param('camera_height', self.CAMERA_HEIGHT)
        self.NOT_FOUND_THRESHOLD = get_param('not_found_threshold', self.NOT_FOUND_THRESHOLD)
        self.MAX_FB_STEP = get_param('max_fb_step', self.MAX_FB_STEP)
        self.MAX_RL_TURN = get_param('max_rl_turn', self.MAX_RL_TURN)
        self.IN_PLACE_FB_STEP = get_param('in_place_fb_step', self.IN_PLACE_FB_STEP)
        self.MIN_FB_STEP = get_param('min_fb_step', self.MIN_FB_STEP)
        self.MIN_RL_TURN = get_param('min_rl_turn', self.MIN_RL_TURN)
        self.UNIT_FB_STEP = get_param('unit_fb_step', self.UNIT_FB_STEP)
        self.UNIT_RL_TURN = get_param('unit_rl_turn', self.UNIT_RL_TURN)
        self.SPOT_FB_OFFSET = get_param('spot_fb_offset', self.SPOT_FB_OFFSET)
        self.SPOT_RL_OFFSET = get_param('spot_rl_offset', self.SPOT_RL_OFFSET)
        self.SPOT_ANGLE_OFFSET = get_param('spot_angle_offset', self.SPOT_ANGLE_OFFSET)
        self.hip_pitch_offset = get_param('hip_pitch_offset', self.hip_pitch_offset)
        self.curr_period_time = get_param('curr_period_time', self.curr_period_time)
        self.DEBUG_PRINT = get_param('debug_print', self.DEBUG_PRINT)

    def reset(self):
        """Reset all values to their initial state."""
        self.__init__()

    def set_fov(self, width, height):
        """Set the field of view."""
        self.FOV_WIDTH = width
        self.FOV_HEIGHT = height

    def set_camera_height(self, height):
        """Set the camera height."""
        self.CAMERA_HEIGHT = height

    def set_thresholds(self, not_found_threshold):
        """Set the thresholds."""
        self.NOT_FOUND_THRESHOLD = not_found_threshold

    def set_step_parameters(self, max_fb, max_rl, in_place_fb, min_fb, min_rl, unit_fb, unit_rl):
        """Set the step parameters."""
        self.MAX_FB_STEP = max_fb
        self.MAX_RL_TURN = max_rl
        self.IN_PLACE_FB_STEP = in_place_fb
        self.MIN_FB_STEP = min_fb
        self.MIN_RL_TURN = min_rl
        self.UNIT_FB_STEP = unit_fb
        self.UNIT_RL_TURN = unit_rl

    def set_spot_offsets(self, fb_offset, rl_offset, angle_offset):
        """Set the spot offsets."""
        self.SPOT_FB_OFFSET = fb_offset
        self.SPOT_RL_OFFSET = rl_offset
        self.SPOT_ANGLE_OFFSET = angle_offset

    def set_hip_pitch_offset(self, offset):
        """Set the hip pitch offset."""
        self.hip_pitch_offset = offset

    def set_period_time(self, period_time):
        """Set the current period time."""
        self.curr_period_time = period_time

    def update_accum_period_time(self, delta_time):
        """Update the accumulated period time."""
        self.accum_period_time += delta_time
        if self.accum_period_time > (self.curr_period_time / 4):
            self.accum_period_time = 0.0
        return self.accum_period_time

    def reset_accum_period_time(self):
        """Reset the accumulated period time."""
        self.accum_period_time = 0.0

    def set_debug_print(self, debug_print):
        """Set the debug print flag."""
        self.DEBUG_PRINT = debug_print

    def get_config_dict(self):
        """Return a dictionary of all configuration parameters."""
        return {
            'FOV_WIDTH': self.FOV_WIDTH,
            'FOV_HEIGHT': self.FOV_HEIGHT,
            'CAMERA_HEIGHT': self.CAMERA_HEIGHT,
            'NOT_FOUND_THRESHOLD': self.NOT_FOUND_THRESHOLD,
            'MAX_FB_STEP': self.MAX_FB_STEP,
            'MAX_RL_TURN': self.MAX_RL_TURN,
            'IN_PLACE_FB_STEP': self.IN_PLACE_FB_STEP,
            'MIN_FB_STEP': self.MIN_FB_STEP,
            'MIN_RL_TURN': self.MIN_RL_TURN,
            'UNIT_FB_STEP': self.UNIT_FB_STEP,
            'UNIT_RL_TURN': self.UNIT_RL_TURN,
            'SPOT_FB_OFFSET': self.SPOT_FB_OFFSET,
            'SPOT_RL_OFFSET': self.SPOT_RL_OFFSET,
            'SPOT_ANGLE_OFFSET': self.SPOT_ANGLE_OFFSET,
            'hip_pitch_offset': self.hip_pitch_offset,
            'curr_period_time': self.curr_period_time,
            'accum_period_time': self.accum_period_time,
            'DEBUG_PRINT': self.DEBUG_PRINT
        }

    def load_config_dict(self, config_dict):
        """Load configuration from a dictionary."""
        for key, value in config_dict.items():
            if hasattr(self, key):
                setattr(self, key, value)



class FollowerInitializeConfig:
    def __init__(self):
        self.count_not_found = 0
        self.count_to_approach = 0
        self.on_tracking = False
        self.approach_object_position = "NotFound"
        self.current_pan = -10
        self.current_tilt = -10
        self.current_x_move = 0.005
        self.current_r_angle = 0
        self.x_error_sum = 0.0
        self.y_error_sum = 0.0
        self.current_object_bottom = 0.0
        self.tracking_status = "NotFound"
        self.object_position = None


    def update_from_params(self, get_param):
        self.count_not_found = get_param('initial_count_not_found', self.count_not_found)
        self.count_to_approach = get_param('initial_count_to_approach', self.count_to_approach)
        self.on_tracking = get_param('initial_on_tracking', self.on_tracking)
        self.approach_object_position = get_param('initial_approach_object_position', self.approach_object_position)
        self.current_pan = get_param('initial_current_pan', self.current_pan)
        self.current_tilt = get_param('initial_current_tilt', self.current_tilt)
        self.current_x_move = get_param('initial_current_x_move', self.current_x_move)
        self.current_r_angle = get_param('initial_current_r_angle', self.current_r_angle)
        self.x_error_sum = get_param('initial_x_error_sum', self.x_error_sum)
        self.y_error_sum = get_param('initial_y_error_sum', self.y_error_sum)
        self.current_object_bottom = get_param('initial_object_bottom', self.current_object_bottom)
        self.tracking_status = get_param('initial_tracking_status', self.tracking_status)


    def reset(self):
        """Reset all values to their initial state."""
        self.__init__()

    def update_object_position(self, x, y, z):
        """Update the object position."""
        self.object_position.x = x
        self.object_position.y = y
        self.object_position.z = z

    def increment_count_not_found(self):
        """Increment the count of frames where object was not found."""
        self.count_not_found += 1

    def reset_count_not_found(self):
        """Reset the count of frames where object was not found."""
        self.count_not_found = 0

    def increment_count_to_approach(self):
        """Increment the count to approach."""
        self.count_to_approach += 1

    def reset_count_to_approach(self):
        """Reset the count to approach."""
        self.count_to_approach = 0

    def set_tracking_status(self, status):
        """Set the current tracking status."""
        self.tracking_status = status

    def update_error_sums(self, x_error, y_error):
        """Update the cumulative error sums."""
        self.x_error_sum += x_error
        self.y_error_sum += y_error

    def reset_error_sums(self):
        """Reset the cumulative error sums."""
        self.x_error_sum = 0.0
        self.y_error_sum = 0.0

    def update_current_object_position(self, pan, tilt, bottom):
        """Update the current object position."""
        self.current_pan = pan
        self.current_tilt = tilt
        self.current_object_bottom = bottom

    def update_movement(self, x_move, r_angle):
        """Update the current movement parameters."""
        self.current_x_move = x_move
        self.current_r_angle = r_angle

    def is_tracking(self):
        """Check if tracking is currently active."""
        return self.on_tracking

    def start_tracking(self):
        """Start the tracking process."""
        self.on_tracking = True

    def stop_tracking(self):
        """Stop the tracking process."""
        self.on_tracking = False

    def set_approach_position(self, position):
        """Set the approach position of the object."""
        self.approach_object_position = position
