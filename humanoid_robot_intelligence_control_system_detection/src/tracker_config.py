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

class TrackerConfig:
    def __init__(self):
        self.FOV_WIDTH = 35.2 * math.pi / 180
        self.FOV_HEIGHT = 21.6 * math.pi / 180
        self.NOT_FOUND_THRESHOLD = 50
        self.WAITING_THRESHOLD = 5
        self.use_head_scan = True
        self.DEBUG_PRINT = False
        self.p_gain = 0.4
        self.i_gain = 0.0
        self.d_gain = 0.0

    def update_from_params(self, get_param):
        self.FOV_WIDTH = get_param('fov_width', self.FOV_WIDTH)
        self.FOV_HEIGHT = get_param('fov_height', self.FOV_HEIGHT)
        self.NOT_FOUND_THRESHOLD = get_param('not_found_threshold', self.NOT_FOUND_THRESHOLD)
        self.WAITING_THRESHOLD = get_param('waiting_threshold', self.WAITING_THRESHOLD)
        self.use_head_scan = get_param('use_head_scan', self.use_head_scan)
        self.DEBUG_PRINT = get_param('debug_print', self.DEBUG_PRINT)
        self.p_gain = get_param('p_gain', self.p_gain)
        self.i_gain = get_param('i_gain', self.i_gain)
        self.d_gain = get_param('d_gain', self.d_gain)

    def reset(self):
        """Reset all values to their initial state."""
        self.__init__()

    def set_fov(self, width, height):
        """Set the field of view."""
        self.FOV_WIDTH = width
        self.FOV_HEIGHT = height

    def set_thresholds(self, not_found_threshold, waiting_threshold):
        """Set the thresholds."""
        self.NOT_FOUND_THRESHOLD = not_found_threshold
        self.WAITING_THRESHOLD = waiting_threshold

    def set_use_head_scan(self, use_head_scan):
        """Set whether to use head scan."""
        self.use_head_scan = use_head_scan

    def set_debug_print(self, debug_print):
        """Set the debug print flag."""
        self.DEBUG_PRINT = debug_print

    def set_pid_gains(self, p_gain, i_gain, d_gain):
        """Set the PID gains."""
        self.p_gain = p_gain
        self.i_gain = i_gain
        self.d_gain = d_gain

    def get_pid_gains(self):
        """Get the PID gains."""
        return self.p_gain, self.i_gain, self.d_gain

    def adjust_pid_gains(self, p_adjust=0, i_adjust=0, d_adjust=0):
        """Adjust the PID gains by the given amounts."""
        self.p_gain += p_adjust
        self.i_gain += i_adjust
        self.d_gain += d_adjust

    def get_config_dict(self):
        """Return a dictionary of all configuration parameters."""
        return {
            'FOV_WIDTH': self.FOV_WIDTH,
            'FOV_HEIGHT': self.FOV_HEIGHT,
            'NOT_FOUND_THRESHOLD': self.NOT_FOUND_THRESHOLD,
            'WAITING_THRESHOLD': self.WAITING_THRESHOLD,
            'use_head_scan': self.use_head_scan,
            'DEBUG_PRINT': self.DEBUG_PRINT,
            'p_gain': self.p_gain,
            'i_gain': self.i_gain,
            'd_gain': self.d_gain
        }

    def load_config_dict(self, config_dict):
        """Load configuration from a dictionary."""
        for key, value in config_dict.items():
            if hasattr(self, key):
                setattr(self, key, value)

    def validate_config(self):
        """Validate the configuration parameters."""
        assert 0 < self.FOV_WIDTH < math.pi, "FOV_WIDTH must be between 0 and pi"
        assert 0 < self.FOV_HEIGHT < math.pi, "FOV_HEIGHT must be between 0 and pi"
        assert self.NOT_FOUND_THRESHOLD > 0, "NOT_FOUND_THRESHOLD must be positive"
        assert self.WAITING_THRESHOLD > 0, "WAITING_THRESHOLD must be positive"
        assert isinstance(self.use_head_scan, bool), "use_head_scan must be a boolean"
        assert isinstance(self.DEBUG_PRINT, bool), "DEBUG_PRINT must be a boolean"
        assert self.p_gain >= 0, "p_gain must be non-negative"
        assert self.i_gain >= 0, "i_gain must be non-negative"
        assert self.d_gain >= 0, "d_gain must be non-negative"

    def print_config(self):
        """Print the current configuration."""
        for key, value in self.get_config_dict().items():
            print(f"{key}: {value}")

    def to_ros_param(self):
        """Convert the configuration to a format suitable for ROS parameters."""
        return {
            'tracker': {
                'fov': {
                    'width': self.FOV_WIDTH,
                    'height': self.FOV_HEIGHT
                },
                'thresholds': {
                    'not_found': self.NOT_FOUND_THRESHOLD,
                    'waiting': self.WAITING_THRESHOLD
                },
                'use_head_scan': self.use_head_scan,
                'debug_print': self.DEBUG_PRINT,
                'pid': {
                    'p': self.p_gain,
                    'i': self.i_gain,
                    'd': self.d_gain
                }
            }
        }



class TrackerInitializeConfig:
    def __init__(self):
        self.count_not_found = 0
        self.on_tracking = False
        self.current_object_pan = 0.0
        self.current_object_tilt = 0.0
        self.x_error_sum = 0.0
        self.y_error_sum = 0.0
        self.current_object_bottom = 0.0
        self.tracking_status = "NotFound"
        self.object_position = None


    def update_from_params(self, get_param):
        self.count_not_found = get_param('initial_count_not_found', self.count_not_found)
        self.on_tracking = get_param('initial_on_tracking', self.on_tracking)
        self.current_object_pan = get_param('initial_object_pan', self.current_object_pan)
        self.current_object_tilt = get_param('initial_object_tilt', self.current_object_tilt)
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
        self.current_object_pan = pan
        self.current_object_tilt = tilt
        self.current_object_bottom = bottom


    def is_tracking(self):
        """Check if tracking is currently active."""
        return self.on_tracking


    def start_tracking(self):
        """Start the tracking process."""
        self.on_tracking = True


    def stop_tracking(self):
        """Stop the tracking process."""
        self.on_tracking = False
