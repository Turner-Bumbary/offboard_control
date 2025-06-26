#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2022 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

__author__ = "Jaeyoung Lim"
__contact__ = "jalim@ethz.ch"

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleOdometry

from vicon_receiver.msg import Position
from math import nan

class OffboardControl(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create subscribers and publishers
        self.mocap_sub = self.create_subscription(Position, 
            '/vicon/Turner_small_drone/Turner_small_drone', self.mocap_callback, 10)
        self.vehicle_odometry_sub = self.create_subscription(VehicleOdometry, 
            "/fmu/out/vehicle_odometry", self.vehicle_odom_callback, qos_profile)
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, 
            '/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, 
            '/fmu/in/trajectory_setpoint', qos_profile)
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)
        
        # Intialize Variables
        self.timesync = 0
        self.position_setpoint = None
        
        self.get_logger().info("Initialized offboard control node.")
        
    # Vicon motion capture callback function. Stores position setpoint of vehicle.
    def mocap_callback(self, msg):
        # Get position data from Vicon message (ENU coordinates)
        x_pos = msg.x_trans/1000.0
        y_pos = msg.y_trans/1000.0
        z_pos = msg.z_trans/1000.0
        
        # Convert and store ENU coordinates as FRD coordinates for PX4
        enu_coordinates = np.float32([x_pos, y_pos, z_pos])
        self.position_setpoint = np.float32([enu_coordinates[1], enu_coordinates[0], -enu_coordinates[2]])
        
    def cmdloop_callback(self):
        # Publish offboard control modes
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = self.timesync
        offboard_msg.position=True
        offboard_msg.velocity=False
        offboard_msg.acceleration=False
        self.publisher_offboard_mode.publish(offboard_msg)

        # Publish trajectory of drone
        trajectory_msg = TrajectorySetpoint()
        if self.position_setpoint is not None:
            trajectory_msg.position[0] = float(self.position_setpoint[0])
            trajectory_msg.position[1] = float(self.position_setpoint[1])
            trajectory_msg.position[2] = float(self.position_setpoint[2])
        else:
            trajectory_msg.position[0] = nan
            trajectory_msg.position[1] = nan
            trajectory_msg.position[2] = nan
        self.publisher_trajectory.publish(trajectory_msg)
        
    # Callback to keep timestamp for synchronization purposes
    def vehicle_odom_callback(self, msg):
        self.timesync = msg.timestamp


def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
