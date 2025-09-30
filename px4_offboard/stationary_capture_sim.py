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
from px4_msgs.msg import VehicleOdometry
from px4_msgs.msg import ActuatorServos

from vicon_receiver.msg import Position
from math import nan

VETICAL_OFFSET = 0.013 # m

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
        # self.local_position_sub = self.create_subscription(VehicleLocalPosition,
        #     '/fmu/out/vehicle_local_position', self.local_position_callback, qos_profile)
        self.vehicle_odometry_sub = self.create_subscription(VehicleOdometry, 
            "/fmu/out/vehicle_odometry", self.vehicle_odom_callback, qos_profile)
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, 
            '/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, 
            '/fmu/in/trajectory_setpoint', qos_profile)
        self.actuator_servos = self.create_publisher(ActuatorServos,
            '/fmu/in/actuator_servos', qos_profile)
        timer_period = 0.45  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)
        
        # Intialize Variables
        self.timesync = 0
        self.position_setpoint = np.float32([1,1,-1])
        self.local_position = None
        self.is_target_captured = False
        
        self.get_logger().info("Initialized offboard control node.")
        
        
    # Callback function for local position subscriber.
    def local_position_callback(self, msg):
        # Calculate distance between drone and target vehicle
        self.local_position = np.float32([msg.x, msg.y, msg.z])
        if not (self.position_setpoint is None):
            dist = np.linalg.norm(self.local_position - self.position_setpoint)
        else:
            dist = np.inf
        
        if dist < 0.1:
            self.is_target_captured = True
            self.get_logger().info("Captured target.")
            
    def cmdloop_callback(self):
        # Publish offboard control modes
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = self.timesync
        offboard_msg.position=True
        offboard_msg.velocity=False
        offboard_msg.acceleration=False
        self.publisher_offboard_mode.publish(offboard_msg)
        
        # Publish servo commands
        servo_msg = ActuatorServos()
        servo_msg.timestamp = self.timesync
        servo_msg.control = [nan, nan, nan, nan, nan, nan, nan, nan] # Placeholder for servo commands
        

        # Publish trajectory of drone
        trajectory_msg = TrajectorySetpoint()
        if self.is_target_captured:
            trajectory_msg.position[0] = 0.0
            trajectory_msg.position[1] = 0.0
            trajectory_msg.position[2] = -0.1
            servo_msg.control[0] = 1.0 # Close servo 
        elif self.position_setpoint is not None:
            trajectory_msg.position[0] = float(self.position_setpoint[0])
            trajectory_msg.position[1] = float(self.position_setpoint[1])
            trajectory_msg.position[2] = float(self.position_setpoint[2])
            servo_msg.control[0] = -1.0 # Open servo
        else:
            trajectory_msg.position[0] = nan
            trajectory_msg.position[1] = nan
            trajectory_msg.position[2] = nan
            servo_msg.control[0] = nan # Disarm servo
        
        self.publisher_trajectory.publish(trajectory_msg)
        self.actuator_servos.publish(servo_msg)
        
    # Callback to keep timestamp for synchronization purposes
    def vehicle_odom_callback(self, msg):
        self.timesync = msg.timestamp
        self.local_position = np.float32([msg.position[0], msg.position[1], msg.position[2]])
        
        if not (self.position_setpoint is None):
            dist = np.linalg.norm(self.local_position - self.position_setpoint)
        else:
            dist = np.inf
        
        if dist < 0.1:
            self.is_target_captured = True
            self.get_logger().info("Captured target.")


def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
