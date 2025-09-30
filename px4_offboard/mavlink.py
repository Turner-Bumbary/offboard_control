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
from px4_msgs.msg import VehicleOdometry
from px4_msgs.msg import ActuatorMotors
from px4_msgs.msg import ActuatorServos

from vicon_receiver.msg import Position
from math import nan

import asyncio
from mavsdk import System

VETICAL_OFFSET = 0.013 # m

class MavlinkControl(Node):

    def __init__(self):
        super().__init__('mavlink_node')

        # Connect to drone via UDP
        loop = asyncio.get_event_loop()
        self.get_logger().info("Connecting to drone...")
        loop.run_until_complete(self.connect_to_drone()) # Block thread until connection is established

        # Create timer
        self.actuator_timer = self.create_timer(1.0, self.cmdloop_callback)
                
        # Intialize Variables
        self.servo_open = False
        
    async def connect_to_drone(self):
        self.drone = System()
        await self.drone.connect(system_address="udp://192.168.0.1:14540")
        self.get_logger().info("Connected to drone")
            
    def cmdloop_callback(self):
        pass
        # # Publish servo commands
        # servo_msg = ActuatorServos()
        # servo_msg.timestamp = self.timesync
        # servo_msg.control = [1.0, 0., 0., 0., 0., 0., 0., 0.]

        # # Publish motor commands
        # motor_msg = ActuatorMotors()
        # motor_msg.timestamp = self.timesync
        # motor_msg.control = [0.5, 0.5, 0.5, 0.5, 0., 0., 0., 0., 0., 0., 0., 0.]
        
        # if self.servo_open == True:
        #     # servo_msg.control[0] = -1.0
        #     self.servo_open = False
        #     self.get_logger().info('Closed Basket.')
        # else:
        #     # servo_msg.control[0] = 1.0
        #     self.servo_open = True
        #     self.get_logger().info('Opened Basket.')
        
        # self.actuator_servos.publish(servo_msg)
        # self.actuator_motors.publish(motor_msg)

def main(args=None):
    rclpy.init(args=args)

    offboard_control = MavlinkControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
