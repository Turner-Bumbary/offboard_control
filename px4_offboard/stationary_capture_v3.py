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
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleOdometry
from px4_msgs.msg import ActuatorServos
from std_msgs.msg import Bool

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
        
        # Motion capture subscribers
        self.mocap_sub = self.create_subscription(Position, 
            '/vicon/Turner_small_drone_5/Turner_small_drone_5', self.target_vehicle_callback, 10)
        self.mocap_sub = self.create_subscription(Position, 
            '/vicon/Turner_x500/Turner_x500', self.x500_callback, 10)
        
        # Vehicle position/clock subscriber
        self.vehicle_attitude_sub = self.create_subscription(VehicleOdometry, 
            "/fmu/out/vehicle_attitude", self.vehicle_attitude_callback, qos_profile) 
        
        # External vision publisher
        self.vehicle_odometry_pub = self.create_publisher(VehicleOdometry, 
            '/fmu/in/vehicle_visual_odometry', qos_profile)
        
        # Offboard control publishers
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, 
            '/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, 
            '/fmu/in/trajectory_setpoint', qos_profile)
        self.actuate_servo_pub = self.create_publisher(Bool, 
            '/actuate_servo', 10)
        
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)
        
        # Intialize Variables
        self.timesync = 0
        self.target_vehicle_position = None
        self.x500_position = None
        self.quaternion = None
        self.is_target_captured = False
        
        self.get_logger().info("Initialized stationary_capture_v3.")
        
    # Vicon motion capture callback function. Stores position setpoint of target vehicle.
    def target_vehicle_callback(self, msg):
        # Get position data from Vicon message (ENU coordinates)
        x_pos = msg.x_trans/1000.0
        y_pos = msg.y_trans/1000.0
        z_pos = msg.z_trans/1000.0
        
        # Convert and store ENU coordinates as FRD coordinates for PX4
        enu_coordinates = np.float32([x_pos, y_pos, z_pos])
        frd_coordinates = np.float32([enu_coordinates[1], enu_coordinates[0], -enu_coordinates[2]])
        
        if np.any(frd_coordinates): # If vicon coordinates are non-zero update target vehicle position
            self.target_vehicle_position = frd_coordinates
        
    # Callback function for local position subscriber.
    def x500_callback(self, msg):
        # Get position data from Vicon message (ENU coordinates)
        x_pos = msg.x_trans/1000.0
        y_pos = msg.y_trans/1000.0
        z_pos = msg.z_trans/1000.0
        
        # Convert and store ENU coordinates as FRD coordinates for PX
        enu_coordinates = np.float32([x_pos, y_pos, z_pos])
        frd_coordinates = np.float32([enu_coordinates[1], enu_coordinates[0], -enu_coordinates[2]])
        
        # Publish Vicon position as external vision odometry message to PX4
        if np.any(frd_coordinates): # If vicon coordinates are non-zero publish coordiantes
            # Create PX4 message from Vicon position data
            msg_px4 = VehicleOdometry() # Message to be sent to PX4
            msg_px4.timestamp = self.timesync # Set timestamp
            msg_px4.timestamp_sample = self.timesync # Timestamp for mocap sample
            msg_px4.pose_frame = 2 # FRD from px4 message
            msg_px4.position = frd_coordinates.tolist() # Convert numpy array to list
            msg_px4.position_variance = [10**(-6), 10**(-6), 10**(-6)] # Assuming 1mmm standard deviation in world error
            self.vehicle_odometry_pub.publish(msg_px4)
            self.x500_position = frd_coordinates
        

        if not self.is_target_captured: # Check if target vehicle is within capture range
            # if (self.target_vehicle_position is not None) and (self.quaternion is not None): # Project the target vehicle's position into the body frame
            #     delta = self.target_vehicle_position - self.x500_position
            #     NED_position = np.array[(0, delta[1], delta[0], -delta[2])]
                
            #     # Use quanternion to rotate target_NED_position into body frame
            #     q = np.array([self.quaternion[0], self.quaternion[1], self.quaternion[2], self.quaternion[3]])
            #     q_inv = np.array[(self.quaternion[0], -self.quaternion[1], -self.quaternion[2], -self.quaternion[3])]

            #     # Calculate the position in the body frame
            #     BODY_position = OffboardControl.quaternion_multiply(q_inv, NED_position)
            #     BODY_position = OffboardControl.quaternion_multiply(BODY_position, q)
                
            #     # Extract the x,y,z coordinates from the quaternion
            #     x = abs(BODY_position[1])
            #     y = abs(BODY_position[2])
            #     z = abs(BODY_position[3])
                
            #     if x <= 0.265 and y<= 0.265 and (z >= 0.04 and z <= 0.20): # Check if projected coordinaes are within capture range
            #         if x <= 0.075 or y <= 0.075:
            #             self.is_target_captured = True
            if np.any(self.x500_position) and np.any(self.target_vehicle_position): # Determine whether the target vehicle is within capture range
                delta = self.target_vehicle_position - self.x500_position
                if np.linalg.norm(delta[0:2]) < 0.145 and (delta[2] >= -0.20 and delta[2] <= -0.04):
                    self.is_target_captured = True
                    
            # If vehicle is now captured, publish servo message
            if self.is_target_captured:
                actuate_servo_msg = Bool()
                actuate_servo_msg.data = True            
                self.actuate_servo_pub.publish(actuate_servo_msg)
                self.get_logger().info("Captured target.")
        else:
            # Check whether target vehicle is still within the cage
            if np.any(self.x500_position) and np.any(self.target_vehicle_position) and np.linalg.norm(self.target_vehicle_position - self.x500_position) >= 0.35:
                self.is_target_captured = False
                self.get_logger().info("Lost target.")
                
            # If vehicle is now lost, publish servo message
            if not self.is_target_captured:
                actuate_servo_msg = Bool()
                actuate_servo_msg.data = False
                self.actuate_servo_pub.publish(actuate_servo_msg)
                self.get_logger().info("Opened basket.")
            
    def cmdloop_callback(self):
        # Publish offboard control modes
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = self.timesync
        offboard_msg.position=True
        offboard_msg.velocity=False
        offboard_msg.acceleration=False
        self.publisher_offboard_mode.publish(offboard_msg)
        
        # # Publish servo actuate command
        # if self.is_target_captured:
        #     actuate_servo_msg = Bool()
        #     actuate_servo_msg.data = True            
        #     self.actuate_servo_pub.publish(actuate_servo_msg)

        # Publish trajectory of drone
        trajectory_msg = TrajectorySetpoint()
        if self.is_target_captured:
            trajectory_msg.position[0] = 0.0
            trajectory_msg.position[1] = 0.0
            trajectory_msg.position[2] = -1.0
            # servo_msg.control[0] = 1.0 # Close servo 
        elif (self.target_vehicle_position is not None) and (self.x500_position is not None):
            vertical_offset = 0.2 # m     
                   
            # Check if drone is alignhed with x,y coordinates of target vehicle
            if np.linalg.norm(self.x500_position[0:2] - self.target_vehicle_position[0:2]) < 0.145:
                # Set trajectory to target vehicle position
                trajectory_msg.position[0] = float(self.target_vehicle_position[0])
                trajectory_msg.position[1] = float(self.target_vehicle_position[1])
                trajectory_msg.position[2] = float(self.target_vehicle_position[2])
            else:
                # Add vertical offset to hover below z-position of target vehicle
                trajectory_msg.position[0] = float(self.target_vehicle_position[0])
                trajectory_msg.position[1] = float(self.target_vehicle_position[1])
                trajectory_msg.position[2] = float(self.target_vehicle_position[2]) + vertical_offset
        else:
            trajectory_msg.position[0] = nan
            trajectory_msg.position[1] = nan
            trajectory_msg.position[2] = nan
            # servo_msg.control[0] = nan # Disarm servo
            
        self.publisher_trajectory.publish(trajectory_msg)
        
    def quaternion_multiply(q1, q2):
        s1, v1 = q1[0], q1[1:]
        s2, v2 = q2[0], q2[1:]
        
        s_result = s1 * s2 - np.dot(v1, v2)
        v_result = s1 * v2 + s2 * v1 + np.cross(v1, v2)
        return np.concatenate(([s_result], v_result))
        
    # Callback to keep timestamp for synchronization purposes
    def vehicle_attitude_callback(self, msg):
        self.timesync = msg.timestamp
        self.quaternion = np.float32(msg.q) / np.linalg.norm(np.float32(msg.q))

def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
