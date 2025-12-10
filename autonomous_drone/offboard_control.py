#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
import numpy as np

class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control')
        
        # Configure QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', 
            self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', 
            self.vehicle_status_callback, qos_profile)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -5.0
        
        # Square pattern variables
        self.path_timer_count = 0
        self.square_side_length = 5.0
        # Reduced ticks per side to 50 (5 seconds) for faster movement (1m/s)
        self.ticks_per_side = 50 
        
        # Create timer (0.1 seconds period)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def vehicle_local_position_callback(self, vehicle_local_position):
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        self.vehicle_status = vehicle_status

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 
            param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def publish_offboard_control_heartbeat_signal(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 0.0  # [-PI:PI]
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, **params):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self):
        self.publish_offboard_control_heartbeat_signal()

        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()

        # Check if airborne with a small buffer
        if self.vehicle_local_position.z > (self.takeoff_height + 0.5) and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)
            
        elif self.vehicle_local_position.z <= (self.takeoff_height + 0.5):
            self.path_timer_count += 1
            
            # Determine which side of the square we are on (0-3)
            phase = (self.path_timer_count // self.ticks_per_side) % 4
            
            # Calculate progress along the current side (0.0 to 1.0)
            # This creates the "sliding" setpoint effect
            prog = (self.path_timer_count % self.ticks_per_side) / self.ticks_per_side
            
            side_len = self.square_side_length
            x_target = 0.0
            y_target = 0.0

            if phase == 0: 
                # Leg 1: Move North (X increases)
                x_target = prog * side_len
                y_target = 0.0
            elif phase == 1: 
                # Leg 2: Move East (Y increases, X stays at max)
                x_target = side_len
                y_target = prog * side_len
            elif phase == 2: 
                # Leg 3: Move South (X decreases, Y stays at max)
                x_target = side_len - (prog * side_len)
                y_target = side_len
            elif phase == 3: 
                # Leg 4: Move West (Y decreases, X stays at 0)
                x_target = 0.0
                y_target = side_len - (prog * side_len)
                
            self.publish_position_setpoint(x_target, y_target, self.takeoff_height)

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

def main(args=None):
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()