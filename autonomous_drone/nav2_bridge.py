#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from px4_msgs.msg import TrajectorySetpoint, OffboardControlMode, VehicleCommand
import math
import time

class Nav2ToPx4Bridge(Node):
    def __init__(self):
        super().__init__('nav2_to_px4_bridge')
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.traj_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.offboard_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.vehicle_cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)

        # periodic publisher to keep offboard mode alive
        self.timer = self.create_timer(0.1, self._periodic)
        self.latest_twist = Twist()
        self.get_logger().info("nav2_to_px4_bridge started")

    def cmd_vel_cb(self, msg: Twist):
        self.latest_twist = msg
        # publish a TrajectorySetpoint with velocity set
        t = TrajectorySetpoint()
        # leave position NaN - velocity control mode
        t.position = [float('nan'), float('nan'), float('nan')]
        # set velocities from cmd_vel
        # NOTE: PX4 NED convention: usually vx positive is forward (NED x), vy right (NED y), vz down (NED z)
        # command conversion below assumes cmd_vel: linear.x forward, linear.y left->right, linear.z up
        # px4 expects velocity in m/s in vehicle frame mapped as vx, vy, vz
        vx = float(msg.linear.x)
        vy = float(msg.linear.y)
        vz = float(msg.linear.z)  # be mindful of sign if using NED semantics
        t.velocity = [vx, vy, vz]
        # yaw handling — convert angular.z into yaw rate if needed
        t.yaw = 0.0
        t.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.traj_pub.publish(t)

    def _periodic(self):
        # keep offboard_control_mode published (position=False, velocity=True)
        m = OffboardControlMode()
        m.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        m.position = False
        m.velocity = True
        m.attitude = False
        m.acceleration = False
        m.body_rate = False
        self.offboard_pub.publish(m)

    def arm_and_offboard(self):
        # one-shot: request offboard & arm (use with caution in sim)
        req = VehicleCommand()
        req.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        req.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        req.param1 = 1.0
        req.param2 = 6.0
        self.vehicle_cmd_pub.publish(req)
        self.get_logger().info("Requested OFFBOARD mode (cmd published)")
        time.sleep(0.15)
        arm = VehicleCommand()
        arm.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        arm.param1 = 1.0
        arm.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_cmd_pub.publish(arm)
        self.get_logger().info("Requested ARM command (cmd published)")

def main(args=None):
    rclpy.init(args=args)
    node = Nav2ToPx4Bridge()
    try:
        # Optionally send arm/offboard once; comment if you do handshake elsewhere.
        # node.arm_and_offboard()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down nav2 bridge")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
