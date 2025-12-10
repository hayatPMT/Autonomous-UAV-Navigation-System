#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import math

from px4_msgs.msg import VehicleLocalPosition
from sensor_msgs.msg import Image, LaserScan, PointCloud2
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from cv_bridge import CvBridge

# Efficient PointCloud2 generation
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header

def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5); sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5); sr = math.sin(roll * 0.5)
    return (sr*cp*cy - cr*sp*sy, cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy, cr*cp*cy + sr*sp*sy)

class BridgeNode(Node):
    def __init__(self):
        super().__init__("drone_bridge")
        
        # 1. Coordinate & TF Setup
        self.x = 0.0; self.y = 0.0; self.z = 0.0; self.yaw = 0.0
        self.has_pose = False
        self.tf_bc = TransformBroadcaster(self)
        self.bridge = CvBridge()
        
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
        
        # Subscribers
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position", self.pos_cb, qos)
        self.create_subscription(LaserScan, "/scan", self.scan_cb, qos)
        self.create_subscription(Image, "/depth_camera", self.depth_cb, qos)
        
        # Publisher: Output 3D points for OctoMap
        self.pcl_pub = self.create_publisher(PointCloud2, "/drone/points", 10)
        
        self.get_logger().info("🚀 JAZZY BRIDGE STARTED: TF + PointCloud Fusion Active")

    def pos_cb(self, msg):
        if not msg.xy_valid: return
        
        # NED -> ENU Conversion
        self.x = msg.y
        self.y = msg.x
        self.z = -msg.z
        
        if hasattr(msg, 'heading'):
            self.yaw = math.pi/2 - msg.heading
            
        self.has_pose = True
        
        # Broadcast Transform (Map -> Base_Link)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = self.z
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = \
            quaternion_from_euler(0, 0, self.yaw)
        self.tf_bc.sendTransform(t)

    def scan_cb(self, msg):
        """Convert 2D Lidar to 3D PointCloud"""
        if not self.has_pose: return
        
        # Create points in the drone's local frame ("base_link")
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        ranges = np.array(msg.ranges)
        
        # Filter invalid ranges
        valid = (ranges > msg.range_min) & (ranges < msg.range_max) & np.isfinite(ranges)
        if not np.any(valid): return
        
        r_valid = ranges[valid]
        a_valid = angles[valid]
        
        # Polar to Cartesian (Z is 0 because laser is flat)
        x = r_valid * np.cos(a_valid)
        y = r_valid * np.sin(a_valid)
        z = np.zeros_like(x) 
        
        points = np.column_stack((x, y, z))
        
        header = Header()
        header.frame_id = "base_link" 
        header.stamp = self.get_clock().now().to_msg()
        # Create PointCloud2 message
        pc_msg = pc2.create_cloud_xyz32(header, points)
        self.pcl_pub.publish(pc_msg)

    def depth_cb(self, msg):
        """Convert Depth Image to 3D PointCloud"""
        if not self.has_pose: return
        try:
            depth_img = self.bridge.imgmsg_to_cv2(msg, "32FC1")
            
            # Downsample (every 6th pixel) to reduce CPU load
            step = 6
            d_small = depth_img[::step, ::step]
            H, W = d_small.shape
            
            # Pinhole Camera Model (Simple approximation)
            fov_h = math.radians(60.0) 
            f = (W / 2) / math.tan(fov_h / 2)
            cx, cy = W / 2, H / 2
            
            # Generate pixel grid
            u, v = np.meshgrid(np.arange(W), np.arange(H))
            
            # Filter valid depth
            valid = (d_small > 0.2) & (d_small < 15.0) & np.isfinite(d_small)
            if not np.any(valid): return

            z_c = d_small[valid]
            x_c = (u[valid] - cx) * z_c / f
            y_c = (v[valid] - cy) * z_c / f
            
            # Transform Camera Frame -> Drone Frame
            # Cam Z (forward) -> Drone X (forward)
            # Cam X (right)   -> Drone -Y (left)
            # Cam Y (down)    -> Drone -Z (up)
            x_b = z_c
            y_b = -x_c
            z_b = -y_c
            
            points = np.column_stack((x_b, y_b, z_b))
            
            header = Header()
            header.frame_id = "base_link"
            header.stamp = self.get_clock().now().to_msg()
            pc_msg = pc2.create_cloud_xyz32(header, points)
            self.pcl_pub.publish(pc_msg)
                
        except Exception:
            pass

def main():
    rclpy.init()
    node = BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()