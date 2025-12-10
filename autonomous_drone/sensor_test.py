#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import numpy as np
import time


class SensorDiagnostic(Node):
    def __init__(self):
        super().__init__('sensor_diagnostic')
        
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Subscribers
        self.depth_subscriber = self.create_subscription(
            Image, '/depth_camera', self.depth_callback, sensor_qos)
        self.lidar_subscriber = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, sensor_qos)
        
        # Stats
        self.depth_count = 0
        self.lidar_count = 0
        self.last_depth_time = 0
        self.last_lidar_time = 0
        
        self.depth_stats = []
        self.lidar_stats = []
        
        self.bridge = CvBridge()
        
        # Print diagnostics every 2 seconds
        self.timer = self.create_timer(2.0, self.print_diagnostics)
        
        self.get_logger().info("🔍 Sensor Diagnostic Tool Started")
        self.get_logger().info("=" * 80)

    def depth_callback(self, msg):
        """Analyze depth camera data"""
        try:
            current_time = time.time()
            self.depth_count += 1
            
            # Calculate frequency
            if self.last_depth_time > 0:
                freq = 1.0 / (current_time - self.last_depth_time)
            else:
                freq = 0
            self.last_depth_time = current_time
            
            # Convert image
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            height, width = depth_image.shape
            
            # Analyze image quality
            valid_pixels = depth_image[(depth_image > 0.1) & (depth_image < 20.0) & np.isfinite(depth_image)]
            
            if valid_pixels.size > 0:
                stats = {
                    'freq': freq,
                    'shape': (height, width),
                    'valid_pixels': valid_pixels.size,
                    'total_pixels': depth_image.size,
                    'valid_percent': (valid_pixels.size / depth_image.size) * 100,
                    'min': np.min(valid_pixels),
                    'max': np.max(valid_pixels),
                    'mean': np.mean(valid_pixels),
                    'std': np.std(valid_pixels),
                    'median': np.median(valid_pixels)
                }
                
                # Analyze regions
                third_w = width // 3
                y_start = int(height * 0.3)
                y_end = int(height * 0.7)
                
                left_region = depth_image[y_start:y_end, 0:third_w]
                center_region = depth_image[y_start:y_end, third_w:2*third_w]
                right_region = depth_image[y_start:y_end, 2*third_w:width]
                
                def region_stats(region, name):
                    valid = region[(region > 0.5) & (region < 15.0) & np.isfinite(region)]
                    if valid.size > 0:
                        return {
                            'name': name,
                            'valid': valid.size,
                            'min': np.min(valid),
                            'p10': np.percentile(valid, 10),
                            'median': np.median(valid),
                            'mean': np.mean(valid)
                        }
                    return None
                
                stats['left'] = region_stats(left_region, 'LEFT')
                stats['center'] = region_stats(center_region, 'CENTER')
                stats['right'] = region_stats(right_region, 'RIGHT')
                
                self.depth_stats.append(stats)
                
                # Keep only last 10 measurements
                if len(self.depth_stats) > 10:
                    self.depth_stats.pop(0)
            
        except Exception as e:
            self.get_logger().error(f"❌ Depth callback error: {e}")

    def lidar_callback(self, msg):
        """Analyze LiDAR data"""
        try:
            current_time = time.time()
            self.lidar_count += 1
            
            # Calculate frequency
            if self.last_lidar_time > 0:
                freq = 1.0 / (current_time - self.last_lidar_time)
            else:
                freq = 0
            self.last_lidar_time = current_time
            
            if msg.ranges:
                ranges_array = np.array(msg.ranges)
                valid_ranges = ranges_array[(ranges_array > msg.range_min) & 
                                           (ranges_array < msg.range_max) & 
                                           np.isfinite(ranges_array)]
                
                stats = {
                    'freq': freq,
                    'total_rays': len(msg.ranges),
                    'valid_rays': len(valid_ranges),
                    'valid_percent': (len(valid_ranges) / len(msg.ranges)) * 100,
                    'angle_min': msg.angle_min,
                    'angle_max': msg.angle_max,
                    'angle_increment': msg.angle_increment,
                    'range_min': msg.range_min,
                    'range_max': msg.range_max,
                }
                
                if len(valid_ranges) > 0:
                    stats['min'] = np.min(valid_ranges)
                    stats['max'] = np.max(valid_ranges)
                    stats['mean'] = np.mean(valid_ranges)
                    stats['std'] = np.std(valid_ranges)
                    stats['median'] = np.median(valid_ranges)
                    
                    # Analyze front sector (center 90 degrees)
                    total_ranges = len(msg.ranges)
                    front_start = total_ranges // 4
                    front_end = 3 * total_ranges // 4
                    front_ranges = ranges_array[front_start:front_end]
                    valid_front = front_ranges[(front_ranges > msg.range_min) & 
                                               (front_ranges < msg.range_max) & 
                                               np.isfinite(front_ranges)]
                    
                    if len(valid_front) > 0:
                        stats['front_min'] = np.min(valid_front)
                        stats['front_p10'] = np.percentile(valid_front, 10)
                        stats['front_median'] = np.median(valid_front)
                
                self.lidar_stats.append(stats)
                
                # Keep only last 10 measurements
                if len(self.lidar_stats) > 10:
                    self.lidar_stats.pop(0)
                    
        except Exception as e:
            self.get_logger().error(f"❌ LiDAR callback error: {e}")

    def print_diagnostics(self):
        """Print comprehensive sensor diagnostics"""
        self.get_logger().info("\n" + "=" * 80)
        self.get_logger().info("📊 SENSOR DIAGNOSTICS")
        self.get_logger().info("=" * 80)
        
        # Depth Camera Status
        self.get_logger().info("\n📷 DEPTH CAMERA:")
        if self.depth_count == 0:
            self.get_logger().error("❌ NO DATA RECEIVED - Check if depth camera is publishing")
        else:
            self.get_logger().info(f"✅ Messages received: {self.depth_count}")
            
            if self.depth_stats:
                latest = self.depth_stats[-1]
                self.get_logger().info(f"   Frequency: {latest['freq']:.1f} Hz")
                self.get_logger().info(f"   Image size: {latest['shape']}")
                self.get_logger().info(f"   Valid pixels: {latest['valid_pixels']}/{latest['total_pixels']} ({latest['valid_percent']:.1f}%)")
                
                if latest['valid_percent'] < 50:
                    self.get_logger().warn("⚠️  LOW VALID PIXEL PERCENTAGE - Check camera quality")
                
                self.get_logger().info(f"   Distance range: {latest['min']:.2f}m - {latest['max']:.2f}m")
                self.get_logger().info(f"   Mean distance: {latest['mean']:.2f}m ± {latest['std']:.2f}m")
                
                # Region analysis
                self.get_logger().info("\n   Region Analysis:")
                for region_key in ['left', 'center', 'right']:
                    region = latest.get(region_key)
                    if region:
                        self.get_logger().info(
                            f"   {region['name']:8s}: min={region['min']:.2f}m, "
                            f"p10={region['p10']:.2f}m, median={region['median']:.2f}m, "
                            f"mean={region['mean']:.2f}m, valid_px={region['valid']}"
                        )
                    else:
                        self.get_logger().warn(f"   {region_key:8s}: NO VALID DATA")
                
                # Check for stability
                if len(self.depth_stats) >= 5:
                    recent_mins = [s['center']['min'] if s.get('center') else 0 for s in self.depth_stats[-5:]]
                    recent_mins = [m for m in recent_mins if m > 0]
                    if recent_mins:
                        variation = np.std(recent_mins)
                        if variation > 1.0:
                            self.get_logger().warn(f"⚠️  HIGH VARIATION in readings: std={variation:.2f}m - Sensor might be noisy")
                        else:
                            self.get_logger().info(f"✅ Stable readings: std={variation:.2f}m")
        
        # LiDAR Status
        self.get_logger().info("\n📡 LIDAR:")
        if self.lidar_count == 0:
            self.get_logger().error("❌ NO DATA RECEIVED - Check if LiDAR is publishing")
            self.get_logger().info("   Expected topic: /scan")
            self.get_logger().info("   Run: ros2 topic list | grep scan")
            self.get_logger().info("   Run: ros2 topic echo /scan --once")
        else:
            self.get_logger().info(f"✅ Messages received: {self.lidar_count}")
            
            if self.lidar_stats:
                latest = self.lidar_stats[-1]
                self.get_logger().info(f"   Frequency: {latest['freq']:.1f} Hz")
                self.get_logger().info(f"   Total rays: {latest['total_rays']}")
                self.get_logger().info(f"   Valid rays: {latest['valid_rays']} ({latest['valid_percent']:.1f}%)")
                
                if latest['valid_percent'] < 30:
                    self.get_logger().warn("⚠️  LOW VALID RAY PERCENTAGE - Check LiDAR quality")
                
                self.get_logger().info(f"   Scan range: {latest['angle_min']:.2f} to {latest['angle_max']:.2f} rad")
                self.get_logger().info(f"   Distance range: {latest['range_min']:.2f}m - {latest['range_max']:.2f}m")
                
                if 'min' in latest:
                    self.get_logger().info(f"   Measured range: {latest['min']:.2f}m - {latest['max']:.2f}m")
                    self.get_logger().info(f"   Mean distance: {latest['mean']:.2f}m ± {latest['std']:.2f}m")
                    
                    if 'front_min' in latest:
                        self.get_logger().info(f"\n   Front Sector (±45°):")
                        self.get_logger().info(f"      Minimum: {latest['front_min']:.2f}m")
                        self.get_logger().info(f"      10th percentile: {latest['front_p10']:.2f}m")
                        self.get_logger().info(f"      Median: {latest['front_median']:.2f}m")
        
        # Recommendations
        self.get_logger().info("\n💡 RECOMMENDATIONS:")
        
        if self.depth_count == 0 and self.lidar_count == 0:
            self.get_logger().error("❌ NO SENSORS WORKING!")
            self.get_logger().info("   1. Check if Gazebo is running")
            self.get_logger().info("   2. Check sensor topics: ros2 topic list")
            self.get_logger().info("   3. Verify sensor plugins in your drone model")
        
        elif self.depth_count == 0:
            self.get_logger().warn("⚠️  Depth camera not working - only using LiDAR")
        
        elif self.lidar_count == 0:
            self.get_logger().warn("⚠️  LiDAR not working - only using depth camera")
        
        else:
            self.get_logger().info("✅ Both sensors are receiving data")
            
            # Check sensor quality
            if self.depth_stats and self.depth_stats[-1]['valid_percent'] < 50:
                self.get_logger().warn("⚠️  Depth camera has low valid pixel percentage")
            
            if self.lidar_stats and self.lidar_stats[-1]['valid_percent'] < 30:
                self.get_logger().warn("⚠️  LiDAR has low valid ray percentage")
        
        self.get_logger().info("=" * 80 + "\n")


def main(args=None):
    rclpy.init(args=args)
    node = SensorDiagnostic()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("\n👋 Diagnostic tool shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()