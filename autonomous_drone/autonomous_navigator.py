#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import heapq
import math
import numpy as np

# ROS Messages
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

# PX4 Messages
from px4_msgs.msg import TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, OffboardControlMode

class AutonomousNavigator(Node):
    def __init__(self):
        super().__init__('autonomous_navigator')

        # --- SAFETY CONFIGURATION ---
        self.acceptance_radius = 0.40   # larger radius for smoother turns
        self.flight_altitude = -1.5     # Target height (NED, negative is Up)
        self.safety_radius_cells = 4    # Crucial: How many grid cells to keep away from walls
                                        # If res=0.1m, 4 cells = 0.4m margin
        
        # --- State Variables ---
        self.map_data = None
        self.map_info = None
        self.current_pos_enu = (0.0, 0.0) # (x=East, y=North)
        self.current_alt = 0.0
        self.active_path = []             
        self.waypoint_index = 0
        self.vehicle_status_ready = False
        self.is_taking_off = False

        # --- QoS Profiles ---
        qos_sensor = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        qos_reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)

        # --- Subscribers ---
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos_reliable)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, qos_reliable)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.position_callback, qos_sensor)

        # --- Publishers ---
        self.viz_path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.traj_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.offboard_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)

        # --- Control Loop (20Hz) ---
        self.create_timer(0.05, self.control_loop)
        self.get_logger().info("Safe Navigator Ready! Waiting for Map...")

    # =========================================================================
    #                               CALLBACKS
    # =========================================================================

    def map_callback(self, msg):
        self.map_info = msg.info
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))

    def position_callback(self, msg):
        # PX4 (NED) -> ROS (ENU)
        self.current_pos_enu = (msg.y, msg.x)
        self.current_alt = -msg.z # Convert NED z to Altitude (Positive Up)
        self.vehicle_status_ready = True

    def goal_callback(self, msg):
        if self.map_data is None or not self.vehicle_status_ready:
            self.get_logger().warn("Not ready to plan yet.")
            return

        target_x, target_y = msg.pose.position.x, msg.pose.position.y
        start_x, start_y = self.current_pos_enu

        self.get_logger().info(f"Planning safe path to ({target_x:.1f}, {target_y:.1f})...")

        # 1. Plan Path
        path_grid = self.run_a_star((start_x, start_y), (target_x, target_y))

        if path_grid:
            self.get_logger().info(f"Path found! Length: {len(path_grid)}")
            self.active_path = [self.grid_to_world(gx, gy) for gx, gy in path_grid]
            self.waypoint_index = 0
            self.is_taking_off = True  # Reset takeoff flag

            # 2. Visualize
            self.publish_viz_path(self.active_path)

            # 3. Start
            self.engage_offboard_mode()
            self.arm_drone()
        else:
            self.get_logger().error("No SAFE path found! Goal might be too close to a wall.")

    # =========================================================================
    #                               A* ALGORITHM (WITH INFLATION)
    # =========================================================================

    def world_to_grid(self, wx, wy):
        if self.map_info is None: return 0, 0
        gx = int((wx - self.map_info.origin.position.x) / self.map_info.resolution)
        gy = int((wy - self.map_info.origin.position.y) / self.map_info.resolution)
        return gx, gy

    def grid_to_world(self, gx, gy):
        wx = self.map_info.origin.position.x + (gx + 0.5) * self.map_info.resolution
        wy = self.map_info.origin.position.y + (gy + 0.5) * self.map_info.resolution
        return wx, wy

    def is_safe(self, gx, gy):
        # Check the specific cell
        if not (0 <= gx < self.map_info.width and 0 <= gy < self.map_info.height):
            return False
        
        if self.map_data[gy, gx] > 50: # Direct Hit
            return False

        # --- INFLATION CHECK (The "Fat Drone" Logic) ---
        margin = self.safety_radius_cells
        # Check a square around the point
        for dy in range(-margin, margin + 1):
            for dx in range(-margin, margin + 1):
                nx, ny = gx + dx, gy + dy
                # If any cell in this box is a wall, this point is unsafe
                if 0 <= nx < self.map_info.width and 0 <= ny < self.map_info.height:
                    if self.map_data[ny, nx] > 50:
                        return False
        return True

    def run_a_star(self, start_world, goal_world):
        start_node = self.world_to_grid(*start_world)
        goal_node = self.world_to_grid(*goal_world)
        
        open_list = []
        heapq.heappush(open_list, (0, start_node))
        came_from = {}
        g_score = {start_node: 0}
        
        movements = [(0,1), (0,-1), (1,0), (-1,0), (1,1), (1,-1), (-1,1), (-1,-1)]

        while open_list:
            _, current = heapq.heappop(open_list)

            if current == goal_node:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.reverse()
                return path

            gx, gy = current
            for dx, dy in movements:
                neighbor = (gx + dx, gy + dy)
                
                # Use the new Safety Check
                if not self.is_safe(neighbor[0], neighbor[1]):
                    continue

                dist_cost = math.sqrt(dx**2 + dy**2)
                tentative_g = g_score[current] + dist_cost

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    f = tentative_g + math.sqrt((goal_node[0]-neighbor[0])**2 + (goal_node[1]-neighbor[1])**2)
                    heapq.heappush(open_list, (f, neighbor))
                    came_from[neighbor] = current
        
        return None

    # =========================================================================
    #                            CONTROL LOOP
    # =========================================================================

    def control_loop(self):
        self.publish_offboard_control_mode()

        if not self.active_path: return

        # --- 1. TAKEOFF SAFETY CHECK ---
        # If we are below 1.0m altitude, ONLY fly up. Do not move sideways.
        if self.current_alt < 1.0:
            self.get_logger().info(f"Taking off... Alt: {self.current_alt:.2f}m")
            # Hover at current X/Y, but target Z
            start_x, start_y = self.current_pos_enu
            self.send_position_setpoint((start_x, start_y)) 
            return # Skip the rest until we are high enough

        # --- 2. PATH FOLLOWING ---
        if self.waypoint_index >= len(self.active_path):
            # Hold last position
            self.send_position_setpoint(self.active_path[-1])
            return

        target_enu = self.active_path[self.waypoint_index]
        curr_x, curr_y = self.current_pos_enu
        dist = math.sqrt((target_enu[0] - curr_x)**2 + (target_enu[1] - curr_y)**2)

        if dist < self.acceptance_radius:
            self.waypoint_index += 1
            if self.waypoint_index < len(self.active_path):
                target_enu = self.active_path[self.waypoint_index]

        self.send_position_setpoint(target_enu)

    def send_position_setpoint(self, enu_pos):
        # ENU (East, North) -> NED (North, East)
        target_east = enu_pos[0]
        target_north = enu_pos[1]

        msg = TrajectorySetpoint()
        msg.position = [target_north, target_east, self.flight_altitude]
        msg.yaw = float('nan') 
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.traj_pub.publish(msg)

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(msg)

    def engage_offboard_mode(self):
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

    def arm_drone(self):
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def send_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)

    def publish_viz_path(self, path_list):
        msg = Path()
        msg.header = Header()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in path_list:
            p = PoseStamped()
            p.header = msg.header
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.position.z = abs(self.flight_altitude)
            msg.poses.append(p)
        self.viz_path_pub.publish(msg)

def main():
    rclpy.init()
    node = AutonomousNavigator()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()