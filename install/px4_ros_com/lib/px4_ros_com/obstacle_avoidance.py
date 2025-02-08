#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from sensor_msgs.msg import LaserScan
import numpy as np
import time
import random
from math import sqrt, atan2, cos, sin, pi

class Node2D:
    """Node class for 2D path planning"""
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z  # Keep z for height control
        self.parent = None

class OffboardControl(Node):
    def __init__(self) -> None:
        super().__init__('offboard_control_2d')

        # Configure QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers (same as before)
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', 
            self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', 
            self.vehicle_status_callback, qos_profile)
        
        # LaserScan subscriber
        self.laser_scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',  # This will be the bridged topic
            self.laser_scan_callback,
            10)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -5.0
        self.land_position = [7.0, 8.0, 0.1]
        self.state = "TAKEOFF"
        self.position_threshold = 0.3
        self.last_replan_time = 0
        self.replan_interval = 2.0

        # Obstacle detection parameters
        self.obstacles = []  # Will store [x, y] coordinates of detected obstacles
        self.min_distance_to_obstacle = 1.2
        self.safety_margin = 1.8
        self.emergency_stop_distance = 1.5
        self.is_emergency = False

        # Path planning parameters
        self.path = []
        self.current_path_index = 0
        self.rrt_step_size = 0.5

        # Create timers
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.obstacle_check_timer = self.create_timer(0.1, self.check_obstacles)

    def laser_scan_callback(self, msg):
        """Process laser scan data to detect obstacles"""
        self.obstacles = []
        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                # Convert polar coordinates (r, angle) to Cartesian (x, y)
                x = r * cos(angle)
                y = r * sin(angle)
                
                # Transform to vehicle frame
                x_vehicle = x + self.vehicle_local_position.x
                y_vehicle = y + self.vehicle_local_position.y
                
                self.obstacles.append([x_vehicle, y_vehicle])
            angle += msg.angle_increment

    def check_obstacles(self):
        """Check for obstacles in path"""
        if not self.obstacles or self.state not in ["FOLLOW_PATH", "PLAN_PATH"]:
            return

        current_pos = np.array([self.vehicle_local_position.x, self.vehicle_local_position.y])
        
        # Check distance to all obstacles
        for obstacle in self.obstacles:
            obstacle_pos = np.array(obstacle)
            distance = np.linalg.norm(current_pos - obstacle_pos)
            
            if distance < self.emergency_stop_distance:
                self.handle_emergency()
                return

    def handle_emergency(self):
        """Handle emergency situation when obstacles are too close"""
        if not self.is_emergency:
            self.is_emergency = True
            self.get_logger().warn("Emergency: Obstacle too close! Executing avoidance maneuver.")
            
            current_pos = [self.vehicle_local_position.x, self.vehicle_local_position.y]
            
            # Find nearest obstacle
            nearest_obstacle = min(self.obstacles, 
                                 key=lambda o: sqrt((o[0] - current_pos[0])**2 + 
                                                  (o[1] - current_pos[1])**2))
            
            # Calculate escape vector (perpendicular to obstacle direction)
            obstacle_vector = np.array([current_pos[0] - nearest_obstacle[0],
                                     current_pos[1] - nearest_obstacle[1]])
            escape_vector = np.array([-obstacle_vector[1], obstacle_vector[0]])
            escape_vector = escape_vector / np.linalg.norm(escape_vector)
            
            # Set emergency setpoint 3 meters away in escape direction
            escape_point = [current_pos[0] + escape_vector[0] * 3,
                          current_pos[1] + escape_vector[1] * 3,
                          self.takeoff_height]
            
            self.publish_position_setpoint(escape_point[0], escape_point[1], escape_point[2])

    def replan_path(self):
        """Replan path if needed"""
        if self.is_emergency:
            return

        start_pos = [self.vehicle_local_position.x, 
                    self.vehicle_local_position.y,
                    self.vehicle_local_position.z]
        new_path = self.plan_rrt_path(start_pos, self.land_position)
        
        if new_path:
            self.path = new_path
            self.current_path_index = 0
            self.last_replan_time = time.time()
            self.get_logger().info("Path replanned successfully")

    def plan_rrt_path(self, start_pos, goal_pos):
        """Enhanced RRT path planning with improved obstacle avoidance"""
        nodes = [Node2D(start_pos[0], start_pos[1], start_pos[2])]
        goal_node = Node2D(goal_pos[0], goal_pos[1], goal_pos[2])
        
        for _ in range(5000):  # Increased maximum iterations
            if random.random() < 0.2:  # Increased goal bias
                sample = goal_node
            else:
                sample = self.get_valid_sample(start_pos, goal_pos)
            
            if not sample:
                continue

            nearest = self.get_nearest_node(nodes, sample)
            new_node = self.steer(nearest, sample)
            
            if new_node and self.is_collision_free(nearest, new_node):
                new_node.parent = nearest
                nodes.append(new_node)
                
                if self.distance(new_node, goal_node) < self.rrt_step_size:
                    return self.extract_path(new_node)
        
        return None

    def get_valid_sample(self, start_pos, goal_pos):
        """Get a valid random sample point"""
        for _ in range(100):  # Limit attempts
            x = random.uniform(min(start_pos[0], goal_pos[0]) - 5,
                             max(start_pos[0], goal_pos[0]) + 5)
            y = random.uniform(min(start_pos[1], goal_pos[1]) - 5,
                             max(start_pos[1], goal_pos[1]) + 5)
            z = random.uniform(min(start_pos[2], goal_pos[2]) - 5,
                             max(start_pos[2], goal_pos[2]) + 5)
            
            # Check if point is too close to any obstacle
            valid = True
            for obstacle in self.obstacles:
                if sqrt((x - obstacle[0])**2 + (y - obstacle[1])**2 + (z - obstacle[2])**2) < \
                   self.min_distance_to_obstacle:
                    valid = False
                    break
            
            if valid:
                return Node2D(x, y, z)
        
        return None

    def get_nearest_node(self, nodes, target):
        """Find nearest node with optimization for large node sets"""
        if len(nodes) < 100:
            return min(nodes, key=lambda n: self.distance(n, target))
        
        # For large sets, sample a subset
        sample_size = min(50, len(nodes))
        sampled_nodes = random.sample(nodes, sample_size)
        return min(sampled_nodes, key=lambda n: self.distance(n, target))

    def steer(self, from_node, to_node):
        """Steer from one node toward another"""
        dist = self.distance(from_node, to_node)
        if dist < self.rrt_step_size:
            return to_node
            
        theta = atan2(to_node.y - from_node.y, to_node.x - from_node.x)
        phi = atan2(to_node.z - from_node.z, sqrt((to_node.x - from_node.x)**2 + (to_node.y - from_node.y)**2))
        new_x = from_node.x + self.rrt_step_size * np.cos(theta) * np.cos(phi)
        new_y = from_node.y + self.rrt_step_size * np.sin(theta) * np.cos(phi)
        new_z = from_node.z + self.rrt_step_size * np.sin(phi)
        
        return Node2D(new_x, new_y, new_z)

    def extract_path(self, node):
        """Extract the path from the goal node to the start node"""
        path = []
        current = node
        while current is not None:
            path.append([current.x, current.y, current.z])
            current = current.parent
        return path[::-1]

    def is_collision_free(self, node1, node2):
        """Modified collision checking for 2D"""
        num_checks = 10
        
        for i in range(num_checks + 1):
            t = i / num_checks
            x = node1.x + t * (node2.x - node1.x)
            y = node1.y + t * (node2.y - node1.y)
            
            for obstacle in self.obstacles:
                dist = sqrt((x - obstacle[0])**2 + (y - obstacle[1])**2)
                if dist < self.min_distance_to_obstacle:
                    return False
        return True

    def distance(self, node1, node2):
        """Calculate Euclidean distance between nodes"""
        return sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2 + (node1.z - node2.z)**2)

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = atan2(y - self.vehicle_local_position.y, x - self.vehicle_local_position.x)  # Face the target
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, **params):
        """Publish a vehicle command."""
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
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()

        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()

        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            if self.state == "TAKEOFF":
                self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)
                if abs(self.vehicle_local_position.z - self.takeoff_height) < self.position_threshold:
                    self.state = "PLAN_PATH"
                    self.get_logger().info("Takeoff complete. Planning path...")

            elif self.state == "PLAN_PATH":
                start_pos = [self.vehicle_local_position.x, 
                            self.vehicle_local_position.y, 
                            self.vehicle_local_position.z]
                self.path = self.plan_rrt_path(start_pos, self.land_position)
                if self.path:
                    self.state = "FOLLOW_PATH"
                    self.current_path_index = 0
                    self.get_logger().info("Path planned successfully. Starting to follow path...")
                else:
                    self.get_logger().warn("Failed to plan path. Retrying...")

            elif self.state == "FOLLOW_PATH":
                if self.current_path_index < len(self.path):
                    target = self.path[self.current_path_index]
                    self.publish_position_setpoint(target[0], target[1], target[2])

                    if self.distance_to_target(target) < self.position_threshold:
                        self.current_path_index += 1
                        if self.current_path_index >= len(self.path):
                            self.state = "LAND"
                            self.get_logger().info("Path completed. Preparing to land...")
                else:
                    self.state = "LAND"

            elif self.state == "LAND":
                self.land()
                if abs(self.vehicle_local_position.z) < 0.1:
                    self.state = "DISARM"
                    self.get_logger().info("Landing complete. Disarming...")

            elif self.state == "DISARM":
                self.disarm()
                self.destroy_node()
                rclpy.shutdown()

        self.offboard_setpoint_counter += 1

    def distance_to_target(self, target):
        """Calculate distance to target position."""
        return sqrt((self.vehicle_local_position.x - target[0])**2 +
                    (self.vehicle_local_position.y - target[1])**2 +
                    (self.vehicle_local_position.z - target[2])**2)

def main(args=None):
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    
    try:
        rclpy.spin(offboard_control)
    except KeyboardInterrupt:
        offboard_control.get_logger().info('Received interrupt, shutting down...')
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        offboard_control.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()




# def main(args=None):
#     rclpy.init(args=args)
#     offboard_control = OffboardControl()
#     rclpy.spin(offboard_control)
#     offboard_control.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
