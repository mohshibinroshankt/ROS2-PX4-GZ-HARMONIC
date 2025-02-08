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
from collections import deque

class Obstacle3D:
    """Class to represent 3D obstacles"""
    def __init__(self, x, y, z_min, z_max, radius):
        self.x = x
        self.y = y
        self.z_min = z_min
        self.z_max = z_max
        self.radius = radius
        self.last_updated = time.time()

class Node3D:
    """Node class for 3D path planning"""
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        self.parent = None
        self.clearance = float('inf')

class OffboardControl(Node):
    def __init__(self) -> None:
        super().__init__('offboard_control_3d')

        # Configure QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
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
            '/scan',
            self.laser_scan_callback,
            1)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -5.0
        self.land_position = [7.0, 0.0, 0.1]
        self.state = "TAKEOFF"
        self.position_threshold = 0.3

        # 3D Obstacle detection parameters
        self.obstacles_3d = {}  # Dictionary to store 3D obstacles
        self.obstacle_grid = {}  # Grid-based obstacle representation
        self.grid_resolution = 0.5  # 0.5m grid cells
        self.obstacle_height = 3.0  # Assumed height of obstacles
        self.min_distance_to_obstacle = 1.2
        self.safety_margin = 1.8
        self.emergency_stop_distance = 1.5
        self.is_emergency = False
        self.obstacle_expiry_time = 4.2  # Time after which obstacles are considered stale
        
        # Height maintenance parameters
        self.height_check_threshold = 0.5  # Threshold for height deviation
        self.needs_height_correction = False
        self.height_correction_margin = 1.0  # Extra margin for height correction
        
        # Parameters for continuous planning
        self.last_lidar_update = 0
        self.lidar_update_interval = 0.1
        self.last_replan_time = 0
        self.replan_interval = 0.5
        self.environment_changed = False
        self.minimum_passage_width = 2.5
        self.path_clearance_threshold = 2.0

        # Path planning parameters
        self.path = []
        self.current_path_index = 0
        self.rrt_step_size = 0.5
        self.path_queue = deque()
        self.local_path_window = 5

        # Create timers
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.obstacle_check_timer = self.create_timer(0.1, self.check_obstacles)
        self.planning_timer = self.create_timer(0.5, self.continuous_planning)
        self.height_check_timer = self.create_timer(0.1, self.check_height)  # New timer for height monitoring

    def check_height(self):
        """Monitor and correct vehicle height"""
        if self.state not in ["FOLLOW_PATH", "PLAN_PATH"]:
            return

        current_height = self.vehicle_local_position.z
        height_error = abs(current_height - self.takeoff_height)

        if height_error > self.height_check_threshold:
            self.needs_height_correction = True
            self.get_logger().warn(f"Height deviation detected: {height_error}m. Initiating correction.")
            self.handle_height_correction()

    def handle_height_correction(self):
        """Handle height correction when needed"""
        if not self.needs_height_correction:
            return

        # Get current position
        current_pos = [self.vehicle_local_position.x, 
                      self.vehicle_local_position.y,
                      self.vehicle_local_position.z]

        # Check if we have enough clearance to move up
        if self.check_vertical_clearance(current_pos):
            # Move to safe height
            target_height = self.takeoff_height - self.height_correction_margin
            self.publish_position_setpoint(current_pos[0], current_pos[1], target_height)
            
            if abs(self.vehicle_local_position.z - target_height) < self.position_threshold:
                self.needs_height_correction = False
                self.get_logger().info("Height corrected successfully")
                # Trigger replanning from new height
                self.environment_changed = True
        else:
            # If we can't move up safely, try to find a clearer area
            self.handle_emergency()

    def check_vertical_clearance(self, position):
        """Check if there's enough vertical clearance to move up"""
        for obstacle_id, obstacle in self.obstacles_3d.items():
            horizontal_dist = sqrt((position[0] - obstacle.x)**2 + 
                                 (position[1] - obstacle.y)**2)
            
            if (horizontal_dist < obstacle.radius + self.safety_margin and 
                position[2] > obstacle.z_min - self.safety_margin and 
                position[2] < obstacle.z_max + self.safety_margin):
                return False
        return True

    def laser_scan_callback(self, msg):
        """Enhanced laser scan processing with 3D obstacle mapping"""
        current_time = time.time()
        if current_time - self.last_lidar_update < self.lidar_update_interval:
            return

        self.last_lidar_update = current_time
        current_height = self.vehicle_local_position.z
        
        # Clear old obstacles
        self.clean_old_obstacles()

        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                # Convert to Cartesian coordinates
                x = r * cos(angle)
                y = r * sin(angle)
                
                # Transform to vehicle frame
                x_vehicle = x + self.vehicle_local_position.x
                y_vehicle = y + self.vehicle_local_position.y
                
                # Create or update 3D obstacle
                self.update_3d_obstacle(x_vehicle, y_vehicle, current_height)
            
            angle += msg.angle_increment

        # Check if environment has significantly changed
        self.environment_changed = self.check_environment_change()

    def update_3d_obstacle(self, x, y, current_height):
        """Update or create 3D obstacle representation"""
        grid_x = round(x / self.grid_resolution)
        grid_y = round(y / self.grid_resolution)
        grid_key = (grid_x, grid_y)

        if grid_key in self.obstacles_3d:
            # Update existing obstacle
            obstacle = self.obstacles_3d[grid_key]
            obstacle.z_min = min(obstacle.z_min, current_height - self.obstacle_height/2)
            obstacle.z_max = max(obstacle.z_max, current_height + self.obstacle_height/2)
            obstacle.last_updated = time.time()
        else:
            # Create new obstacle
            self.obstacles_3d[grid_key] = Obstacle3D(
                x=x,
                y=y,
                z_min=current_height - self.obstacle_height/2,
                z_max=current_height + self.obstacle_height/2,
                radius=self.min_distance_to_obstacle
            )

    def clean_old_obstacles(self):
        """Remove stale obstacles"""
        current_time = time.time()
        stale_obstacles = []
        
        for grid_key, obstacle in self.obstacles_3d.items():
            if current_time - obstacle.last_updated > self.obstacle_expiry_time:
                stale_obstacles.append(grid_key)
        
        for grid_key in stale_obstacles:
            del self.obstacles_3d[grid_key]

    def check_environment_change(self):
        """Check if the 3D environment has significantly changed"""
        # Implement logic to detect significant changes in obstacle positions/dimensions
        # This is a simplified version - you might want to make it more sophisticated
        return len(self.obstacles_3d) > 0

    def check_obstacles(self):
        """Check for obstacles in 3D space"""
        if not self.obstacles_3d or self.state not in ["FOLLOW_PATH", "PLAN_PATH"]:
            return

        current_pos = [self.vehicle_local_position.x, 
                      self.vehicle_local_position.y,
                      self.vehicle_local_position.z]
        
        for obstacle in self.obstacles_3d.values():
            # Check both horizontal and vertical distance
            horizontal_dist = sqrt((current_pos[0] - obstacle.x)**2 + 
                                 (current_pos[1] - obstacle.y)**2)
            
            if (horizontal_dist < self.emergency_stop_distance and 
                current_pos[2] > obstacle.z_min - self.safety_margin and 
                current_pos[2] < obstacle.z_max + self.safety_margin):
                self.handle_emergency()
                return

    def handle_emergency(self):
        """Enhanced emergency handling with 3D awareness"""
        if not self.is_emergency:
            self.is_emergency = True
            self.get_logger().warn("Emergency: Obstacle too close! Executing 3D avoidance maneuver.")
            
            current_pos = [self.vehicle_local_position.x, 
                          self.vehicle_local_position.y,
                          self.vehicle_local_position.z]
            
            # Find nearest obstacle
            nearest_obstacle = None
            min_dist = float('inf')
            for obstacle in self.obstacles_3d.values():
                dist = sqrt((current_pos[0] - obstacle.x)**2 + 
                          (current_pos[1] - obstacle.y)**2)
                if dist < min_dist:
                    min_dist = dist
                    nearest_obstacle = obstacle
            
            if nearest_obstacle:
                # Calculate escape vector in horizontal plane
                escape_vector = np.array([
                    current_pos[0] - nearest_obstacle.x,
                    current_pos[1] - nearest_obstacle.y
                ])
                escape_vector = escape_vector / np.linalg.norm(escape_vector)
                
                # Set emergency setpoint with height adjustment
                escape_point = [
                    current_pos[0] + escape_vector[0] * 3,
                    current_pos[1] + escape_vector[1] * 3,
                    self.takeoff_height  # Return to safe height
                ]
                
                self.publish_position_setpoint(escape_point[0], 
                                            escape_point[1], 
                                            escape_point[2])

    def continuous_planning(self):
        """Continuous path planning with 3D awareness"""
        if self.state not in ["FOLLOW_PATH", "PLAN_PATH"]:
            return

        current_time = time.time()
        if (current_time - self.last_replan_time < self.replan_interval and 
            not self.environment_changed and 
            not self.is_emergency and
            not self.needs_height_correction):
            return

        self.last_replan_time = current_time
        self.environment_changed = False

        start_pos = [self.vehicle_local_position.x, 
                    self.vehicle_local_position.y,
                    self.vehicle_local_position.z]

        if self.state == "FOLLOW_PATH" and self.path:
            local_goal = self.get_local_goal()
        else:
            local_goal = self.land_position

        new_path = self.plan_rrt_path(start_pos, local_goal)
        
        if new_path:
            if self.validate_path_clearance_3d(new_path):
                self.update_path(new_path)
                self.get_logger().info("Path replanned successfully")
            else:
                self.get_logger().warn("Generated path too narrow or unsafe, retrying...")
                self.handle_narrow_path()

    def validate_path_clearance_3d(self, path):
        """Validate if the path has sufficient clearance in 3D"""
        if not path:
            return False

        for i in range(len(path) - 1):
            if not self.check_segment_clearance_3d(path[i], path[i + 1]):
                return False
        return True

    def check_segment_clearance_3d(self, point1, point2):
        """Check if a path segment has sufficient clearance in 3D"""
        num_checks = 10
        for i in range(num_checks + 1):
            t = i / num_checks
            x = point1[0] + t * (point2[0] - point1[0])
            y = point1[1] + t * (point2[1] - point1[1])
            z = point1[2] + t * (point2[2] - point1[2])
            
            # Check clearance against all 3D obstacles
            for obstacle in self.obstacles_3d.values():
                horizontal_dist = sqrt((x - obstacle.x)**2 + (y - obstacle.y)**2)
                
                if (horizontal_dist < obstacle.radius + self.minimum_passage_width/2 and
                    z > obstacle.z_min - self.safety_margin and
                    z < obstacle.z_max + self.safety_margin):
                    return False
        return True

    def plan_rrt_path(self, start, goal):
        """Plan a path using RRT algorithm with 3D awareness"""
        nodes = [Node3D(start[0], start[1], start[2])]
        goal_node = Node3D(goal[0], goal[1], goal[2])
        
        for _ in range(2000):  # Maximum iterations
            if random.random() < 0.1:  # Bias towards goal
                sample = [goal[0], goal[1], goal[2]]
            else:
                sample = [
                    random.uniform(start[0] - 10, start[0] + 10),
                    random.uniform(start[1] - 10, start[1] + 10),
                    random.uniform(self.takeoff_height - 1, self.takeoff_height + 1)
                ]
            
            # Find nearest node
            nearest = min(nodes, key=lambda n: sqrt(
                (n.x - sample[0])**2 + 
                (n.y - sample[1])**2 + 
                (n.z - sample[2])**2))
            
            # Create new node
            direction = np.array([
                sample[0] - nearest.x,
                sample[1] - nearest.y,
                sample[2] - nearest.z
            ])
            
            norm = np.linalg.norm(direction)
            if norm == 0:
                continue
                
            direction = direction / norm
            new_point = [
                nearest.x + direction[0] * self.rrt_step_size,
                nearest.y + direction[1] * self.rrt_step_size,
                nearest.z + direction[2] * self.rrt_step_size
            ]
            
            # Check if new point is collision-free
            if self.is_point_safe_3d(new_point):
                new_node = Node3D(new_point[0], new_point[1], new_point[2])
                new_node.parent = nearest
                nodes.append(new_node)
                
                # Check if we can reach goal
                dist_to_goal = sqrt(
                    (new_node.x - goal[0])**2 + 
                    (new_node.y - goal[1])**2 + 
                    (new_node.z - goal[2])**2)
                
                if dist_to_goal < self.rrt_step_size:
                    goal_node.parent = new_node
                    return self.extract_path(goal_node)
        
        return None

    def is_point_safe_3d(self, point):
        """Check if a point is safe in 3D space"""
        for obstacle in self.obstacles_3d.values():
            horizontal_dist = sqrt(
                (point[0] - obstacle.x)**2 + 
                (point[1] - obstacle.y)**2)
            
            if (horizontal_dist < obstacle.radius + self.safety_margin and
                point[2] > obstacle.z_min - self.safety_margin and
                point[2] < obstacle.z_max + self.safety_margin):
                return False
        return True

    def extract_path(self, goal_node):
        """Extract path from goal node to start node"""
        path = []
        current = goal_node
        while current:
            path.append([current.x, current.y, current.z])
            current = current.parent
        return path[::-1]

    def get_local_goal(self):
        """Get local goal from current path"""
        if not self.path:
            return self.land_position
            
        current_pos = np.array([
            self.vehicle_local_position.x,
            self.vehicle_local_position.y,
            self.vehicle_local_position.z
        ])
        
        # Ensure current_path_index is within bounds
        if self.current_path_index >= len(self.path):
            return self.land_position
        
        # Find the furthest visible point in the path
        remaining_points = len(self.path) - self.current_path_index
        look_ahead = min(remaining_points, self.local_path_window)
        
        for i in range(look_ahead):
            if self.current_path_index + i >= len(self.path):
                break
                
            point = self.path[self.current_path_index + i]
            if not self.is_path_to_point_clear(current_pos, point):
                if i == 0:  # If even the next point isn't clear
                    return self.path[self.current_path_index]  # Stay at current target
                return self.path[self.current_path_index + i - 1]
        
        # If all points in the window are clear, return the furthest one
        return self.path[self.current_path_index + look_ahead - 1]


    def is_path_to_point_clear(self, start, end):
        """Check if path to point is clear of obstacles"""
        direction = np.array(end) - np.array(start)
        distance = np.linalg.norm(direction)
        if distance == 0:
            return True
            
        direction = direction / distance
        num_checks = max(int(distance / self.grid_resolution), 1)
        
        for i in range(num_checks + 1):
            point = start + direction * (i * distance / num_checks)
            if not self.is_point_safe_3d(point):
                return False
        return True

    def update_path(self, new_path):
        """Update the current path"""
        self.path = new_path
        self.current_path_index = 0
        self.state = "FOLLOW_PATH"

    def handle_narrow_path(self):
        """Handle cases where path is too narrow"""
        self.get_logger().warn("Searching for alternative path with better clearance...")
        # Temporarily increase safety margins and retry planning
        original_margin = self.safety_margin
        self.safety_margin *= 1.5
        
        new_path = self.plan_rrt_path(
            [self.vehicle_local_position.x,
             self.vehicle_local_position.y,
             self.vehicle_local_position.z],
            self.land_position
        )
        
        self.safety_margin = original_margin
        
        if new_path and self.validate_path_clearance_3d(new_path):
            self.update_path(new_path)
        else:
            self.handle_emergency()

    def timer_callback(self):
        """Timer callback for main control loop"""
        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()
        
        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1
            
        # Publish offboard control mode
        self.publish_offboard_control_mode()
        
        # State machine
        if self.state == "TAKEOFF":
            if abs(self.vehicle_local_position.z - self.takeoff_height) < self.position_threshold:
                self.state = "PLAN_PATH"
            else:
                self.publish_position_setpoint(
                    self.vehicle_local_position.x,
                    self.vehicle_local_position.y,
                    self.takeoff_height
                )
                
        elif self.state == "PLAN_PATH":
            start_pos = [
                self.vehicle_local_position.x,
                self.vehicle_local_position.y,
                self.vehicle_local_position.z
            ]
            new_path = self.plan_rrt_path(start_pos, self.land_position)
            
            if new_path:
                self.update_path(new_path)
            
        elif self.state == "FOLLOW_PATH":
            if self.current_path_index >= len(self.path):
                self.state = "LAND"
                return
                
            target = self.path[self.current_path_index]
            current_pos = np.array([
                self.vehicle_local_position.x,
                self.vehicle_local_position.y,
                self.vehicle_local_position.z
            ])
            
            if np.linalg.norm(current_pos - np.array(target)) < self.position_threshold:
                self.current_path_index += 1
            
            self.publish_position_setpoint(target[0], target[1], target[2])
            
        elif self.state == "LAND":
            if abs(self.vehicle_local_position.z) < 0.1:
                self.state = "COMPLETE"
                self.disarm()
            else:
                self.publish_position_setpoint(
                    self.land_position[0],
                    self.land_position[1],
                    self.land_position[2]
                )

    def arm(self):
        """Send arm command"""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm command sent")

    def disarm(self):
        """Send disarm command"""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info("Disarm command sent")

    def engage_offboard_mode(self):
        """Switch to offboard mode"""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.get_logger().info("Offboard mode command sent")

    def publish_offboard_control_mode(self):
        """Publish offboard control mode"""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x, y, z, yaw=0.0):
        """Publish position setpoint"""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        """Publish vehicle command"""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def vehicle_local_position_callback(self, msg):
        """Callback for local position updates"""
        self.vehicle_local_position = msg

    def vehicle_status_callback(self, msg):
        """Callback for vehicle status updates"""
        self.vehicle_status = msg

def main(args=None):
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()










