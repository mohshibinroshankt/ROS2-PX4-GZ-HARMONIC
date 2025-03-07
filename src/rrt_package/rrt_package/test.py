#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from sensor_msgs.msg import LaserScan
import numpy as np
from math import atan2, sqrt, cos, sin, pi
import heapq
import csv
import os
import pandas as pd
import matplotlib.pyplot as plt

# A* Path Planner
class AStarPlanner:
    def __init__(self, grid_size, resolution):
        self.grid_size = grid_size
        self.resolution = resolution
        self.grid = np.zeros((grid_size, grid_size), dtype=bool)  # True for obstacles

    def plan(self, start, goal):
        """A* algorithm to find the shortest path."""
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                return self.reconstruct_path(came_from, current)

            for neighbor in self.get_neighbors(current):
                tentative_g_score = g_score[current] + self.distance(current, neighbor)

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return None  # No path found

    def heuristic(self, a, b):
        """Euclidean distance heuristic."""
        return sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def distance(self, a, b):
        """Distance between two points."""
        return sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def get_neighbors(self, point):
        """Get valid neighbors for a given point."""
        neighbors = []
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]:
            new_x = point[0] + dx
            new_y = point[1] + dy
            if 0 <= new_x < self.grid_size and 0 <= new_y < self.grid_size and not self.grid[new_x, new_y]:
                neighbors.append((new_x, new_y))
        return neighbors

    def reconstruct_path(self, came_from, current):
        """Reconstruct the path from start to goal."""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]

    def update_grid(self, obstacle_list):
        """Update the grid with new obstacles."""
        self.grid.fill(False)  # Reset grid
        for (x, y, size) in obstacle_list:
            grid_x = int(x / self.resolution)
            grid_y = int(y / self.resolution)
            for dx in range(-int(size / self.resolution), int(size / self.resolution) + 1):
                for dy in range(-int(size / self.resolution), int(size / self.resolution) + 1):
                    if 0 <= grid_x + dx < self.grid_size and 0 <= grid_y + dy < self.grid_size:
                        self.grid[grid_x + dx, grid_y + dy] = True

# Dynamic Window Approach (DWA) for Obstacle Avoidance
class DWA:
    def __init__(self, max_speed, max_yaw_rate, dt, obstacle_safe_distance):
        self.max_speed = max_speed
        self.max_yaw_rate = max_yaw_rate
        self.dt = dt
        self.obstacle_safe_distance = obstacle_safe_distance

    def compute_velocity(self, current_pose, goal, obstacles):
        """Compute the best velocity and yaw rate using DWA."""
        best_speed = 0.0
        best_yaw_rate = 0.0
        best_score = -float('inf')

        for speed in np.linspace(0, self.max_speed, 10):
            for yaw_rate in np.linspace(-self.max_yaw_rate, self.max_yaw_rate, 10):
                # Simulate trajectory
                future_pose = self.simulate_trajectory(current_pose, speed, yaw_rate)

                # Calculate score
                goal_distance = sqrt((future_pose[0] - goal[0])**2 + (future_pose[1] - goal[1])**2)
                obstacle_distance = self.get_min_obstacle_distance(future_pose, obstacles)
                score = goal_distance + 1.0 / (obstacle_distance + 1e-5)

                if score > best_score and obstacle_distance > self.obstacle_safe_distance:
                    best_score = score
                    best_speed = speed
                    best_yaw_rate = yaw_rate

        return best_speed, best_yaw_rate

    def simulate_trajectory(self, pose, speed, yaw_rate):
        """Simulate the trajectory for a given speed and yaw rate."""
        x, y, yaw = pose
        new_x = x + speed * cos(yaw) * self.dt
        new_y = y + speed * sin(yaw) * self.dt
        new_yaw = yaw + yaw_rate * self.dt
        return (new_x, new_y, new_yaw)

    def get_min_obstacle_distance(self, pose, obstacles):
        """Get the minimum distance to obstacles."""
        min_distance = float('inf')
        for (ox, oy, _) in obstacles:
            distance = sqrt((pose[0] - ox)**2 + (pose[1] - oy)**2)
            if distance < min_distance:
                min_distance = distance
        return min_distance

# Offboard Control Node with A* and DWA
class OffboardControl(Node):
    def __init__(self) -> None:
        super().__init__('offboard_control_with_local_planning')

        # QoS Profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers and Subscribers
        self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.vehicle_local_position_subscriber = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.lidar_subscriber = self.create_subscription(LaserScan, '/world/iris_maze_nowall/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan', self.lidar_callback, 10)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -5.0
        self.target_position = [7.0, 0.0, self.takeoff_height]
        self.current_path = None
        self.current_path_index = 0
        self.obstacle_list = []
        self.has_taken_off = False
        self.last_planning_time = 0
        self.planning_interval = 1.0  # Replan every 1 second
        self.waypoint_threshold = 1.0  # Threshold to consider a waypoint reached
        self.velocity_setpoint = 0.2  # Desired velocity
        self.obstacle_detection_angle = pi  # Field of view for obstacle detection
        self.obstacle_safe_distance = 1.5  # Safe distance from obstacles (increased for uncertainty)
        self.nearest_obstacle = None  # Nearest obstacle information
        self.avoidance_direction = None  # Direction to avoid obstacles
        self.is_avoiding = False
        self.last_avoidance_time = 0
        self.avoidance_cooldown = 2.0
        self.replanning_attempts = 0
        self.max_replanning_attempts = 10  # Max attempts before fail-safe
        self.fail_safe_triggered = False

        # A* Planner
        self.grid_size = 100
        self.resolution = 0.5  # meters per grid cell
        self.planner = AStarPlanner(self.grid_size, self.resolution)

        # DWA for Obstacle Avoidance
        self.dwa = DWA(max_speed=1.0, max_yaw_rate=1.0, dt=0.1, obstacle_safe_distance=self.obstacle_safe_distance)

        # Data Logging
        self.log_file = open('drone_data.csv', 'w')
        self.log_writer = csv.writer(self.log_file)
        self.log_writer.writerow(['time', 'x', 'y', 'z', 'target_x', 'target_y', 'nearest_obstacle_distance', 'velocity', 'yaw_rate'])

        # Timers
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.planning_timer = self.create_timer(self.planning_interval, self.planning_timer_callback)

    def lidar_callback(self, msg):
        """Process LIDAR data to update obstacle list."""
        self.obstacle_list = []
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))

        # Find nearest obstacle
        min_distance = float('inf')
        min_angle = 0
        for i, range_val in enumerate(msg.ranges):
            if msg.range_min < range_val < msg.range_max:
                angle = angles[i]
                x = range_val * cos(angle)
                y = range_val * sin(angle)
                global_x = x + self.vehicle_local_position.x
                global_y = y + self.vehicle_local_position.y
                self.obstacle_list.append((global_x, global_y, 0.5))  # Obstacle size = 0.5m

                # Check if obstacle is in front sector
                if abs(angle) < self.obstacle_detection_angle:
                    distance = sqrt(x**2 + y**2)
                    if distance < min_distance:
                        min_distance = distance
                        min_angle = angle

        # Update nearest obstacle information
        if min_distance != float('inf'):
            self.nearest_obstacle = (min_distance, min_angle)
            self.get_logger().info(f"Nearest obstacle: distance={min_distance:.2f}m, angle={min_angle:.2f}rad")
        else:
            self.nearest_obstacle = None

        # Update A* grid
        self.planner.update_grid(self.obstacle_list)

    def planning_timer_callback(self):
        """Periodic path planning."""
        if self.has_taken_off and self.obstacle_list:
            self.plan_path()

    def plan_path(self):
        """Generate path using A*."""
        start = (int(self.vehicle_local_position.x / self.resolution), int(self.vehicle_local_position.y / self.resolution))
        goal = (int(self.target_position[0] / self.resolution), int(self.target_position[1] / self.resolution))

        global_path = self.planner.plan(start, goal)
        if global_path:
            self.current_path = global_path
            self.current_path_index = 0
            self.replanning_attempts = 0
            self.get_logger().info(f"Global path planned: {global_path}")
        else:
            self.replanning_attempts += 1
            self.get_logger().warn("Global planning failed")

    def timer_callback(self):
        """Main control loop."""
        self.publish_offboard_control_heartbeat_signal()

        if self.offboard_setpoint_counter == 5:
            self.set_com_of_loss_t(5.0)
        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()

        if not self.has_taken_off and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.publish_velocity_position_setpoint(0.0, 0.0, self.takeoff_height)
            if abs(self.vehicle_local_position.z - self.takeoff_height) < 0.5:
                self.has_taken_off = True
                self.get_logger().info('Takeoff complete, starting mission')

        elif self.has_taken_off and not self.fail_safe_triggered:
            if self.current_path and self.current_path_index < len(self.current_path):
                current_target = self.current_path[self.current_path_index]
                target_x = current_target[0] * self.resolution
                target_y = current_target[1] * self.resolution

                # Use DWA for local obstacle avoidance
                current_pose = (self.vehicle_local_position.x, self.vehicle_local_position.y, atan2(target_y - self.vehicle_local_position.y, target_x - self.vehicle_local_position.x))
                speed, yaw_rate = self.dwa.compute_velocity(current_pose, (target_x, target_y), self.obstacle_list)

                # Publish setpoint
                self.publish_velocity_position_setpoint(target_x, target_y, self.takeoff_height)

                # Log data
                current_time = self.get_clock().now().to_msg().sec
                self.log_writer.writerow([current_time, self.vehicle_local_position.x, self.vehicle_local_position.y, self.vehicle_local_position.z, self.target_position[0], self.target_position[1], self.nearest_obstacle[0] if self.nearest_obstacle else -1.0, speed, yaw_rate])

                # Check if waypoint is reached
                dist_to_waypoint = sqrt((self.vehicle_local_position.x - target_x)**2 + (self.vehicle_local_position.y - target_y)**2)
                if dist_to_waypoint < self.waypoint_threshold:
                    self.current_path_index += 1
                    self.get_logger().info(f"Reached waypoint {self.current_path_index}")

            # Check if target is reached
            dist_to_target = sqrt((self.vehicle_local_position.x - self.target_position[0])**2 + (self.vehicle_local_position.y - self.target_position[1])**2)
            if dist_to_target < self.waypoint_threshold:
                self.get_logger().info("Target reached. Landing and disarming.")
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)  # Disarm
                self.fail_safe_triggered = True  # Stop further commands

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

    def publish_velocity_position_setpoint(self, x: float, y: float, z: float):
        """Publish position setpoint with velocity limit."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = atan2(y - self.vehicle_local_position.y, x - self.vehicle_local_position.x)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_offboard_control_heartbeat_signal(self):
        """Publish offboard control heartbeat signal."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        for key, value in params.items():
            setattr(msg, key, value)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def set_com_of_loss_t(self, value: float):
        """Set the COM_OF_LOSS_T parameter."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_PARAMETER,
            param1=1.0,  # COM_OF_LOSS_T parameter ID
            param2=value
        )
        self.get_logger().info(f'Set COM_OF_LOSS_T to {value}')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def arm(self):
        """Arm the vehicle."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def vehicle_local_position_callback(self, msg):
        """Callback for vehicle local position."""
        self.vehicle_local_position = msg

    def vehicle_status_callback(self, msg):
        """Callback for vehicle status."""
        self.vehicle_status = msg

    def destroy_node(self):
        """Close the log file and destroy the node."""
        self.log_file.close()
        super().destroy_node()

# Main Function
def main(args=None) -> None:
    print('Starting offboard control node with local path planning...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f"Error: {e}")

# Data Visualization Script
def visualize_data():
    """Visualize the logged data."""
    data = pd.read_csv('drone_data.csv')

    # Plot Drone Trajectory
    plt.figure()
    plt.plot(data['x'], data['y'], label='Drone Path')
    plt.scatter(data['target_x'], data['target_y'], color='red', label='Target')
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.title('Drone Trajectory')
    plt.legend()
    plt.grid()
    plt.show()

    # Plot Distance to Nearest Obstacle
    plt.figure()
    plt.plot(data['time'], data['nearest_obstacle_distance'], label='Distance to Nearest Obstacle')
    plt.axhline(y=1.5, color='r', linestyle='--', label='Safe Distance')
    plt.xlabel('Time (s)')
    plt.ylabel('Distance (m)')
    plt.title('Distance to Nearest Obstacle Over Time')
    plt.legend()
    plt.grid()
    plt.show()

    # Plot Velocity and Yaw Rate
    plt.figure()
    plt.plot(data['time'], data['velocity'], label='Velocity')
    plt.plot(data['time'], data['yaw_rate'], label='Yaw Rate')
    plt.xlabel('Time (s)')
    plt.ylabel('Value')
    plt.title('Velocity and Yaw Rate Over Time')
    plt.legend()
    plt.grid()
    plt.show()

# Run Visualization
visualize_data()