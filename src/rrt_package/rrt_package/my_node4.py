#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from sensor_msgs.msg import LaserScan
import numpy as np
from math import atan2, sqrt, cos, sin, pi
import random

# Node for RRT*
class Node2D:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0.0

# RRT* Path Planner (Local Planner)
class RRTStar:
    def __init__(self, start, goal, obstacle_list, rand_area, max_iter=100, step_size=1.0, goal_sample_rate=0.2, search_radius=3.0):
        self.start = Node2D(start[0], start[1])
        self.goal = Node2D(goal[0], goal[1])
        self.obstacle_list = obstacle_list
        self.min_rand, self.max_rand = rand_area
        self.max_iter = max_iter
        self.step_size = step_size
        self.goal_sample_rate = goal_sample_rate
        self.search_radius = search_radius
        self.node_list = [self.start]

    def plan(self):
        for _ in range(self.max_iter):
            rnd = self.get_random_point()
            nearest_node = self.get_nearest_node(rnd)
            new_node = self.steer(nearest_node, rnd)

            if new_node and not self.check_collision(new_node, self.obstacle_list):
                near_nodes = self.find_near_nodes(new_node)
                new_node = self.choose_parent(new_node, near_nodes)
                if new_node:
                    self.node_list.append(new_node)
                    self.rewire(new_node, near_nodes)

                if self.is_near_goal(new_node):
                    final_node = self.steer(new_node, self.goal)
                    if final_node and not self.check_collision(final_node, self.obstacle_list):
                        return self.generate_final_path(final_node)
        return None

    def steer(self, from_node, to_point):
        dist = sqrt((to_point.x - from_node.x)**2 + (to_point.y - from_node.y)**2)
        if dist > self.step_size:
            theta = atan2(to_point.y - from_node.y, to_point.x - from_node.x)
            new_x = from_node.x + self.step_size * cos(theta)
            new_y = from_node.y + self.step_size * sin(theta)
        else:
            new_x = to_point.x
            new_y = to_point.y
        new_node = Node2D(new_x, new_y)
        new_node.parent = from_node
        new_node.cost = from_node.cost + dist
        return new_node

    def get_random_point(self):
        if random.random() > self.goal_sample_rate:
            return Node2D(
                random.uniform(self.min_rand, self.max_rand),
                random.uniform(self.min_rand, self.max_rand)
            )
        return self.goal

    def get_nearest_node(self, rnd_node):
        distances = [(node.x - rnd_node.x)**2 + (node.y - rnd_node.y)**2 for node in self.node_list]
        return self.node_list[distances.index(min(distances))]

    def check_collision(self, node, obstacle_list):
        for (ox, oy, size) in obstacle_list:
            dx = ox - node.x
            dy = oy - node.y
            d = sqrt(dx*dx + dy*dy)
            if d <= size + 1.0:  # Safety margin
                return True
        return False

    def find_near_nodes(self, new_node):
        n_nodes = len(self.node_list)
        r = self.search_radius
        dist_list = [(node.x - new_node.x)**2 + (node.y - new_node.y)**2 for node in self.node_list]
        near_inds = [i for i, d in enumerate(dist_list) if d <= r**2]
        return [self.node_list[i] for i in near_inds]

    def choose_parent(self, new_node, near_nodes):
        if not near_nodes:
            return new_node
        costs = []
        for near_node in near_nodes:
            d = sqrt((near_node.x - new_node.x)**2 + (near_node.y - new_node.y)**2)
            if not self.check_collision(new_node, self.obstacle_list):
                cost = near_node.cost + d
                costs.append((cost, near_node))
        if not costs:
            return None
        min_cost_node = min(costs, key=lambda x: x[0])[1]
        new_node.parent = min_cost_node
        new_node.cost = min_cost_node.cost + sqrt((new_node.x - min_cost_node.x)**2 + (new_node.y - min_cost_node.y)**2)
        return new_node

    def rewire(self, new_node, near_nodes):
        for near_node in near_nodes:
            d = sqrt((near_node.x - new_node.x)**2 + (near_node.y - new_node.y)**2)
            scost = new_node.cost + d
            if near_node.cost > scost and not self.check_collision(near_node, self.obstacle_list):
                near_node.parent = new_node
                near_node.cost = scost

    def is_near_goal(self, node):
        d = sqrt((node.x - self.goal.x)**2 + (node.y - self.goal.y)**2)
        return d < self.step_size * 2

    def generate_final_path(self, goal_node):
        path = []
        node = goal_node
        while node.parent is not None:
            path.append((node.x, node.y))
            node = node.parent
        path.append((self.start.x, self.start.y))
        return path[::-1]

# Offboard Control Node with RRT* as Local Planner
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
        self.lidar_subscriber = self.create_subscription(LaserScan, '/world/iris_maze/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan', self.lidar_callback, 10)

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
        self.waypoint_threshold = 1.0
        self.velocity_setpoint = 0.1  # Reduced speed for better obstacle avoidance
        self.obstacle_detection_angle = pi 
        self.obstacle_safe_distance = 0.8
        self.nearest_obstacle = None
        self.is_avoiding = False
        self.last_avoidance_time = 0
        self.avoidance_cooldown = 2.0

        # Timers
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.planning_timer = self.create_timer(self.planning_interval, self.planning_timer_callback)

    def set_com_of_loss_t(self, value: float):
        """Set the COM_OF_LOSS_T parameter."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_PARAMETER,
            param1=float(self.get_param_id('COM_OF_LOSS_T')),
            param2=value
        )
        self.get_logger().info(f'Set COM_OF_LOSS_T to {value}')

    def get_param_id(self, param_name: str) -> int:
        """Get the parameter ID for a given parameter name."""
        param_id_map = {
            'COM_OF_LOSS_T': 1,  # Replace with the actual parameter ID
        }
        return param_id_map.get(param_name, 0)

    def lidar_callback(self, msg):
        """Process LIDAR data to update obstacle list."""
        self.obstacle_list = []
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))

        # Find nearest obstacle in front sector
        min_distance = float('inf')
        min_angle = 0
        front_obstacles = []

        for i, range_val in enumerate(msg.ranges):
            if msg.range_min < range_val < msg.range_max:
                angle = angles[i]
                x = range_val * cos(angle)
                y = range_val * sin(angle)

                # Store obstacle in global frame
                if abs(x) < 20 and abs(y) < 20:
                    global_x = x + self.vehicle_local_position.x
                    global_y = y + self.vehicle_local_position.y
                    self.obstacle_list.append((global_x, global_y, 0.5))

                # Check if obstacle is in front sector
                if abs(angle) < self.obstacle_detection_angle:
                    front_obstacles.append((range_val, angle))
                    if range_val < min_distance:
                        min_distance = range_val
                        min_angle = angle

        # Update nearest obstacle information
        if front_obstacles:
            self.nearest_obstacle = (min_distance, min_angle)
            self.get_logger().info(f'Nearest obstacle: distance={min_distance:.2f}m, angle={min_angle:.2f}rad')
        else:
            self.nearest_obstacle = None
            self.get_logger().debug('No obstacles detected in front sector')

    def planning_timer_callback(self):
        """Periodic path planning."""
        if self.has_taken_off and self.obstacle_list:
            self.plan_path()

    def plan_path(self):
        """Generate path using RRT*."""
        if not self.has_taken_off:
            return

        start = (self.vehicle_local_position.x, self.vehicle_local_position.y)
        goal = (self.target_position[0], self.target_position[1])

        # Check if we're close enough to goal
        dist_to_goal = sqrt((start[0] - goal[0])**2 + (start[1] - goal[1])**2)
        if dist_to_goal < 0.5:
            return

        # Local planning (RRT*)
        rrt_star = RRTStar(
            start=start,
            goal=goal,
            obstacle_list=self.obstacle_list,
            rand_area=[min(start[0], goal[0]) - 5, max(start[0], goal[0]) + 5],
            max_iter=100  # Reduced for performance
        )
        new_path = rrt_star.plan()
        if new_path:
            self.current_path = new_path
            self.current_path_index = 0
            # Log the new path
            path_str = " -> ".join([f"({x:.1f}, {y:.1f})" for x, y in self.current_path])
            self.get_logger().info(f'Local path planned: {path_str}')
        else:
            self.get_logger().warn("Local planning failed")

    def check_path_safety(self, target_x: float, target_y: float) -> bool:
        """
        Check if the path to the target point is safe from obstacles.
        """
        if not self.nearest_obstacle:
            return True  # No obstacles detected, path is safe

        # Vector from current position to target
        path_vector = np.array([
            target_x - self.vehicle_local_position.x,
            target_y - self.vehicle_local_position.y
        ])
        path_length = np.linalg.norm(path_vector)
        
        if path_length == 0:
            return True  # Target is the current position, path is safe

        # Normalize path vector
        path_direction = path_vector / path_length

        # Check along the path for obstacles
        check_points = np.linspace(0, path_length, num=10)  # Sample 10 points along the path
        for distance in check_points:
            check_point = np.array([
                self.vehicle_local_position.x + path_direction[0] * distance,
                self.vehicle_local_position.y + path_direction[1] * distance
            ])
            
            # Check if point is too close to any obstacle
            for obstacle in self.obstacle_list:
                obstacle_dist = sqrt(
                    (check_point[0] - obstacle[0])**2 +
                    (check_point[1] - obstacle[1])**2
                )
                if obstacle_dist < self.obstacle_safe_distance:
                    return False  # Path is unsafe

        return True  # Path is safe

    def publish_velocity_position_setpoint(self, x: float, y: float, z: float):
        """Publish position setpoint with velocity limit."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]

        # During takeoff, maintain current yaw to avoid rotation
        if not self.has_taken_off:
            msg.yaw = self.vehicle_local_position.heading  # Use current yaw
        else:
            msg.yaw = self.get_yaw_to_target(x, y)  # Face the target after takeoff

        # Calculate velocity vector
        dx = x - self.vehicle_local_position.x
        dy = y - self.vehicle_local_position.y
        distance = sqrt(dx*dx + dy*dy)

        if distance > 0:
            # Normalize and scale by desired velocity
            msg.velocity = [
                (dx/distance) * self.velocity_setpoint,
                (dy/distance) * self.velocity_setpoint,
                0.0
            ]
        else:
            msg.velocity = [0.0, 0.0, 0.0]

        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Target position: {[x, y, z]}, velocity: {msg.velocity}")

    def get_yaw_to_target(self, target_x, target_y):
        """Calculate yaw angle to face the target."""
        dx = target_x - self.vehicle_local_position.x
        dy = target_y - self.vehicle_local_position.y
        return atan2(dy, dx)

    def vehicle_local_position_callback(self, vehicle_local_position):
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        self.vehicle_status = vehicle_status

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def engage_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
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

    def publish_vehicle_command(self, command, **params) -> None:
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

    def timer_callback(self) -> None:
        self.publish_offboard_control_heartbeat_signal()

        # Initial setup
        if self.offboard_setpoint_counter == 5:
            self.set_com_of_loss_t(5.0)  # Set COM_OF_LOSS_T to 5 seconds
        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()

        # Takeoff phase
        if not self.has_taken_off and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.publish_velocity_position_setpoint(0.0, 0.0, self.takeoff_height)
            if abs(self.vehicle_local_position.z - self.takeoff_height) < 0.5:
                self.has_taken_off = True
                self.get_logger().info('Takeoff complete, starting mission')

        # Navigation phase
        elif self.has_taken_off:
            current_time = self.get_clock().now().seconds_nanoseconds()[0]

            # Check if we've reached the final target
            dist_to_final = sqrt(
                (self.vehicle_local_position.x - self.target_position[0])**2 +
                (self.vehicle_local_position.y - self.target_position[1])**2
            )
            if dist_to_final < 0.5:
                self.get_logger().info('Reached final target')
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                return

            # Normal navigation with path following
            if not self.current_path:
                self.plan_path()
                self.publish_velocity_position_setpoint(
                    self.vehicle_local_position.x,
                    self.vehicle_local_position.y,
                    self.takeoff_height
                )
            elif self.current_path_index < len(self.current_path):
                current_target = self.current_path[self.current_path_index]

                # Check if path to next waypoint is safe
                if self.check_path_safety(current_target[0], current_target[1]):
                    dist_to_waypoint = sqrt(
                        (self.vehicle_local_position.x - current_target[0])**2 +
                        (self.vehicle_local_position.y - current_target[1])**2
                    )

                    if dist_to_waypoint < self.waypoint_threshold:
                        self.current_path_index += 1
                        self.get_logger().info(f'Reached waypoint {self.current_path_index}')
                        
                        # Log current path progress
                        if self.current_path_index < len(self.current_path):
                            remaining_path = " -> ".join([f"({x:.1f}, {y:.1f})" for x, y in self.current_path[self.current_path_index:]])
                            self.get_logger().info(f'Remaining path: {remaining_path}')
                    else:
                        self.publish_velocity_position_setpoint(
                            current_target[0],
                            current_target[1],
                            self.takeoff_height
                        )
                else:
                    # Path is not safe, trigger replanning
                    self.get_logger().warn('Current path is not safe, triggering replanning')
                    self.current_path = None
                    self.current_path_index = 0
            else:
                # If we've run out of waypoints but haven't reached the target, replan
                self.current_path = None
                self.current_path_index = 0

        # Increment counter for initialization
        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

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