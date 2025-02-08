#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from px4_msgs.msg import VehicleLocalPosition, OffboardControlMode, TrajectorySetpoint
import numpy as np
import random
from math import sqrt, atan2

class Node3D:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        self.parent = None

class PathPlanningNode(Node):
    def __init__(self):
        super().__init__('path_planning_node')
        
        # Configure QoS profile to match PX4's settings
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        
        # Subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position', 
            self.vehicle_local_position_callback,
            qos_profile)
        
        # Publishers for PX4 control
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            qos_profile)
            
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            qos_profile)

        # Parameters
        self.rrt_step_size = 0.10
        self.min_distance_to_obstacle = 1.5
        self.target_position = [5.0, 7.0, 0.1]  # Target position [x, y, z]
        self.obstacles = [[3.0, 3.0, 1.0]]  # Example obstacle
        
        # Create timer for publishing control modes
        self.create_timer(0.1, self.publish_control_mode)  # 10Hz timer
        
        self.current_path = None
        self.current_path_index = 0

    def vehicle_local_position_callback(self, vehicle_local_position):
        if not vehicle_local_position.xy_valid:
            self.get_logger().warn('Position data not valid')
            return

        start_pos = [vehicle_local_position.x, vehicle_local_position.y, vehicle_local_position.z]
        
        try:
            # Only plan new path if we don't have one or reached the end of current path
            if self.current_path is None:
                path = self.plan_rrt_path(start_pos, self.target_position)
                if path:
                    self.current_path = path
                    self.current_path_index = 0
                    self.get_logger().info('New path planned')
                else:
                    self.get_logger().warn('No valid path found')
            
            # Publish next setpoint in path
            if self.current_path:
                self.publish_setpoint(self.current_path[self.current_path_index])
                
                # Check if close to current waypoint
                current_pos = np.array(start_pos)
                waypoint = np.array(self.current_path[self.current_path_index])
                if np.linalg.norm(current_pos - waypoint) < 0.2:  # Within 20cm
                    self.current_path_index += 1
                    if self.current_path_index >= len(self.current_path):
                        self.current_path = None  # Path completed
                        self.get_logger().info('Path completed')
                
        except Exception as e:
            self.get_logger().error(f'Path planning failed: {str(e)}')

    def publish_control_mode(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_control_mode_publisher.publish(msg)

    def publish_setpoint(self, position):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [position[0], position[1], position[2]]
        msg.yaw = 0.0  # Default yaw
        self.trajectory_setpoint_publisher.publish(msg)

    # ... [Rest of the RRT planning methods remain the same] ...
    def plan_rrt_path(self, start_pos, goal_pos):
        nodes = [Node3D(start_pos[0], start_pos[1], start_pos[2])]
        goal_node = Node3D(goal_pos[0], goal_pos[1], goal_pos[2])
        
        for _ in range(5000):
            if random.random() < 0.2:
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
        for _ in range(100):
            x = random.uniform(min(start_pos[0], goal_pos[0]) - 5,
                             max(start_pos[0], goal_pos[0]) + 5)
            y = random.uniform(min(start_pos[1], goal_pos[1]) - 5,
                             max(start_pos[1], goal_pos[1]) + 5)
            z = random.uniform(min(start_pos[2], goal_pos[2]) - 1,
                             max(start_pos[2], goal_pos[2]) + 1)
            
            valid = True
            for obstacle in self.obstacles:
                dist = sqrt((x - obstacle[0])**2 + (y - obstacle[1])**2 + (z - obstacle[2])**2)
                if dist < self.min_distance_to_obstacle:
                    valid = False
                    break
            
            if valid:
                return Node3D(x, y, z)
        
        return None

    def get_nearest_node(self, nodes, target):
        if not nodes:
            return None
        return min(nodes, key=lambda n: self.distance(n, target))

    def steer(self, from_node, to_node):
        if not from_node or not to_node:
            return None
            
        dist = self.distance(from_node, to_node)
        if dist < self.rrt_step_size:
            return to_node
            
        theta = atan2(to_node.y - from_node.y, to_node.x - from_node.x)
        phi = atan2(to_node.z - from_node.z, sqrt((to_node.x - from_node.x)**2 + (to_node.y - from_node.y)**2))
        
        new_x = from_node.x + self.rrt_step_size * np.cos(theta) * np.cos(phi)
        new_y = from_node.y + self.rrt_step_size * np.sin(theta) * np.cos(phi)
        new_z = from_node.z + self.rrt_step_size * np.sin(phi)
        
        return Node3D(new_x, new_y, new_z)

    def extract_path(self, node):
        if not node:
            return None
            
        path = []
        current = node
        while current is not None:
            path.append([current.x, current.y, current.z])
            current = current.parent
        return path[::-1]

    def is_collision_free(self, node1, node2):
        if not node1 or not node2:
            return False
            
        num_checks = 50
        
        for i in range(num_checks + 1):
            t = i / num_checks
            x = node1.x + t * (node2.x - node1.x)
            y = node1.y + t * (node2.y - node1.y)
            z = node1.z + t * (node2.z - node1.z)
            
            for obstacle in self.obstacles:
                dist = sqrt((x - obstacle[0])**2 + (y - obstacle[1])**2 + (z - obstacle[2])**2)
                if dist < self.min_distance_to_obstacle:
                    return False
        return True

    def distance(self, node1, node2):
        if not node1 or not node2:
            return float('inf')
        return sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2 + (node1.z - node2.z)**2)

def main(args=None):
    rclpy.init(args=args)
    try:
        node = PathPlanningNode()
        rclpy.spin(node)
    except Exception as e:
        print(f'Error in main: {str(e)}')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()