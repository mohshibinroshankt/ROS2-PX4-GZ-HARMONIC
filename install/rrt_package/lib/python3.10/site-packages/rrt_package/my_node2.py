#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, BatteryStatus, WindEstimation
from sensor_msgs.msg import LaserScan
import numpy as np
from math import sqrt, cos, sin, pi, atan2
import time
from collections import deque
from scipy.interpolate import splprep, splev

class PathOptimizer:
    def __init__(self):
        self.min_distance = 0.5
        self.smoothing_factor = 0.1
        
    def smooth_path(self, path):
        """Smooth path using B-spline interpolation"""
        if len(path) < 4:
            return path
            
        path_array = np.array(path)
        tck, u = splprep([path_array[:,0], path_array[:,1], path_array[:,2]], s=self.smoothing_factor)
        u_new = np.linspace(0, 1, len(path))
        return list(zip(*splev(u_new, tck)))
        
    def simplify_path(self, path):
        """Remove redundant waypoints"""
        if len(path) < 3:
            return path
            
        simplified = [path[0]]
        for i in range(1, len(path)-1):
            prev = np.array(simplified[-1])
            curr = np.array(path[i])
            next_point = np.array(path[i+1])
            
            # Check if current point is necessary
            if np.linalg.norm(curr - prev) < self.min_distance or \
               np.abs(np.cross(next_point - prev, curr - prev)).max() < self.min_distance:
                continue
            simplified.append(path[i])
        simplified.append(path[-1])
        return simplified

class VelocityPlanner:
    def __init__(self):
        self.max_velocity = 2.0
        self.max_acceleration = 1.0
        self.slowdown_distance = 2.0
        
    def compute_velocity_profile(self, path, current_velocity):
        """Generate velocity profile for the path"""
        velocities = []
        prev_velocity = np.linalg.norm(current_velocity)
        
        for i in range(len(path)-1):
            distance = np.linalg.norm(np.array(path[i+1]) - np.array(path[i]))
            
            # Check if approaching goal
            distance_to_goal = np.linalg.norm(np.array(path[-1]) - np.array(path[i]))
            if distance_to_goal < self.slowdown_distance:
                target_velocity = self.max_velocity * (distance_to_goal / self.slowdown_distance)
            else:
                target_velocity = self.max_velocity
            
            # Apply acceleration constraints
            achievable_velocity = sqrt(prev_velocity**2 + 2*self.max_acceleration*distance)
            velocity = min(target_velocity, achievable_velocity)
            
            velocities.append(velocity)
            prev_velocity = velocity
            
        velocities.append(0.0)  # Zero velocity at goal
        return velocities

class AdvancedDynamicPlanner(Node):
    def __init__(self):
        super().__init__('advanced_dynamic_planner')
        
        # Initialize components
        self.path_optimizer = PathOptimizer()
        self.velocity_planner = VelocityPlanner()
        
        # Enhanced parameters
        self._init_parameters()
        
        # Initialize publishers and subscribers
        self._init_communication()
        
        # Initialize state variables
        self._init_state()
        
        # Initialize safety monitoring
        self._init_safety_monitoring()
        
        # Create timers
        self._init_timers()

    def _init_parameters(self):
        """Initialize enhanced parameters"""
        # Planning parameters
        self.rrt_step_size = 1.0
        self.rrt_connect_attempts = 20
        self.max_planning_time = 0.2
        self.replanning_threshold = 0.5
        
        # Safety parameters
        self.critical_battery_level = 20.0
        self.max_wind_speed = 8.0
        self.emergency_land_height = 2.0
        self.obstacle_inflation_radius = 0.3
        
        # Mission parameters
        self.takeoff_height = -5.0
        self.goal_position = [7.0, 0.0, self.takeoff_height]
        self.position_threshold = 0.3

    def _init_safety_monitoring(self):
        """Initialize safety monitoring systems"""
        self.battery_level = 100.0
        self.wind_speed = 0.0
        self.last_obstacle_check = time.time()
        self.emergency_count = 0
        self.max_emergency_count = 3
        
    def _init_state(self):
        """Initialize state variables"""
        self.current_position = np.zeros(3)
        self.current_velocity = np.zeros(3)
        self.local_path = deque(maxlen=20)
        self.velocity_profile = []
        self.state = "INIT"
        self.is_armed = False
        self.obstacle_map = {}
        self.wind_estimation = np.zeros(3)

    def _init_communication(self):
        """Initialize enhanced communication"""
        # QoS Profile
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Additional subscribers
        self.create_subscription(
            BatteryStatus, '/fmu/out/battery_status',
            self.battery_callback, self.qos_profile)
        self.create_subscription(
            WindEstimation, '/fmu/out/wind_estimation',
            self.wind_callback, self.qos_profile)
            
        # Existing subscribers
        self._init_base_subscribers()
        
        # Publishers
        self._init_publishers()

    def bidirectional_rrt(self, start, goal):
        """Enhanced RRT-Connect algorithm"""
        start_tree = [(np.array(start), None)]
        goal_tree = [(np.array(goal), None)]
        
        for _ in range(self.rrt_connect_attempts):
            # Grow both trees
            if len(start_tree) < len(goal_tree):
                new_node = self._extend_tree(start_tree, goal_tree)
                if new_node is not None:
                    start_tree.append(new_node)
            else:
                new_node = self._extend_tree(goal_tree, start_tree)
                if new_node is not None:
                    goal_tree.append(new_node)
            
            # Check for connection
            if self._try_connect_trees(start_tree, goal_tree):
                return self._extract_path(start_tree, goal_tree)
        
        return None

    def _extend_tree(self, tree, target_tree):
        """Extend one tree towards the other"""
        if np.random.random() < 0.2:
            sample = target_tree[-1][0]  # Bias towards other tree
        else:
            sample = self._generate_random_sample()
            
        nearest_idx = self._find_nearest(tree, sample)
        new_point = self._steer(tree[nearest_idx][0], sample)
        
        if new_point is not None and self._is_safe(new_point):
            return (new_point, nearest_idx)
        return None

    def _try_connect_trees(self, tree1, tree2):
        """Try to connect two RRT trees"""
        nearest1 = tree1[-1][0]
        nearest2_idx = self._find_nearest(tree2, nearest1)
        nearest2 = tree2[nearest2_idx][0]
        
        if np.linalg.norm(nearest1 - nearest2) < self.rrt_step_size:
            return self._is_path_safe(nearest1, nearest2)
        return False

    def _generate_random_sample(self):
        """Generate random sample with dynamic bounds"""
        bounds_min = self.current_position - 10
        bounds_max = self.current_position + 10
        bounds_max[2] = self.takeoff_height + 2  # Limit altitude variation
        
        return np.random.uniform(bounds_min, bounds_max)

    def _is_safe(self, point):
        """Check if point is safe with enhanced criteria"""
        # Check basic obstacle clearance
        if not super()._is_safe(point):
            return False
            
        # Check height bounds
        if abs(point[2] - self.takeoff_height) > 2.0:
            return False
            
        # Check wind conditions at point
        if np.linalg.norm(self.wind_estimation) > self.max_wind_speed:
            return False
            
        return True

    def update_safety_status(self):
        """Update safety status and handle emergencies"""
        emergency_triggered = False
        
        # Check battery level
        if self.battery_level < self.critical_battery_level:
            self.get_logger().warn(f"Critical battery level: {self.battery_level}%")
            emergency_triggered = True
            
        # Check wind conditions
        wind_speed = np.linalg.norm(self.wind_estimation)
        if wind_speed > self.max_wind_speed:
            self.get_logger().warn(f"High wind speed: {wind_speed} m/s")
            emergency_triggered = True
            
        # Check obstacle proximity
        min_obstacle_dist = float('inf')
        for obs_pos in self.obstacle_map.values():
            dist = np.linalg.norm(np.array(obs_pos) - self.current_position)
            min_obstacle_dist = min(min_obstacle_dist, dist)
            
        if min_obstacle_dist < self.emergency_land_height:
            self.get_logger().warn(f"Obstacle too close: {min_obstacle_dist}m")
            emergency_triggered = True
            
        if emergency_triggered:
            self.handle_emergency()

    def handle_emergency(self):
        """Enhanced emergency handling"""
        self.emergency_count += 1
        
        if self.emergency_count >= self.max_emergency_count:
            self.get_logger().error("Multiple emergencies detected. Initiating emergency landing.")
            self.state = "EMERGENCY_LAND"
            return
            
        # Try to find safe direction
        safe_point = self.find_safe_direction()
        if safe_point is not None:
            self.local_path.clear()
            self.local_path.append(safe_point)
            self.state = "EMERGENCY_AVOID"
        else:
            self.state = "EMERGENCY_LAND"

    def find_safe_direction(self):
        """Find safe direction for emergency maneuver"""
        best_direction = None
        max_clearance = 0
        
        for angle in np.linspace(0, 2*pi, 16):
            direction = np.array([cos(angle), sin(angle), 0])
            test_point = self.current_position + direction * 2.0
            
            clearance = float('inf')
            for obs_pos in self.obstacle_map.values():
                dist = np.linalg.norm(np.array(obs_pos) - test_point)
                clearance = min(clearance, dist)
                
            if clearance > max_clearance:
                max_clearance = clearance
                best_direction = test_point
                
        return best_direction if max_clearance > self.emergency_land_height else None

    def battery_callback(self, msg):
        """Handle battery status updates"""
        self.battery_level = msg.remaining
        
    def wind_callback(self, msg):
        """Handle wind estimation updates"""
        self.wind_estimation = np.array([msg.windspeed_north, 
                                       msg.windspeed_east,
                                       msg.windspeed_down])

    def control_loop(self):
        """Enhanced main control loop"""
        # Update safety status
        self.update_safety_status()
        
        # Handle different states
        if self.state == "INIT":
            if self.offboard_setpoint_counter >= 10:
                self.engage_offboard_mode()
                self.arm()
                self.state = "TAKEOFF"
        
        elif self.state == "TAKEOFF":
            self.handle_takeoff()
            
        elif self.state == "NAVIGATE":
            self.handle_navigation()
            
        elif self.state == "EMERGENCY_AVOID":
            self.handle_emergency_avoidance()
            
        elif self.state == "EMERGENCY_LAND":
            self.handle_emergency_landing()
            
        # Publish control mode
        self.publish_offboard_control_mode()

    def handle_takeoff(self):
        """Handle takeoff phase"""
        if abs(self.current_position[2] - self.takeoff_height) < self.position_threshold:
            self.state = "NAVIGATE"
            self.local_path = deque([self.current_position])
        else:
            target = self.current_position.copy()
            target[2] = self.takeoff_height
            self.publish_setpoint(*target)

    def handle_navigation(self):
        """Handle normal navigation"""
        if not self.local_path:
            success = self.replan_path()
            if not success:
                self.handle_emergency()
            return
            
        target = self.local_path[0]
        if np.linalg.norm(self.current_position - np.array(target)) < self.position_threshold:
            self.local_path.popleft()
            
        if self.local_path:
            velocity = self.velocity_profile[0] if self.velocity_profile else 1.0
            self.publish_position_setpoint_with_velocity(self.local_path[0], velocity)

    def handle_emergency_avoidance(self):
        """Handle emergency avoidance"""
        if not self.local_path:
            if self.emergency_count < self.max_emergency_count:
                self.state = "NAVIGATE"
            else:
                self.state = "EMERGENCY_LAND"
            return
            
        target = self.local_path[0]
        if np.linalg.norm(self.current_position - np.array(target)) < self.position_threshold:
            self.local_path.popleft()
            
        if self.local_path:
            self.publish_setpoint(*self.local_path[0], velocity=0.5)  # Slower during emergency

    def handle_emergency_landing(self):
        """Handle emergency landing"""
        if abs(self.current_position[2]) < 0.1:
            self.state = "COMPLETE"
            self.disarm

    