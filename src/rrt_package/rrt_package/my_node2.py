#!/usr/bin/env python3

import rclpy
import numpy as np
import math
import heapq
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from typing import List, Tuple, Dict, Set, Optional, Any


class DStarLite:
    """D* Lite algorithm implementation for path planning with dynamic obstacle detection."""

    def __init__(self, grid_resolution: float = 0.5):
        self.grid_resolution = grid_resolution
        self.obstacles: Set[Tuple[int, int]] = set()
        self.start: Optional[Tuple[int, int]] = None
        self.goal: Optional[Tuple[int, int]] = None
        self.g_values: Dict[Tuple[int, int], float] = {}
        self.rhs_values: Dict[Tuple[int, int], float] = {}
        self.km = 0  # Constant to handle replanning
        self.open_set: List[Tuple[float, float, Tuple[int, int]]] = []
        
        # Grid boundaries
        self.x_min, self.x_max = -20, 20
        self.y_min, self.y_max = -20, 20
        self.directions = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)]

    def world_to_grid(self, x, y) -> Tuple[int, int]:
        """Convert world coordinates to grid coordinates."""
        return (int(round(x / self.grid_resolution)), 
                int(round(y / self.grid_resolution)))

    def grid_to_world(self, grid_x, grid_y) -> Tuple[float, float]:
        """Convert grid coordinates to world coordinates."""
        return (grid_x * self.grid_resolution, grid_y * self.grid_resolution)

    def calculate_key(self, s: Tuple[int, int]) -> Tuple[float, float]:
        """Calculate the key for a vertex."""
        if s not in self.g_values:
            self.g_values[s] = float('inf')
        if s not in self.rhs_values:
            self.rhs_values[s] = float('inf')
        
        k1 = min(self.g_values[s], self.rhs_values[s]) + self.heuristic(s, self.goal) + self.km
        k2 = min(self.g_values[s], self.rhs_values[s])
        return (k1, k2)

    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Calculate the heuristic (Euclidean distance) between two points."""
        if a is None or b is None:
            return float('inf')
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def update_vertex(self, u: Tuple[int, int]) -> None:
        """Update a vertex in the search."""
        if u != self.goal:
            self.rhs_values[u] = min([self.cost(u, s) + self.g_values.get(s, float('inf')) 
                                   for s in self.get_neighbors(u)])
        
        if u in [item[2] for item in self.open_set]:
            # Remove from open set if present
            self.open_set = [item for item in self.open_set if item[2] != u]
            heapq.heapify(self.open_set)
        
        if self.g_values.get(u, float('inf')) != self.rhs_values.get(u, float('inf')):
            heapq.heappush(self.open_set, (*self.calculate_key(u), u))

    def get_neighbors(self, s: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Get valid neighbors of a grid cell."""
        neighbors = []
        for dx, dy in self.directions:
            nx, ny = s[0] + dx, s[1] + dy
            if (self.x_min <= nx <= self.x_max and 
                self.y_min <= ny <= self.y_max and 
                (nx, ny) not in self.obstacles):
                neighbors.append((nx, ny))
        return neighbors

    def cost(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Calculate the cost between two adjacent cells."""
        if b in self.obstacles:
            return float('inf')
        return self.heuristic(a, b)

    def compute_shortest_path(self) -> None:
        """Compute the shortest path using D* Lite."""
        if not self.open_set:
            return
        
        while (self.open_set and 
               (heapq.nsmallest(1, self.open_set)[0][:2] < self.calculate_key(self.start) or 
                self.rhs_values.get(self.start, float('inf')) != self.g_values.get(self.start, float('inf')))):
            
            if not self.open_set:  # Safety check
                break
                
            k_old = heapq.nsmallest(1, self.open_set)[0][:2]
            u = heapq.heappop(self.open_set)[2]
            k_new = self.calculate_key(u)
            
            if k_old < k_new:
                heapq.heappush(self.open_set, (*k_new, u))
            elif self.g_values.get(u, float('inf')) > self.rhs_values.get(u, float('inf')):
                self.g_values[u] = self.rhs_values[u]
                for s in self.get_neighbors(u):
                    self.update_vertex(s)
            else:
                self.g_values[u] = float('inf')
                for s in self.get_neighbors(u) + [u]:
                    self.update_vertex(s)

    def initialize(self, start: Tuple[float, float], goal: Tuple[float, float]) -> None:
        """Initialize the D* Lite search."""
        self.start = self.world_to_grid(*start[:2])
        self.goal = self.world_to_grid(*goal[:2])
        
        self.g_values = {self.goal: float('inf')}
        self.rhs_values = {self.goal: 0}
        
        self.open_set = []
        heapq.heappush(self.open_set, (*self.calculate_key(self.goal), self.goal))
        self.compute_shortest_path()

    def update_obstacles(self, obstacles: List[np.ndarray], height: float, radius: float = 1.0) -> bool:
        """Update the obstacle grid based on point cloud data."""
        old_obstacles = self.obstacles.copy()
        new_obstacles = set()
        
        for obs in obstacles:
            # Only consider obstacles near the flight height
            if abs(obs[2] - height) <= 1.0:
                grid_pos = self.world_to_grid(obs[0], obs[1])
                
                # Add the obstacle plus a buffer zone
                for dx in range(-int(radius/self.grid_resolution), int(radius/self.grid_resolution)+1):
                    for dy in range(-int(radius/self.grid_resolution), int(radius/self.grid_resolution)+1):
                        if dx*dx + dy*dy <= (radius/self.grid_resolution)**2:
                            new_obstacles.add((grid_pos[0] + dx, grid_pos[1] + dy))
        
        # Check if obstacles have changed
        if new_obstacles != self.obstacles:
            self.obstacles = new_obstacles
            # Reset path if obstacles changed
            if self.start and self.goal:
                # Update km to maintain consistency in replanning
                self.km += self.heuristic(self.start, self.goal)
                # Update vertices affected by obstacle changes
                for obs in self.obstacles - old_obstacles:  # New obstacles
                    self.update_vertex(obs)
                    for s in self.get_neighbors(obs):
                        self.update_vertex(s)
                for obs in old_obstacles - self.obstacles:  # Removed obstacles
                    self.update_vertex(obs)
                    for s in self.get_neighbors(obs):
                        self.update_vertex(s)
                self.compute_shortest_path()
            return True
        return False

    def get_next_pos(self, current_pos: Tuple[float, float]) -> Optional[Tuple[float, float]]:
        """Get the next position along the path from the current position."""
        if not self.start or not self.goal:
            return None
        
        curr_grid = self.world_to_grid(*current_pos[:2])
        if curr_grid == self.goal:
            return self.grid_to_world(*self.goal)
            
        # Find the neighbor with minimum cost
        neighbors = self.get_neighbors(curr_grid)
        if not neighbors:
            return None
            
        best_next = min(neighbors, key=lambda s: self.g_values.get(s, float('inf')))
        return self.grid_to_world(*best_next)

    def get_path(self, current_pos: Tuple[float, float]) -> List[Tuple[float, float]]:
        """Get the complete path from current position to goal."""
        if not self.start or not self.goal:
            return []
            
        path = []
        curr = self.world_to_grid(*current_pos[:2])
        
        while curr != self.goal:
            path.append(self.grid_to_world(*curr))
            neighbors = self.get_neighbors(curr)
            if not neighbors:
                break
                
            curr = min(neighbors, key=lambda s: self.g_values.get(s, float('inf')))
            # Avoid infinite loop
            if curr in [self.world_to_grid(*p) for p in path]:
                break
                
        path.append(self.grid_to_world(*self.goal))
        return path


class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode with D* Lite path planning and obstacle avoidance."""

    def __init__(self) -> None:
        super().__init__('offboard_control_dstar_obstacle_avoidance')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.pointcloud_subscriber = self.create_subscription(
            PointCloud2, '/world/iris_maze/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan/points', self.pointcloud_callback, 10)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -5.0
        self.target_position = [7.0, 0.0, self.takeoff_height]
        self.point_cloud_data = None
        self.safety_distance = 1.0
        self.waypoint_threshold = 0.5
        self.max_velocity = 0.5  # Maximum velocity in m/s
        self.current_velocity = [0.0, 0.0, 0.0]
        
        # Flight states
        self.TAKEOFF = 0
        self.ALIGN = 1
        self.NAVIGATE = 2
        self.HOLD = 3
        self.LAND = 4
        self.flight_state = self.TAKEOFF
        
        # Initialize D* Lite path planner
        self.dstar = DStarLite(grid_resolution=0.5)
        self.path: List[Tuple[float, float]] = []
        self.current_waypoint = None
        self.path_initialized = False
        self.hold_start_time = None

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def pointcloud_callback(self, msg):
        """Callback function for point cloud data."""
        self.point_cloud_data = msg

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
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,  # Custom mode
            param2=6.0   # Offboard mode
        )
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def process_point_cloud(self) -> List[np.ndarray]:
        """Process point cloud data to get obstacle positions."""
        if self.point_cloud_data is None:
            return []

        obstacles = []
        try:
            for point in pc2.read_points(self.point_cloud_data, 
                                       field_names=("x", "y", "z"),
                                       skip_nans=True):
                # Filter points within the drone's operational height range
                obstacles.append(np.array([point[0], point[1], point[2]]))
        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {str(e)}')
            return []
        
        return obstacles

    def calculate_yaw_to_target(self, current_pos: List[float], target_pos: List[float]) -> float:
        """Calculate the yaw angle to face the target position."""
        dx = target_pos[0] - current_pos[0]
        dy = target_pos[1] - current_pos[1]
        return math.atan2(dy, dx)

    def is_aligned_to_target(self, current_yaw: float, target_yaw: float, threshold: float = 0.1) -> bool:
        """Check if the current yaw is aligned with the target yaw within a threshold."""
        # Normalize angles
        def normalize_angle(angle):
            while angle > math.pi:
                angle -= 2 * math.pi
            while angle < -math.pi:
                angle += 2 * math.pi
            return angle
        
        diff = abs(normalize_angle(current_yaw - target_yaw))
        return diff < threshold

    def is_at_position(self, current_pos: List[float], target_pos: List[float]) -> bool:
        """Check if vehicle has reached the target position."""
        return np.linalg.norm(np.array(current_pos) - np.array(target_pos)) < self.waypoint_threshold

    def is_at_height(self, current_height: float, target_height: float, threshold: float = 0.2) -> bool:
        """Check if vehicle has reached the target height."""
        return abs(current_height - target_height) < threshold

    def update_path_planning(self) -> None:
        """Update the path planning based on current position and obstacles."""
        current_pos = [self.vehicle_local_position.x, self.vehicle_local_position.y, self.vehicle_local_position.z]
        obstacles = self.process_point_cloud()
        
        if not self.path_initialized:
            self.dstar.initialize(current_pos, self.target_position)
            self.path_initialized = True
            self.get_logger().info("D* Lite initialized")
        
        # Update obstacles and replan if necessary
        if self.dstar.update_obstacles(obstacles, self.takeoff_height, self.safety_distance):
            self.path = self.dstar.get_path(current_pos)
            if self.path:
                self.get_logger().info(f"Replanned path with {len(self.path)} waypoints")
            else:
                self.get_logger().warning("No valid path found during replanning")
        
        # Get next waypoint if needed
        if not self.current_waypoint or self.is_at_position(current_pos, self.current_waypoint):
            if self.path and len(self.path) > 0:
                self.current_waypoint = list(self.path[0]) + [self.takeoff_height]
                self.path = self.path[1:]
                self.get_logger().info(f"New waypoint: {self.current_waypoint}")
            elif not self.is_at_position(current_pos, self.target_position):
                # If no path available, move directly toward target
                self.current_waypoint = self.target_position
                self.get_logger().info(f"No waypoints available, heading directly to target: {self.target_position}")
            else:
                # At target position
                self.current_waypoint = self.target_position

    def calculate_velocity(self, current_pos: List[float], target_pos: List[float]) -> List[float]:
        """Calculate velocity vector towards the target position."""
        direction = np.array(target_pos) - np.array(current_pos)
        distance = np.linalg.norm(direction)
        
        if distance > 0:
            direction = direction / distance
            # Scale velocity based on distance to target
            velocity_scale = min(self.max_velocity, max(0.1, distance / 2.0))
            velocity = direction * velocity_scale
            return list(velocity)
        else:
            return [0.0, 0.0, 0.0]

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = True  # Enable velocity control
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, position: List[float], yaw: float = 1.57079):
        """Publish position setpoint without velocity."""
        msg = TrajectorySetpoint()
        msg.position = position
        msg.velocity = [0.0, 0.0, 0.0]
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Position setpoint: {position}, Yaw: {yaw}")

    def publish_position_and_velocity_setpoint(self, position: List[float], velocity: List[float], yaw: float = 1.57079):
        """Publish position and velocity setpoint."""
        msg = TrajectorySetpoint()
        msg.position = position
        msg.velocity = velocity
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Position: {position}, Velocity: {velocity}, Yaw: {yaw}")

    def publish_vehicle_command(self, command, **params) -> None:
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

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        try:
            self.publish_offboard_control_heartbeat_signal()

            if self.offboard_setpoint_counter == 10:
                self.engage_offboard_mode()
                self.arm()

            if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                current_pos = [self.vehicle_local_position.x, 
                              self.vehicle_local_position.y, 
                              self.vehicle_local_position.z]
                
                # State machine for flight phases
                if self.flight_state == self.TAKEOFF:
                    # Take off to specified height
                    takeoff_setpoint = [0.0, 0.0, self.takeoff_height]
                    self.publish_position_setpoint(takeoff_setpoint)
                    
                    if self.is_at_height(current_pos[2], self.takeoff_height):
                        self.get_logger().info("Takeoff complete, aligning to target")
                        self.flight_state = self.ALIGN
                
                elif self.flight_state == self.ALIGN:
                    # Calculate yaw to face target
                    target_yaw = self.calculate_yaw_to_target(current_pos, self.target_position)
                    current_pos_at_takeoff_height = [current_pos[0], current_pos[1], self.takeoff_height]
                    self.publish_position_setpoint(current_pos_at_takeoff_height, target_yaw)
                    
                    if self.is_aligned_to_target(self.vehicle_local_position.heading, target_yaw):
                        self.get_logger().info("Aligned to target, starting navigation")
                        self.flight_state = self.NAVIGATE
                        # Initialize path planning
                        self.update_path_planning()
                
                elif self.flight_state == self.NAVIGATE:
                    # Update path planning
                    self.update_path_planning()
                    
                    # Ensure we have a current waypoint
                    if self.current_waypoint is None:
                        self.current_waypoint = self.target_position
                    
                    # Calculate velocity
                    velocity = self.calculate_velocity(current_pos, self.current_waypoint)
                    target_yaw = self.calculate_yaw_to_target(current_pos, self.current_waypoint)
                    
                    # Publish setpoint
                    self.publish_position_and_velocity_setpoint(
                        self.current_waypoint,
                        velocity,
                        target_yaw
                    )
                    
                    # Check if reached final target
                    if self.is_at_position(current_pos, self.target_position):
                        self.get_logger().info("Reached target position, holding position")
                        self.flight_state = self.HOLD
                        self.hold_start_time = self.get_clock().now()
                
                elif self.flight_state == self.HOLD:
                    # Hold position for 5 seconds before landing
                    self.publish_position_setpoint(self.target_position)
                    
                    if self.hold_start_time is None:
                        self.hold_start_time = self.get_clock().now()
                    
                    elapsed_time = (self.get_clock().now() - self.hold_start_time).nanoseconds / 1e9
                    if elapsed_time > 5.0:
                        self.get_logger().info("Hold complete, initiating landing")
                        self.flight_state = self.LAND
                
                elif self.flight_state == self.LAND:
                    self.land()
                    self.get_logger().info("Landing command sent")

            if self.offboard_setpoint_counter < 11:
                self.offboard_setpoint_counter += 1

        except Exception as e:
            self.get_logger().error(f'Error in timer callback: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())

def main(args=None) -> None:
    print('Starting offboard control node with D* Lite path planning and lidar obstacle avoidance...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)