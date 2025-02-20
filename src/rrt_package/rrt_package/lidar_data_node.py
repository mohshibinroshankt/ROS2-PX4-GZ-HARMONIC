import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')
        self.subscription = self.create_subscription(
            LaserScan,
            '/world/iris_maze/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan',
            self.lidar_callback,
            10)
        
    def lidar_callback(self, msg):
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        # Filter out invalid readings
        valid_mask = np.isfinite(ranges) & (ranges > 0)
        filtered_ranges = ranges[valid_mask]
        filtered_angles = angles[valid_mask]

        # Convert to Cartesian coordinates
        x_coords = filtered_ranges * np.cos(filtered_angles)
        y_coords = filtered_ranges * np.sin(filtered_angles)

        # Detect obstacles within threshold
        obstacle_threshold = 2.0  # Meters
        obstacles = np.column_stack((x_coords, y_coords))
        close_obstacles_mask = np.linalg.norm(obstacles, axis=1) < obstacle_threshold
        close_obstacles = obstacles[close_obstacles_mask]
        close_distances = filtered_ranges[close_obstacles_mask]
        close_angles = filtered_angles[close_obstacles_mask]

        # Convert angles to degrees for easier reading
        close_angles_deg = np.degrees(close_angles)

        # Log detailed obstacle information
        if len(close_obstacles) > 0:
            self.get_logger().info("\n=== Obstacle Detection Report ===")
            self.get_logger().info(f"Found {len(close_obstacles)} obstacles within {obstacle_threshold}m")
            
            # Report closest obstacles in different directions
            sectors = {
                "Front": (-45, 45),
                "Right": (-135, -45),
                "Back": (135, -135),
                "Left": (45, 135)
            }
            
            for sector, (min_angle, max_angle) in sectors.items():
                sector_mask = (close_angles_deg >= min_angle) & (close_angles_deg < max_angle)
                if np.any(sector_mask):
                    min_dist = np.min(close_distances[sector_mask])
                    self.get_logger().info(f"{sector}: Closest obstacle at {min_dist:.2f}m")
                else:
                    self.get_logger().info(f"{sector}: No obstacles within {obstacle_threshold}m")
            
            self.get_logger().info("===========================")
        else:
            self.get_logger().info("No obstacles detected within threshold")

def main(args=None):
    rclpy.init(args=args)
    node = LidarProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()