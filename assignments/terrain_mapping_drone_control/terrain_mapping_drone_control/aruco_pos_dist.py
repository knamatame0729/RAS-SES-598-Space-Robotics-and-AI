#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import numpy as np
from custom_msgs.msg import MarkerWithId
import matplotlib.pyplot as plt
import os
from pathlib import Path

class ArucoDistributionNode(Node):
    def __init__(self):
        super().__init__('aruco_distribution_node')
        self.get_logger().info('Aruco distribution node initialized')

        # Subscribe to Aruco marker positions
        self.subscription = self.create_subscription(
            MarkerWithId,
            '/aruco/marker_position',
            self.marker_position_callback,
            10
        )

        # Dictionary to store marker positions
        self.marker_positions = {'0': [], '1': []}  # Example IDs 0 and 1

        # Output directory
        self.output_dir = str(Path.home() / "ros2_ws/src/terrain_mapping_drone_control")
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
            self.get_logger().info(f"Created directory {self.output_dir}")

    def marker_position_callback(self, msg):
        """Store Aruco marker position"""
        marker_id = str(msg.id)
        if marker_id not in self.marker_positions:
            self.marker_positions[marker_id] = []

        position = Point(x=msg.x, y=msg.y, z=msg.z)
        self.marker_positions[marker_id].append(position)
        self.get_logger().info(
            f"Marker ID {marker_id}: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f} stored"
        )

    def compute_and_save_distribution(self):
        """Compute distribution from all data and save to log and graph"""
        for marker_id, positions in self.marker_positions.items():
            if len(positions) < 10:  # Require at least 10 data points
                self.get_logger().info(f"Marker ID {marker_id}: Insufficient data ({len(positions)} points)")
                continue

            # Create lists of x, y, z values
            x_vals = [p.x for p in positions]
            y_vals = [p.y for p in positions]
            z_vals = [p.z for p in positions]

            # Histgram
            x_hist, x_bins = np.histogram(x_vals, bins=100, density=True)
            y_hist, y_bins = np.histogram(y_vals, bins=100, density=True)
            z_hist, z_bins = np.histogram(z_vals, bins=100, density=True)

            # Compute mean and variance
            x_mean, x_std = np.mean(x_vals), np.var(x_vals)
            y_mean, y_std = np.mean(y_vals), np.var(y_vals)
            z_mean, z_std = np.mean(z_vals), np.var(z_vals)

            # Visualize and save histogram
            self.visualize_histogram(marker_id, x_vals, y_vals, z_vals)

    def visualize_histogram(self, marker_id, x_vals, y_vals, z_vals):
        plt.figure(figsize=(15, 5))

        # X-axis histogram
        plt.subplot(1, 3, 1)
        plt.hist(x_vals, bins=10, density=True, color='blue', alpha=0.7)
        plt.title(f'Marker {marker_id} - X Distribution')
        plt.xlabel('X (m)')
        plt.ylabel('Density')

        # Y-axis histogram
        plt.subplot(1, 3, 2)
        plt.hist(y_vals, bins=100, density=True, color='green', alpha=0.7)
        plt.title(f'Marker {marker_id} - Y Distribution')
        plt.xlabel('Y (m)')
        plt.ylabel('Density')

        # Z-axis histogram
        plt.subplot(1, 3, 3)
        plt.hist(z_vals, bins=100, density=True, color='red', alpha=0.7)
        plt.title(f'Marker {marker_id} - Z Distribution')
        plt.xlabel('Z (m)')
        plt.ylabel('Density')

        # Adjust layout and save
        plt.tight_layout()
        output_path = os.path.join(self.output_dir, f'marker_{marker_id}_final_distribution.png')
        plt.savefig(output_path)
        plt.close()
        self.get_logger().info(f"Final distribution graph for Marker ID {marker_id} saved to {output_path}")

    def on_shutdown(self):
        """Compute and save overall distribution when node shuts down"""
        self.get_logger().info('Node is shutting down. Saving overall distribution.')
        self.compute_and_save_distribution()

def main():
    rclpy.init()
    node = ArucoDistributionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()  # Save distribution on shutdown
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()