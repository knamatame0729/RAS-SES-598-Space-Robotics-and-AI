#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import numpy as np
import math
from collections import deque
from std_msgs.msg import Float64
from rcl_interfaces.msg import SetParametersResult


class BoustrophedonController(Node):
    def __init__(self):
        super().__init__('lawnmower_controller')
        
        # Declare parameters with default values
        self.declare_parameters(
            namespace='',
            parameters=[
                ('Kp_linear', 10.0),
                ('Kd_linear', 0.1),
                ('Kp_angular', 5.0),
                ('Kd_angular', 0.2),
                ('spacing', 1.0)
            ]
        )
        
        # Get initial parameter values
        self.Kp_linear = self.get_parameter('Kp_linear').value
        self.Kd_linear = self.get_parameter('Kd_linear').value
        self.Kp_angular = self.get_parameter('Kp_angular').value
        self.Kd_angular = self.get_parameter('Kd_angular').value
        self.spacing = self.get_parameter('spacing').value
        
        # Add parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # Create publisher and subscriber
        self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        
        # Lawnmower pattern parameters
        self.waypoints = self.generate_waypoints()
        self.current_waypoint = 0
        
        # Cross-track error calculation
        self.cross_track_errors = deque(maxlen=1000)  # Store last 1000 errors
        
        # State variables
        self.pose = Pose()
        self.prev_linear_error = 0.0
        self.prev_angular_error = 0.0
        self.prev_time = self.get_clock().now()
        
        # Create control loop timer
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # Add publisher for cross-track error
        self.error_pub = self.create_publisher(
            Float64, 
            'cross_track_error', 
            10
        )
        
        self.get_logger().info('Lawnmower controller started')
        self.get_logger().info(f'Following waypoints: {self.waypoints}')

    def generate_waypoints(self):
        waypoints = []
        # Start higher up in the window (around y=8) and work down to y=3
        y = 8.0  # Start higher in the window
        
        while y >= 3.0:  # End higher than before to use middle space
            # Add points for current line
            if len(waypoints) % 2 == 0:
                waypoints.append((2.0, y))  # Left to right, but not too close to edge
                waypoints.append((9.0, y))  # Not too close to right edge
            else:
                waypoints.append((9.0, y))  # Right to left
                waypoints.append((2.0, y))
            
            y -= self.spacing  # Move down instead of up
        
        return waypoints

    def calculate_cross_track_error(self):
        """
        Calculate signed cross-track error from current position to line segment
        Returns:
            float: Signed cross-track error (positive = left of path, negative = right of path)
        """
        if self.current_waypoint < 1:
            return 0.0
            
        # Get current line segment endpoints
        start = np.array(self.waypoints[self.current_waypoint - 1])
        end = np.array(self.waypoints[self.current_waypoint])
        pos = np.array([self.pose.x, self.pose.y])
        
        # Calculate path vector and normalize
        path_vector = end - start
        path_length = np.linalg.norm(path_vector)
        if path_length < 1e-6:  # Avoid division by zero
            return np.linalg.norm(pos - start)
            
        path_unit = path_vector / path_length
        
        # Vector from start to current position
        pos_vector = pos - start
        
        # Calculate projection length along path
        projection_length = np.dot(pos_vector, path_unit)
        
        # Clamp projection to line segment
        projection_length = max(0, min(path_length, projection_length))
        
        # Calculate projected point on path
        projected_point = start + projection_length * path_unit
        
        # Calculate error vector
        error_vector = pos - projected_point
        
        # Calculate signed error (positive = left of path, negative = right of path)
        # Using 2D cross product for sign
        error_sign = np.sign(np.cross(path_unit, error_vector/np.linalg.norm(error_vector)))
        error = np.linalg.norm(error_vector) * error_sign
        
        # Store error and calculate statistics
        self.cross_track_errors.append(abs(error))  # Store absolute error for statistics
        current_avg = sum(self.cross_track_errors) / len(self.cross_track_errors)
        current_max = max(self.cross_track_errors)
        current_min = min(self.cross_track_errors)
        
        # Publish error
        error_msg = Float64()
        error_msg.data = error
        self.error_pub.publish(error_msg)
        
        # Log statistics
        self.get_logger().info(
            f'Cross-track error - Current: {error:.3f}, '
            f'Avg: {current_avg:.3f}, '
            f'Min: {current_min:.3f}, '
            f'Max: {current_max:.3f}, '
            f'Segment: ({start[0]:.1f},{start[1]:.1f})->({end[0]:.1f},{end[1]:.1f})'
        )
        
        return error

    def pose_callback(self, msg):
        self.pose = msg

    def get_distance(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def get_angle(self, x1, y1, x2, y2):
        return math.atan2(y2 - y1, x2 - x1)

    def control_loop(self):
        if self.current_waypoint >= len(self.waypoints):
            # Pattern complete
            self.get_logger().info('Lawnmower pattern complete')
            if self.cross_track_errors:
                final_avg_error = sum(self.cross_track_errors) / len(self.cross_track_errors)
                self.get_logger().info(f'Final average cross-track error: {final_avg_error:.3f}')
            self.timer.cancel()
            return

        # Calculate cross-track error
        cross_track_error = self.calculate_cross_track_error()

        target_x, target_y = self.waypoints[self.current_waypoint]
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9  # Convert to seconds

        # Calculate errors
        distance = self.get_distance(self.pose.x, self.pose.y, target_x, target_y)
        target_angle = self.get_angle(self.pose.x, self.pose.y, target_x, target_y)
        angular_error = target_angle - self.pose.theta

        # Normalize angular error to [-pi, pi]
        while angular_error > math.pi:
            angular_error -= 2 * math.pi
        while angular_error < -math.pi:
            angular_error += 2 * math.pi

        # Calculate error derivatives
        linear_error_derivative = (distance - self.prev_linear_error) / dt
        angular_error_derivative = (angular_error - self.prev_angular_error) / dt

        # PD control
        linear_velocity = self.Kp_linear * distance + self.Kd_linear * linear_error_derivative
        angular_velocity = self.Kp_angular * angular_error + self.Kd_angular * angular_error_derivative

        # Create and publish velocity command
        vel_msg = Twist()
        vel_msg.linear.x = min(linear_velocity, 2.0)  # Limit maximum speed
        vel_msg.angular.z = angular_velocity
        self.velocity_publisher.publish(vel_msg)

        # Update previous errors and time
        self.prev_linear_error = distance
        self.prev_angular_error = angular_error
        self.prev_time = current_time

        # Check if we've reached the current waypoint
        if distance < 0.1:  # Within 0.1 units of target
            self.current_waypoint += 1
            self.get_logger().info(f'Reached waypoint {self.current_waypoint}')

    def parameter_callback(self, params):
        """Callback for parameter updates"""
        for param in params:
            if param.name == 'Kp_linear':
                self.Kp_linear = param.value
                self.get_logger().info(f'Updated Kp_linear to {param.value}')
            elif param.name == 'Kd_linear':
                self.Kd_linear = param.value
                self.get_logger().info(f'Updated Kd_linear to {param.value}')
            elif param.name == 'Kp_angular':
                self.Kp_angular = param.value
                self.get_logger().info(f'Updated Kp_angular to {param.value}')
            elif param.name == 'Kd_angular':
                self.Kd_angular = param.value
                self.get_logger().info(f'Updated Kd_angular to {param.value}')
            elif param.name == 'spacing':
                self.spacing = param.value
                self.waypoints = self.generate_waypoints()  # Regenerate waypoints with new spacing
                self.get_logger().info(f'Updated spacing to {param.value} and regenerated waypoints')
        
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    controller = BoustrophedonController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 