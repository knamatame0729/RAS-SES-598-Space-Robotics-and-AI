#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import numpy as np
from scipy import linalg

class CartPoleLQRController(Node):
    def __init__(self):
        super().__init__('cart_pole_lqr_controller')
        
        # System parameters
        self.M = 1.0  # Mass of cart (kg)
        self.m = 1.0  # Mass of pole (kg)
        self.L = 1.0  # Length of pole (m)
        self.g = 9.81  # Gravity (m/s^2)
        
        # State space matrices
        self.A = np.array([
            [0, 1, 0, 0],
            [0, 0, (self.m * self.g) / self.M, 0],
            [0, 0, 0, 1],
            [0, 0, ((self.M + self.m) * self.g) / (self.M * self.L), 0]
        ])
        
        self.B = np.array([
            [0],
            [1/self.M],
            [0],
            [-1/(self.M * self.L)]
        ])
        
        # LQR cost matrices
        self.Q = np.diag([1.0, 1.0, 10.0, 10.0])  # State cost
        self.R = np.array([[0.1]])  # Control cost
        
        # Compute LQR gain matrix
        self.K = self.compute_lqr_gain()
        self.get_logger().info(f'LQR Gain Matrix: {self.K}')
        
        # Initialize state estimate
        self.x = np.zeros((4, 1))
        self.state_initialized = False
        self.last_control = 0.0
        self.control_count = 0

        # For Analyze the System
        self.max_cart_displacement = 0.0
        self.max_pole_angle = 0.0
        self.cart_position = []
        self.control_effort = []
        self.disturbance = None

        # Create publishers and subscribers
        self.cart_cmd_pub = self.create_publisher(
            Float64, 
            '/model/cart_pole/joint/cart_to_base/cmd_force', 
            10
        )
        
        # Verify publisher created successfully
        if self.cart_cmd_pub:
            self.get_logger().info('Force command publisher created successfully')
        
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/world/empty/model/cart_pole/joint_state',
            self.joint_state_callback,
            10
        )

        
        
        # Control loop timer (50 Hz)
        self.timer = self.create_timer(0.02, self.control_loop)
        
        self.get_logger().info('Cart-Pole LQR Controller initialized')
    
    def compute_lqr_gain(self):
        """Compute the LQR gain matrix K."""
        P = linalg.solve_continuous_are(self.A, self.B, self.Q, self.R)
        K = np.linalg.inv(self.R) @ self.B.T @ P
        return K
    
    def joint_state_callback(self, msg):
        """Update state estimate from joint states."""
        try:
            # Get indices for cart and pole joints
            cart_idx = msg.name.index('cart_to_base')  # Cart position/velocity
            pole_idx = msg.name.index('pole_joint')    # Pole angle/velocity
            
            # State vector: [x, ẋ, θ, θ̇]
            self.x = np.array([
                [msg.position[cart_idx]],     # Cart position (x)
                [msg.velocity[cart_idx]],     # Cart velocity (ẋ)
                [msg.position[pole_idx]],     # Pole angle (θ)
                [msg.velocity[pole_idx]]      # Pole angular velocity (θ̇)
            ])
            
            # Compute maximum values of cart displacement and pole angle
            self.max_cart_displacement = max(self.max_cart_displacement, abs(self.x[0, 0]))
            self.max_pole_angle = max(self.max_pole_angle, abs(self.x[2, 0]))

            # Add cart position on the list
            self.cart_position.append(self.x[0, 0])
            
            #if self.disturbance is None:
             #   self.disturbance = False
              #  return
            
            # If the pole angle is more than ±0.5 rad, set the 
            if not self.disturbance:
                if abs(self.x[2, 0]) > 0.08:
                    self.disturbance = True
                    # Time tracking for computing the recovery time
                    self.start_time_disturbance = self.get_clock().now().nanoseconds / 1e9 
                    self.get_logger().info('-----------Timer Start------')

            if not self.state_initialized:
                self.get_logger().info(f'Initial state: cart_pos={msg.position[cart_idx]:.3f}, cart_vel={msg.velocity[cart_idx]:.3f}, pole_angle={msg.position[pole_idx]:.3f}, pole_vel={msg.velocity[pole_idx]:.3f}')
                self.state_initialized = True
                
        except (ValueError, IndexError) as e:
            self.get_logger().warn(f'Failed to process joint states: {e}, msg={msg.name}')
    
    def compute_rms_error(self, target_position, real_position):
        return np.sqrt(np.mean((np.array(real_position) - target_position)**2))

    def control_loop(self):
        """Compute and apply LQR control."""
        try:
            if not self.state_initialized:
                self.get_logger().warn('State not initialized yet')
                return

            # Compute control input u = -Kx
            u = -self.K @ self.x
            force = float(u[0])

            # Add control effort on the list
            self.control_effort.append(abs(force))

            # Log control input periodically
            if abs(force - self.last_control) > 0.1 or self.control_count % 100 == 0:
                self.get_logger().info(f'State: {self.x.T}, Control force: {force:.3f}N')
            
            # Compute Recovery time
            if self.disturbance:
                if abs(self.x[2, 0]) < 0.007:
                    self.recovery_time = self.get_clock().now().nanoseconds / 1e9 - self.start_time_disturbance
                    self.get_logger().info(f'Recovery time: {self.recovery_time:.3f} seconds')
                    self.disturbance = False

            # Publish control command
            msg = Float64()
            msg.data = force
            self.cart_cmd_pub.publish(msg)
            
            self.last_control = force
            self.control_count += 1
            
        except Exception as e:
            self.get_logger().error(f'Control loop error: {e}')

    def shutdown(self):
       # Compute average control effort and max control effort
       avg_control_effort = np.mean(self.control_effort)
       max_control_effort = max(self.control_effort)

       # Compute RMS error
       rms_error = self.compute_rms_error(0.0, self.cart_position)

       self.get_logger().info(f'Max Cart Displacement: {self.max_cart_displacement:.3f} m')
       self.get_logger().info(f'Max Pole Angle: {self.max_pole_angle:.3f} rad')
       self.get_logger().info(f'Average Control Effort: {avg_control_effort:.3f} N')
       self.get_logger().info(f'Max Control Effort: {max_control_effort:.3f} N')
       self.get_logger().info(f'RMS Cart Position Error: {rms_error:.3f} m')



def main(args=None):
    rclpy.init(args=args)
    controller = CartPoleLQRController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.shutdown()
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 