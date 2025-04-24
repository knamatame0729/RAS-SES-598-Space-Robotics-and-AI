#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo

class CameraInfoLogger(Node):
    def __init__(self):
        super().__init__('camera_info_logger')

        self.declare_parameter('camera_info_topic', '/drone/down_mono/camera_info')
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.get_logger().info(f"Camera info topic: {self.camera_info_topic}")
        
        self.camera_params = None
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            10
        )
        self.get_logger().info("CameraInfoLogger initialized, waiting for CameraInfo messages")

    def camera_info_callback(self, msg):
        if self.camera_params is None:  
            self.camera_params = {
                'fx': msg.k[0],
                'fy': msg.k[4],
                'cx': msg.k[2],
                'cy': msg.k[5],
                'width': msg.width,
                'height': msg.height,
                'distortion': msg.d
            }
            self.get_logger().info(
                f"Received CameraInfo:\n"
                f"  fx: {self.camera_params['fx']}\n"
                f"  fy: {self.camera_params['fy']}\n"
                f"  cx: {self.camera_params['cx']}\n"
                f"  cy: {self.camera_params['cy']}\n"
                f"  width: {self.camera_params['width']}\n"
                f"  height: {self.camera_params['height']}\n"
                f"  distortion: {self.camera_params['distortion']}"
            )

def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()