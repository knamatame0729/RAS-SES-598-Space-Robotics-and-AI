#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import BatteryStatus
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import Float32

class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor')
        
        # Configure QoS profile for PX4 communication
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        # Publish battery usage
        self.battery_usage_publisher = self.create_publisher(
            Float32, '/battery/battery_usage', 10
        )

        # Subscribe battery status from PX4-Autopilot/src/lib/battery/battery.cpp
        self.battery_status_subscriber = self.create_subscription(
            BatteryStatus, '/fmu/out/battery_status',
            self.battery_status_callback, qos_profile
        )

        self.initial_battery = None
        self.current_battery = None

    
    def battery_status_callback(self, msg):
        self.current_battery = msg.remaining
        if self.initial_battery is None:
            self.initial_battery = msg.remaining
        
        # Compute battery usage
        battery_usage = (self.initial_battery - self.current_battery) * 100
        
        # Publish battery usage
        battery_usage_msg = Float32()
        battery_usage_msg.data = battery_usage
        self.battery_usage_publisher.publish(battery_usage_msg)
        """
        self.get_logger().info(
            f"Battery: {self.current_battery*100:.1f}% remaining, "
            f"Usage: {battery_usage*100:.1f}%"
        )
        """
        
def main():
    rclpy.init()
    node = BatteryMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
