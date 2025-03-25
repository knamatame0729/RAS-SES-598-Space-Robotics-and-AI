#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, VehicleControlMode, OffboardControlMode, TrajectorySetpoint, VehicleOdometry, VehicleStatus
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray, Int32
from custom_msgs.msg import MarkerWithId

class SimpleTestNode(Node):
    def __init__(self):
        super().__init__('simple_test_node')
        self.get_logger().info('Cylinder landing node initialized')
        
        # Configure QoS profile for PX4 communication
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        
        # Subscribers
        self.vehicle_odometry_subscriber = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', 
            self.vehicle_odometry_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self.vehicle_status_callback, qos_profile)
            
        # Subscribe to geometry tracker outputs
        self.cylinder_pose_subscriber = self.create_subscription(
            Point, '/geometry/cylinder_center',
            self.cylinder_pose_callback, 10)
        self.cylinder_info_subscriber = self.create_subscription(
            Float32MultiArray, '/geometry/cylinder_info',
            self.cylinder_info_callback, 10)

        # Subscribe aruco marker position
        self.aruco_marker_pos = self.create_subscription(
            MarkerWithId, '/aruco/marker_position',
            self.marker_position_callback, 10)
        """
        # Subscribe amount of marker
        self.marker_count_sub  = self.create_subscription(
            Int32, '/aruco/marker_count',
            self.marker_count_callback, 10)
        """
        
        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_odometry = VehicleOdometry()
        self.vehicle_status = VehicleStatus()
        self.cylinder_position = None
        self.cylinder_info = None
        self.aruco_positions = {'0': [], '1': []}
        self.world_positions = {'0': [], '1': []}
        self.target_position = None
        self.marker_search_frag = False
        self.target_position = None
        self.found_tallest_rock = False
        self.marker_count = 0
        
        # Flight parameters
        self.TARGET_HEIGHT = 11.0  # meters
        self.POSITION_THRESHOLD = 0.1  # meters
        self.MIN_ARUCO_COUNT = 2 # at least two aruco marker
        self.WAYPOINT_WAIT_TIME = 10.0

        # Waypoints
        self.search_points = [(0.0, 0.0), (0.0, 5.0), (0.0, -5.0)]
        self.current_waypoint = 0
        self.wait_timer = 0.0
        
        # State machine
        self.state = "TAKEOFF"
        
        # Create a timer to publish control commands
        self.create_timer(0.1, self.control_loop)  # 10Hz control loop

    def vehicle_odometry_callback(self, msg):
        """Store vehicle position from odometry."""
        self.vehicle_odometry = msg

    def vehicle_status_callback(self, msg):
        """Store vehicle status."""
        self.vehicle_status = msg

    def arm(self):
        """Send arm command."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info("Arm command sent")

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Offboard mode command sent")

    def publish_offboard_control_mode(self):
        """Publish offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        """Publish vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def publish_trajectory_setpoint(self, x, y, z, yaw):
        """Publish trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = float(yaw)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def is_at_position(self, x, y, z):
        """Check if the drone has reached the target height."""
        try:
            current_x = self.vehicle_odometry.position[0]
            current_y = self.vehicle_odometry.position[1]
            current_height = -self.vehicle_odometry.position[2]  # Convert NED to altitude
            return (abs(current_x - x) < self.POSITION_THRESHOLD and 
                    abs(current_y - y) < self.POSITION_THRESHOLD and
                    abs(current_height - z) < self.POSITION_THRESHOLD)
        except (IndexError, AttributeError):
            return False

    def cylinder_pose_callback(self, msg):
        """Store cylinder position from geometry tracker."""
        self.cylinder_position = msg
        if self.offboard_setpoint_counter % 10 == 0:  # Log every second
            self.get_logger().info(
                f'Cylinder at pixel ({msg.x:.1f}, {msg.y:.1f}) depth {msg.z:.2f}m'
            )

    def cylinder_info_callback(self, msg):
        """Store cylinder information from geometry tracker."""
        self.cylinder_info = msg.data  # [width, height, angle, confidence]
        if self.offboard_setpoint_counter % 10 == 0:  # Log every second
            self.get_logger().info(
                f'Cylinder size: {msg.data[0]:.1f}x{msg.data[1]:.1f} '
                f'angle: {msg.data[2]:.1f}° confidence: {msg.data[3]:.2f}'
            )
    """
    def marker_count_callback(self, msg):
        self.marker_count = msg.data
        if self.offboard_setpoint_counter % 10 == 0:
            self.get_logger().info(f"Detected {self.marker_count} markers")
    """
          
    def marker_position_callback(self, msg):
        if self.marker_search_frag:
            marker_id = msg.id
            if marker_id not in self.aruco_positions:
                self.aruco_positions[marker_id] = []
                self.world_positions[marker_id] = []

            self.aruco_positions[marker_id].append(msg)
            world_pos = self.camera_to_world(Point(x=msg.x, y=msg.y, z=msg.z))
            self.world_positions[marker_id].append(world_pos)
            #self.get_logger().info(f"Saved marker position: x={msg.x}, y={msg.y}, z={msg.z}")

    def compute_average_position(self, marker_id):
        if marker_id in self.world_positions and len(self.world_positions[marker_id]) >= 200:
            positions = self.world_positions[marker_id]
            avg_x = sum(p.x for p in positions) / len(positions)
            avg_y = sum(p.y for p in positions) / len(positions)
            avg_z = sum(p.z for p in positions) / len(positions)
            return Point(x=avg_x, y=avg_y, z=avg_z)
        return None
    
    def camera_to_world(self, aruco_pos):
        """ Convert cylinder rock position to world coordinate from aruco """
        try:
            drone_x = self.vehicle_odometry.position[0]
            drone_y = self.vehicle_odometry.position[1]
            drone_z = -self.vehicle_odometry.position[2]

            rock_world_x = drone_x + aruco_pos.x
            rock_world_y = drone_y + aruco_pos.y
            rock_world_z = drone_z - aruco_pos.z
            
            return Point(x=rock_world_x, y=rock_world_y, z=rock_world_z)
        except (IndexError, AttributeError):
            self.get_logger().info("Failed to convert to world coordinates")
            return Point(x=0.0, y=0.0, z=0.0)

    def find_tallest_rock(self):
        self.found_tallest_rock = False
        if self.world_positions:
            avg_positions = {}
            for marker_id in self.world_positions:
                avg_pos = self.compute_average_position(marker_id)
                if avg_pos:
                    avg_positions[marker_id] = avg_pos
            if avg_positions:
                self.target_position = max(avg_positions.values(), key=lambda p: p.z)
                self.get_logger().info(f"Tallest cylinder: x={self.target_position.x:.2f}, y={self.target_position.y:.2f}, z={self.target_position.z:.2f}")
                self.found_tallest_rock = True
        return self.found_tallest_rock

    def control_loop(self):
        """Timer callback for control loop."""
        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()
            self.get_logger().info("Vehicle armed and offboard mode enabled")

        self.publish_offboard_control_mode()

        try:
            current_height = -self.vehicle_odometry.position[2]
        except (IndexError, AttributeError):
            current_height = 0.0

        if self.state == "TAKEOFF":
            # Take off to target height
            self.publish_trajectory_setpoint(
                x=0.0,
                y=0.0,
                z=-self.TARGET_HEIGHT,  # Negative because PX4 uses NED
                yaw=0.0
            )
            
            # Log current height every second
            if self.offboard_setpoint_counter % 10 == 0:
                self.get_logger().info(f"Taking off... Current height: {current_height:.2f}m")
            
            # Check if we've reached target height
            if self.is_at_position(0.0, 0.0, self.TARGET_HEIGHT):
                # When the drone reached target height, beginning searching aruco marker
                self.marker_search_frag = True
                self.state = "SEARCH"
                self.get_logger().info("Reached desired height, beginning searching makers")

        elif self.state == "SEARCH":
            waypoint_x, waypoint_y = self.search_points[self.current_waypoint]
            self.publish_trajectory_setpoint(
                x=waypoint_x,
                y=waypoint_y,
                z=-self.TARGET_HEIGHT,  # Negative because PX4 uses NED
                yaw=0.0
            )

            if self.offboard_setpoint_counter % 10 == 0:
                self.get_logger().info(f"Searching... Found {self.marker_count} markers")
            
            if self.is_at_position(waypoint_x, waypoint_y, self.TARGET_HEIGHT):
                if self.current_waypoint == 0:
                    self.current_waypoint += 1
                else:
                    if self.wait_timer < self.WAYPOINT_WAIT_TIME:
                        self.wait_timer += 0.1
                    else:
                        self.wait_timer = 0.0 # Initialize timer
                        self.current_waypoint += 1

                if self.current_waypoint >= len(self.search_points):
                    if self.find_tallest_rock():
                        self.state = "PRELANDING"
                
            
        elif self.state == "PRELANDING":
            self.publish_trajectory_setpoint(
                x=self.target_position.x,
                y=self.target_position.y,
                z=-self.TARGET_HEIGHT,  # Negative because PX4 uses NED
                yaw=0.0
            )

            if self.offboard_setpoint_counter % 10 == 0:
                self.get_logger().info(f"Setting up for landing... At x={self.target_position.x:.2f}, y={self.target_position.y:.2f}, z={self.target_position.z}")
            if self.is_at_position(self.target_position.x, self.target_position.y, self.TARGET_HEIGHT):
                self.state = "LAND"

        elif self.state == "LAND":
            # Land by going to height 0
            self.publish_trajectory_setpoint(
                x=self.target_position.x,
                y=self.target_position.y,
                z=-self.target_position.z, 
                yaw=0.0
            )

            if self.offboard_setpoint_counter % 10 == 0:
                self.get_logger().info(f"Landing... Height: {current_height:.2f}m")
            if self.is_at_position(self.target_position.y, self.target_position.x, 0.0):
                self.state = "COMPLETE"

        elif self.state == "COMPLETE":
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0
            )
            self.get_logger().info("Mission Complete")
            rclpy.shutdown()

        self.offboard_setpoint_counter += 1

def main():
    rclpy.init()
    node = SimpleTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 