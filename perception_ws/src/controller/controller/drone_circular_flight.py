#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus

class CircularFlight(Node):
    """Node for making a PX4 x500 drone fly in a circular path"""

    def __init__(self) -> None:
        super().__init__('circular_flight')

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

        # State variables
        self.vehicle_status = None
        self.vehicle_local_position = None
        self.offboard_mode = False
        self.armed = False
        self.start_position = None

        # Circle parameters --- need to relate speed, radius, ang velocity
        self.radius = 200.0   # Radius of the circle (meters)
        self.altitude = -10.0  # Fixed altitude (NED frame: negative values)
        self.center_x = 0.0
        self.center_y = 0.0
        self.theta = 0.0      # Initial angle (radians)
        self.angular_velocity = 0.01  # Speed of rotation (radians per step)

        # Create a timer to publish control commands (10 Hz or 0.1s interval)
        self.timer = self.create_timer(0.1, self.timer_callback)


    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position
        if self.start_position is None and vehicle_local_position.xy_valid:
            self.start_position = [vehicle_local_position.x, vehicle_local_position.y, vehicle_local_position.z]

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status
        self.offboard_mode = vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        self.armed = vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED


    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    # def disarm(self):
    #     """Send a disarm command to the vehicle."""
    #     self.publish_vehicle_command(
    #         VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
    #     self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode..." + str(VehicleStatus.NAVIGATION_STATE_OFFBOARD))

    def takeoff(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param7=15.0)
        self.get_logger().info("Taking off...")


    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self):
        """Compute and publish the next position setpoint along the circle."""
        # Compute next point on the circle
        x = self.center_x + self.radius * math.cos(self.theta)
        y = self.center_y + self.radius * math.sin(self.theta)
        z = self.altitude  # Keep altitude constant
        yaw = self.theta + math.pi / 2  # Keep nose pointing along the tangent (0 = facing East, pi/2 = facing North, ...)

        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Flying to [{x:.2f}, {y:.2f}, {z:.2f}] with yaw {yaw:.2f}")

        # Increment angle to move along the circle
        self.theta += self.angular_velocity
        if self.theta > 2 * math.pi:
            self.theta = 0  # Reset after full circle

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
        # If not in offboard mode, send command to enter offboard
        if not self.offboard_mode:
            self.engage_offboard_mode()

        # If not armed, send command to arm
        if not self.armed:
            self.arm()

        self.publish_offboard_control_heartbeat_signal()
        self.publish_position_setpoint()

def main(args=None) -> None:
    print('Starting circular flight node...')
    rclpy.init(args=args)
    circular_flight = CircularFlight()
    rclpy.spin(circular_flight)
    circular_flight.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
