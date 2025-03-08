#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleLocalPosition
import math

class ManualTakeoff(Node):
    def __init__(self):
        super().__init__('manual_plane_takeoff_node')

        # Configure QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Subscribers
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.local_position_callback, qos_profile)

        # Timer for publishing control commands
        self.create_timer(0.1, self.timer_callback)  # 10Hz

        # Takeoff parameters
        self.ground_speed = 100.0  # m/s - initial takeoff speed
        # self.climb_speed = 5.0    # m/s - vertical climb speed after rotation
        self.takeoff_altitude = 30.0  # meters
        self.rotation_speed = 80.0  # m/s - speed at which to start rotation
        self.rotation_pitch = math.radians(10.0)  # radians - pitch angle for initial climb
        self.cruise_altitude = 50.0  # meters - final cruise altitude
        
        # State variables
        self.vehicle_status = None
        self.local_position = None
        self.offboard_mode = False
        self.armed = False
        self.takeoff_state = 'INIT'  # States: INIT, GROUNDROLL, ROTATION, CLIMB, CRUISE
        self.start_position = None


    def vehicle_status_callback(self, msg):
        """Monitor vehicle status for arming and flight modes"""
        self.vehicle_status = msg
        self.offboard_mode = msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        self.armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def local_position_callback(self, msg):
        self.local_position = msg
        if self.start_position is None and msg.xy_valid:
            self.start_position = [msg.x, msg.y, msg.z]


    def timer_callback(self):
        """Publish control commands at regular intervals"""
        # If not in offboard mode, send command to enter offboard
        if not self.offboard_mode:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

        # If not armed, send command to arm
        if not self.armed:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

        self.publish_offboard_control_mode()
        self.execute_takeoff_sequence()


    def publish_offboard_control_mode(self):
        """Publish offboard control mode"""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def execute_takeoff_sequence(self):
        if not self.local_position or not self.start_position:
            return

        msg = TrajectorySetpoint()
        current_altitude = -self.local_position.z  # Convert from NED to altitude
        ground_speed = math.sqrt(self.local_position.vx**2 + self.local_position.vy**2)     # Calculate velocity from velocity in x and velocity in y

        if self.takeoff_state == 'INIT' and self.armed and self.offboard_mode:
            self.takeoff_state = 'ROTATION'
            self.get_logger().info('Starting ground roll')

        elif self.takeoff_state == 'GROUNDROLL':
            # Ground roll: Accelerate straight ahead while maintaining zero altitude
            # msg.position = [
            #     self.start_position[0] + 1000.0,  # Long distance ahead to maintain straight path
            #     self.start_position[1],
            #     self.start_position[2]
            # ]
            msg.velocity = [self.ground_speed, self.ground_speed, 0.0]
            
            if ground_speed >= self.rotation_speed:
                self.takeoff_state = 'ROTATION'
                self.get_logger().info('Starting rotation')

        elif self.takeoff_state == 'ROTATION':
            # Rotation: Maintain speed and begin pitching up
            desired_pitch = math.radians(10.0)
            
            # Calculate proper trajectory for desired pitch
            #   If we project 100m ahead:
            #       For 10 degrees this means:
            #           altitude_change = tan(10°) * 100
            #           altitude_change ≈ 17.6 meters
            # So we're telling the aircraft to climb 17.6m over 100m of forward travel
            # This naturally results in approximately 10 degrees of pitch

            # projection_distance = 100.0
            # altitude_change = math.tan(desired_pitch) * projection_distance
            # current_altitude = -self.local_position.z
    
            # msg.position = [
            #     self.start_position[0] + projection_distance,
            #     self.start_position[1],
            #     -(current_altitude + altitude_change)
            # ]

            msg.position = [
                self.start_position[0] + 1000.0,
                self.start_position[1],
                -self.cruise_altitude
            ]

            # Split total velocity from ground into components to achieve desired pitch
            msg.velocity = [
                self.ground_speed * math.cos(desired_pitch),
                0.0,
                -self.ground_speed * math.sin(desired_pitch)
            ]
            
            if current_altitude >= 5.0:  # Minimum safe altitude
                self.takeoff_state = 'CLIMB'
                self.get_logger().info('Starting climb')

        elif self.takeoff_state == 'CLIMB':
            # Climb: Maintain pitch and speed until reaching cruise altitude
            desired_pitch = math.radians(10.0)
            msg.position = [
                self.start_position[0] + 1000.0,
                self.start_position[1],
                -self.cruise_altitude
            ]
            msg.velocity = [
                self.ground_speed * math.cos(desired_pitch),
                0.0,
                -self.ground_speed * math.sin(desired_pitch)
            ]
            
            if current_altitude >= self.cruise_altitude * 0.95:
                self.takeoff_state = 'CRUISE'
                self.get_logger().info('Reached cruise altitude')

        elif self.takeoff_state == 'CRUISE':
            # Cruise: Maintain altitude and speed
            msg.position = [
                self.start_position[0] + 1000.0,
                self.start_position[1],
                -self.cruise_altitude
            ]
            msg.velocity = [self.ground_speed, 0.0, 0.0]

        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        """Publish vehicle commands"""
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

def main(args=None):
    rclpy.init(args=args)
    node = ManualTakeoff()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()