import rclpy
from rclpy.node import Node

import numpy as np
from collections import deque
from ament_index_python.packages import get_package_prefix
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16MultiArray, String, Bool
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped

import os

from ros2_pydata import to_ackermann
from rclpy.qos import qos_profile_sensor_data  # Quality of Service settings for real-time data
from smart_parking.utils import get_mapping, replay_bagfile, to_string, to_bool
from odometry.call_service import call_reset_odometry_service


class AutonomousParkingNode(Node):
    def __init__(self, bagfile_path, frame='USS_SRF'):
        super().__init__('autonomous_parking_node')

        # Check if the bagfile exists before doing anything else
        if not os.path.exists(bagfile_path):
            raise FileNotFoundError(f"Bagfile not found: {bagfile_path}")

        self.bagfile_path = bagfile_path

        # Define Quality of Service (QoS) for communication
        qos_profile = qos_profile_sensor_data  # Suitable for sensor data
        qos_profile.depth = 1  # Keep only the latest message

        # --- ROS Subscribers ---
        self.create_subscription(PoseStamped, '/ackermann/pose', self.odometry_callback, qos_profile)
        self.create_subscription(Int16MultiArray, '/uss_sensors', self.uss_callback, qos_profile)
        self.create_subscription(AckermannDriveStamped, '/rc/ackermann_cmd', self.ackermann_callback, qos_profile)
        self.create_subscription(Joy, '/rc/joy', self.joy_callback, qos_profile)

        # --- ROS Publishers ---
        self.autonomous_pub = self.create_publisher(AckermannDriveStamped, '/autonomous/ackermann_cmd', qos_profile)
        self.status_pub = self.create_publisher(String, '/parking/status', qos_profile)
        self.open_pub = self.create_publisher(Bool, '/parking/open', qos_profile)

        # --- Parking Logic Parameters & Enums ---
        self.DEADMAN = 0
        self.AUTONOMOUS = 1
        self.MANUAL = 2
        self.names = {self.DEADMAN: 'Deadman', self.AUTONOMOUS: 'Autonomous', self.MANUAL: 'Manual'}

        self.mode_btn = 0  # Joystick button index for switching modes

        # --- State Machine & Threshold Variables ---
        self.window_size = 4           # Size of moving window for filtering sensor noise
        self.dist_open = 0.25          # Minimum distance (m) to consider a spot "open"
        self.parking_length = 0.9      # Required open space length (m) to fit the vehicle
        self.forward_adjustment = 0.3  # Extra distance (m) to drive forward before reversing
        self.dx = 0                    # Change in X position per step
        self.previous_x = 0            # Last recorded X position
        self.length = 0.0              # Tracked length of the current open parking spot

        # --- Sensor Mapping Setup ---
        self.frame2index = get_mapping()
        self.uss_index = self.frame2index[frame] # Extract the target ultrasonic sensor index

        # Initialize tracking arrays and structures
        self.init_mapping()

        # --- Default Robot State Variables ---
        self.mode = None               # Current driving mode (Deadman, Autonomous, Manual)
        self.status = "DRIVING"        # State machine: DRIVING -> SCANNING -> POSITIONING -> PARKING_IN
        self.is_open = False           # Flag indicating if a spot is currently detected
        self.xpos = 0.0                # Tracked vehicle X position (added initialization)

        # Inline helper function to publish a full stop command
        self.brake = lambda: self.autonomous_pub.publish(to_ackermann(.0, .0))

        self.get_logger().info("Autonomous Parking Node has been started.")

    def odometry_callback(self, pose_msg: PoseStamped):
        """Updates the vehicle's current position along the X-axis."""
        self.xpos = pose_msg.pose.position.x

    def uss_callback(self, msg: Int16MultiArray):
        """Processes raw ultrasonic sensor data to detect openings and update state."""
        measurements = np.array(msg.data) / 100.0  # Convert centimeters to meters
        distance = measurements[self.uss_index]
        
        if distance < 0: 
            return  # Ignore invalid (negative) distance readings
            
        self.update_state(distance)

        # Publish the current state for visualization/monitoring
        self.status_pub.publish(to_string(self.status))
        self.open_pub.publish(to_bool(self.is_open))

    def ackermann_callback(self, msg: AckermannDriveStamped):
        # Forward manual commands as autonomous commands,
        # allowing manual steering along the parking space in autonomous mode.

        # Stop once parking starts to avoid conflicts with parking commands.
        if self.mode != self.AUTONOMOUS or self.status == "PARKING_IN":
            return

        self.autonomous_pub.publish(msg)

    def joy_callback(self, joy_msg: Joy):
        # Reset the odometry and switch the vehicle from driving to scanning mode.
        # Update the driving mode based on joystick input.
        mode = joy_msg.buttons[self.mode_btn]

        if self.mode != mode:  # Mode changed
            self.mode = mode
            self.get_logger().info(f"Switched to {mode}: {self.names[mode]} mode!")

            if mode == self.AUTONOMOUS:
                self.init_mapping()
                self.status = "SCANNING"
            if mode == self.MANUAL:
                self.status = "DRIVING"

    def update_state(self, distance):
        """Main state machine managing scanning, positioning, and parking execution."""

        # Moving window filter: spot is open if any reading in the window exceeds dist_open
        self.window.append(distance)
        self.is_open = np.any(np.array(self.window) >= self.dist_open)
        
        self.get_logger().info(f"x: {self.xpos:.3f}, len: {self.length:.3f}, d: {distance:.2f}, is_open: {self.is_open}, status: {self.status}")

        # Calculate incremental distance traveled since last callback
        self.dx = self.xpos - self.previous_x
        self.previous_x = self.xpos

        # --- STATE: SCANNING FOR A SPOT ---
        if self.status == "SCANNING":
            if self.is_open:
                self.length += self.dx  # Accumulate spot length while space is open
            else:
                self.length = 0.0       # Reset tracking if space is blocked again

            # If spot is long enough, transition to positioning phase
            if self.length >= self.parking_length:
                self.status = "POSITIONING"
                self.target_position = self.xpos + self.forward_adjustment
                self.get_logger().info(f"Measured parking space length: {self.length:.2f} meters")
                self.get_logger().info(f"Initialising parking sequence: adjust position: {self.target_position:.2f}...")

        # --- STATE: POSITIONING THE CAR FOR REVERSE ---
        # If target position is reached, stop the vehicle and start the parking maneuver.
        if self.status == "POSITIONING":
            if self.xpos >= self.target_position:
                self.get_logger().info(f"Parking position x = {self.target_position:.3f} reached!")
                self.brake()
                self.status = "PARKING_IN"
                replay_bagfile(self.bagfile_path) # Playback pre-recorded sequence to park

    def init_mapping(self):
        """Resets moving filter window and resets internal odometry tracking variables."""

        # Call the reset service so the odometry starts from the origin with a cleared path.
        call_reset_odometry_service(self)
        
        self.window = deque([0.0] * self.window_size, maxlen=self.window_size)
        # Reset tracking properties to ensure state calculations start clean
        self.length = 0.0
        self.previous_x = self.xpos if hasattr(self, 'xpos') else 0.0
        self.get_logger().info("Mapping and state initialized.")


def main(args=None):
    rclpy.init(args=args)

    # Resolve paths: rewrites ROS2 workspace structure to find source asset files
    pkg_dir = get_package_prefix('smart_parking').replace('install', 'src')  
    bagfile_path = pkg_dir + '/bagfiles/reverse_parallel_parking_status/reverse_parallel_parking_status.mcap'

    node = AutonomousParkingNode(bagfile_path)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Autonomous Parking Node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()