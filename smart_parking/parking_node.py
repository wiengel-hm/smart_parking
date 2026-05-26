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
from smart_parking.plot_utils import plot_vals

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
        self.parking_length = 0.7      # Required open space length (m) to fit the vehicle
        self.forward_adjustment = 0.3  # Extra distance (m) to drive forward before reversing
        self.dx = 0                    # Change in X position per step
        self.previous_x = 0            # Last recorded X position
        self.length = 0.0              # Tracked length of the current open parking spot
        self.positions = []            # All positions in x

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
        self.positions.append(self.xpos)

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

    def update_state(self, distance):
        """
        State machine with 4 states: DRIVING → SCANNING → POSITIONING → PARKING_IN.
        - DRIVING:     Car moves forward before reaching the parking zone.
        - SCANNING:    Car measures open space alongside it via the USS sensor.
                       A sliding window smooths noisy readings — a single spike
                       could falsely open or close a spot.
        - POSITIONING: Spot is long enough; car drives forward to align for reverse entry.
        - PARKING_IN:  Car executes the pre-recorded parking maneuver.
        """
        # Transition from DRIVING to SCANNING once the car passes the parking zone start
        if self.xpos > 0.6 and self.status == "DRIVING":
            self.status = "SCANNING"

        # TODO: SCANNING
        #   Use the sliding window to decide if the space is open or blocked.
        #   While open, accumulate self.length as the car moves (use self.dx).
        #   Reset self.length to 0.0 if the space becomes blocked again.
        #   Once self.length >= self.parking_length, set self.status = "POSITIONING"
        #   and compute self.target_position (how far ahead the car still needs to go).
 
        # TODO: POSITIONING
        #   Track self.xpos and wait until the car reaches self.target_position.
        #   Once there, brake the car and set self.status = "PARKING_IN".
        if self.status == "POSITIONING":
            self.get_logger().info(f"Waiting for car to reach target position...")
 
        # TODO: PARKING_IN
        #   Replay the pre-recorded bagfile to execute the parking maneuver.
        if self.status == "PARKING_IN":
            self.get_logger().info("Replaying reverse parallel parking maneuver...")



    def init_mapping(self):
        """Resets moving filter window and resets internal odometry tracking variables."""
        self.window = deque([0.0] * self.window_size, maxlen=self.window_size)
        # Reset tracking properties to ensure state calculations start clean
        self.length = 0.0
        self.previous_x = self.xpos if hasattr(self, 'xpos') else 0.0
        self.get_logger().info("Mapping and state initialized.")

    def plot_results(self, pkg_path):
        plots_path = pkg_path + '/plots/xpos_movement.png'
        plot_vals(self.positions, plots_path)

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
        pass
    finally:
        node.plot_results(pkg_dir)
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()