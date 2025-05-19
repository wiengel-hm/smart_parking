import rclpy
from rclpy.node import Node

import numpy as np
from collections import deque
from ament_index_python.packages import get_package_prefix
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16MultiArray
from ackermann_msgs.msg import AckermannDriveStamped

import os
from geometry_msgs.msg import PoseStamped

from ros2_numpy import to_ackermann
from nav_msgs.msg import Path
from rclpy.qos import qos_profile_sensor_data  # Quality of Service settings for real-time data
from smart_parking.utils import get_mapping, replay_bagfile

from odometry.call_service import call_reset_odometry_service  

class AutonomousParkingNode(Node):
    def __init__(self, bagfile_path, frame='USS_SRF'):
        super().__init__('autonomous_parking_node')

        if not os.path.exists(bagfile_path):
            raise FileNotFoundError(f"Bagfile not found: {bagfile_path}")

        self.bagfile_path = bagfile_path

        # Define Quality of Service (QoS) for communication
        qos_profile = qos_profile_sensor_data  # Suitable for sensor data
        qos_profile.depth = 1  # Keep only the latest message

        # --- ROS Subscribers ---
        self.create_subscription(PoseStamped, '/position', self.odometry_callback, qos_profile)
        self.create_subscription(Int16MultiArray, '/uss_sensors', self.uss_callback, qos_profile)
        self.create_subscription(AckermannDriveStamped, '/rc/ackermann_cmd', self.ackermann_callback, qos_profile)
        self.create_subscription(Joy, '/rc/joy', self.joy_callback, qos_profile)
        
        # --- ROS Publisher ---
        self.autonomous_pub = self.create_publisher(AckermannDriveStamped, '/autonomous/ackermann_cmd', qos_profile)

        # --- Parking Logic Parameters ---
        self.DEADMAN = 0
        self.AUTONOMOUS = 1
        self.MANUAL = 2

        self.names = {self.DEADMAN: 'Deadman', self.AUTONOMOUS: 'Autonomous', self.MANUAL: 'Manual'}

        self.SCANNING = 0
        self.POSITIONING = 1
        self.PARKING = 2

        self.BLOCKED = 0
        self.OPEN = 1

        self.mode_btn = 0

        self.speed_limit = 0.5
        self.dist_open = 0.35
        self.dist_blocked = 0.1
        self.min_number = 3
        self.parking_length = 0.7
        self.forward_adjustment = 0.35

        self.frame2index = get_mapping()

        self.uss_index = self.frame2index[frame]

        self.init_mapping()

        # Default mode and state based on initial conditions
        self.mode = None
        self.status = self.SCANNING
        self.zone = self.BLOCKED

        self.brake = lambda: self.autonomous_pub.publish(to_ackermann(.0, .0)) 
        
        self.get_logger().info("Autonomous Parking Node has been started.")


            
    def odometry_callback(self, pose_msg: PoseStamped):
        # TODO: Extract the x-position from the pose message and store it in self.xpos
        self.xpos = 0.0

        
    def uss_callback(self, msg: Int16MultiArray):
        # TODO: Convert the selected sensor reading to meters using self.uss_index
        # Ignore the value if it's invalid (e.g., negative distance)
        # Call self.update_state with the processed distance
        distance = 0.0
        self.update_state(distance)


    def ackermann_callback(self, msg: AckermannDriveStamped):
        # Only process if in autonomous scanning mode.
        if self.mode != self.AUTONOMOUS or self.status == self.PARKING:
            return

        msg.drive.speed = min(msg.drive.speed, self.speed_limit)
        self.autonomous_pub.publish(msg)


    def joy_callback(self, joy_msg: Joy):

        # Update the driving mode based on joystick input.
        mode = joy_msg.buttons[self.mode_btn]

        if self.mode != mode: # mode changed
            self.mode = mode           
            self.get_logger().info(f"Switched to {mode}: {self.names[mode]} mode!")

            if mode == self.AUTONOMOUS:
                self.init_mapping()


    def update_state(self, distance):

        # This method handles the transition between parking states based on USS distance readings.
        # The robot can be in one of the following states:
        # - SCANNING: looking for a valid parking spot
        # - POSITIONING: driving forward to align with the start of the open space
        # - PARKING: performing the parking maneuver (triggered by a bag replay)
        
        # TODO: Analyze recent distance readings to determine the current zone (BLOCKED or OPEN)
        # and update the mapping log accordingly.

        if self.status == self.SCANNING:
            if self.zone == self.OPEN:
                length = self.measure_parking_length()
                # Measure the length of the open space and check if it's long enough.
                # If yes, compute the target position and switch to POSITIONING state.

        # If target position is reached,
        # stop the vehicle and start the parking maneuver.
        if self.status == self.POSITIONING:
            if self.xpos >= self.target_position:
                self.get_logger().info(f"Parking position x = {self.target_position:.3f} reached!")
                self.brake()
                self.status = self.PARKING
                replay_bagfile(self.bagfile_path)


    def measure_parking_length(self):
        # TODO: Calculate the length of the most recent open space by measuring
        # the x-position range since the last zone change.
        return 0.0

    def init_mapping(self):
        # TODO: Set the starting x-position to zero.
        # Create a fixed-size buffer to store recent distance measurements.
        # Initialize the mapping array to log zone, x-position, and distance.
        self.status = self.SCANNING
        call_reset_odometry_service(self) # Reset odometry position to zero
        self.get_logger().info("Mapping and state initialized.")


def main(args=None):
    rclpy.init(args=args)

    pkg_dir = get_package_prefix('smart_parking').replace('install', 'src') #  /mxck2_ws/install/smart_parking → /mxck2_ws/src/smart_parking
    bagfile_path = pkg_dir + '/bagfiles/reverse_parallel_parking/reverse_parallel_parking_mcap_0.mcap'
    
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