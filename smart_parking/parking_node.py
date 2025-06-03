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

        self.speed_limit = 0.4
        self.dist_open = 0.25
        self.dist_blocked = 0.1
        self.min_number = 5
        self.parking_length = 0.7
        self.forward_adjustment = 1.0

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
        self.xpos = pose_msg.pose.position.x

        
    def uss_callback(self, msg: Int16MultiArray):
        # Process USS sensor data.
        measurements = np.array(msg.data) / 100 # to m
        distance = measurements[self.uss_index]
        if distance < 0: return # Ignore invalid (negative) distance readings
        self.distances.append(distance)
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
        
        self.get_logger().info(f"x: {self.xpos:.3f}, d: {distance:.2f}, zone: {self.zone}")
        if self.status == self.SCANNING:

            # Before buffer is full, assume blocked (or placeholder)
            if len(self.distances) < self.min_number:
                self.mapping = np.vstack((self.mapping, [self.BLOCKED, self.xpos, distance]))
                return

            self.zone = self.OPEN if np.mean((np.array(self.distances) >= self.dist_open)) >= 0.5 else self.BLOCKED

            if self.zone == self.OPEN:
                self.mapping = np.vstack((self.mapping, [self.OPEN, self.xpos, distance]))
                self.mapping[-self.min_number:, 0] = self.OPEN  # Overwrite last N entries
                length = self.measure_parking_length(self.mapping)
                

                if length >= self.parking_length:
                    self.status = self.POSITIONING
                    self.target_position = self.xpos + self.forward_adjustment
                    self.get_logger().info(f"Measured parking space length: {length:.2f} meters")
                    self.get_logger().info(f"Initialising parking sequence: adjust position: {self.target_position:.2f}...")
            else:
                # Otherwise, label current measurement as blocked
                self.mapping = np.vstack((self.mapping, [self.BLOCKED, self.xpos, distance]))
        
        # If target position is reached,
        # stop the vehicle and start the parking maneuver.
        if self.status == self.POSITIONING:
            if self.xpos >= self.target_position:
                self.get_logger().info(f"Parking position x = {self.target_position:.3f} reached!")
                self.brake()
                self.status = self.PARKING
                replay_bagfile(self.bagfile_path)


    def measure_parking_length(self, mapping):

        # Find the indices where the parking is NOT open
        idx = np.where(mapping[:,0] != self.OPEN)[0]

        if len(idx) == 0:
            return 0.0
        
        # Get the last index where the spot was NOT open
        last_idx = idx[-1]

        # Extract the x-psotions startinmg right after the last not-open index
        x = mapping[last_idx + 1:, 1]

        # Return the distance between min and max (i.e. parking length)
        return np.ptp(x) if len(x) > 0 else 0.0

    def init_mapping(self):
        # Reset mapping and state when reinitializing.
        self.xpos = 0.0
        self.distances = deque(maxlen=self.min_number)
        self.mapping = np.empty((0, 3))
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