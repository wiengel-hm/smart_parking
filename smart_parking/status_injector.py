import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data  # Quality of Service settings for real-time data

class StatusInjectorNode(Node):
    def __init__(self):
        super().__init__('status_injector_node')
        
        # Define Quality of Service (QoS) matching the original sensor stream
        qos_profile = qos_profile_sensor_data  # Suitable for sensor/real-time data
        qos_profile.depth = 1                  # Keep only the latest message
        
        # Subscriber to watch the bagfile playback
        self.create_subscription(
            AckermannDriveStamped, 
            '/rc/ackermann_cmd', 
            self.cmd_callback, 
            qos_profile
        )
        
        # Publisher to output the enriched status
        self.status_pub = self.create_publisher(String, '/parking/status', qos_profile)
        self.get_logger().info("Status Injector Node is ready with Sensor QoS.")

    def cmd_callback(self, msg):
        # Every time an Ackermann command plays from the bag, inject a status message
        status_msg = String()
        status_msg.data = 'PARKING_IN'
        
        # ROS2 automatically assigns the bagfile's timestamp here because use_sim_time is True
        self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = StatusInjectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()