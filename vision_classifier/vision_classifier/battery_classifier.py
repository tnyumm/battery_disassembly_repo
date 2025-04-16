# This node simulates a camera or AI model classifying batteries.
# Every 2 seconds, it publishes a fake classification to a topic.

import rclpy  # ROS 2 Python client library
from rclpy.node import Node  # Base class for ROS 2 nodes
from std_msgs.msg import String  # Message type for publishing classifications
import random  # Used to randomly select battery classification types

class BatteryClassifier(Node):  # Define a ROS 2 node class that inherits from Node
    def __init__(self):  # This special method runs when the class is instantiated (the constructor)
        # 'super()' calls the constructor of the parent class (Node), which sets up the ROS 2 node functionality
        # The argument 'battery_classifier' is the name this node will have in the ROS graph
        super().__init__('battery_classifier')

        # Create a publisher that sends messages of type String to the 'battery_type' topic
        # The queue size of 10 means it can buffer up to 10 messages if the subscriber is slow
        self.publisher = self.create_publisher(String, 'battery_type', 10)

        # This timer calls self.publish_fake_classification() every 2.0 seconds
        # It simulates a stream of battery type classifications
        self.timer = self.create_timer(2.0, self.publish_fake_classification)

    def publish_fake_classification(self):  # Method that is called repeatedly by the timer
        battery_type = random.choice(['safe', 'hazardous', 'damaged'])  # Simulate classification
        msg = String()  # Create a new String message
        msg.data = battery_type  # Assign the chosen classification to the message

        self.publisher.publish(msg)  # Publish the message to the 'battery_type' topic
        # self.get_logger() provides a logger for this node; .info() logs an info-level message to the console
        self.get_logger().info(f"Published: {battery_type}")

def main(args=None):  # Entry point for the node when run as a script
    rclpy.init(args=args)  # Initialize the ROS 2 system
    node = BatteryClassifier()  # Instantiate the BatteryClassifier node class
    rclpy.spin(node)  # Keep the node alive and responsive to events (like timers, subscriptions)
    rclpy.shutdown()  # Shut down cleanly when the node is stopped