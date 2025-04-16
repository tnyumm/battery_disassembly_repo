import rclpy    # ROS 2 Python client library
from rclpy.node import Node      # Base class for ROS 2 nodes
from std_msgs.msg import String     # Import the standard String message type for ROS 2 communication.

# This node simulates a robot arm receiving commands.
# It logs the received command to simulate arm action.
class ArmDriverNode(Node):

    # Constructor method that initialises the node when an instance is created.
    def __init__(self):
        super().__init__('arm_driver_node')  # Initialise the parent 'Node' class with the node name 'arm_driver_node'.

        # Create a subscriber to the 'arm_command' topic:
        # - It listens for incoming messages of type String.
        # - When a message is received, it triggers 'command_callback'.
        # - The number 10 indicates the buffer size for incoming messages.
        self.subscription = self.create_subscription(
            String,
            'arm_command',
            self.command_callback,
            10)

    # Define callback method that is called whenever a message arrives on 'arm_command'.
    def command_callback(self, msg):
        self.get_logger().info(f"[SIM] Executing command: {msg.data}")  # Uses ROS 2 built-in logging system to display received command messages on the terminal.

# Entry point when running this script directly.
def main(args=None):
    rclpy.init(args=args)   # Initialises the ROS 2 communication system.
    node = ArmDriverNode()  # Create an instance of the 'ArmDriverNode' node class.
    rclpy.spin(node)    # Keep the node running indefinitely, waiting for incoming messages and callbacks.
    rclpy.shutdown()    # Gracefully shutdown ROS 2 communications when the node is stopped (e.g., via Ctrl+C).