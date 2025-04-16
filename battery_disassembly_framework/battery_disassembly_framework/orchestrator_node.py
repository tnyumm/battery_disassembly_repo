import rclpy    # ROS 2 Python client library
from rclpy.node import Node     # Base class for ROS 2 nodes
from std_msgs.msg import String     # Message type for publishing classifications

# This node listens to classifications from the battery_classifier node
# Based on the label, it decides where the arm should place the battery

# Define a node class to orchestrate battery sorting
class OrchestratorNode(Node):

    # Constructor method
    def __init__(self):

        super().__init__('orchestrator_node')   # Naming the Orchestrator node and call the parent constructor

        # Subscribes the Orchestrator node to a "battery_type" topic
        self.subscription = self.create_subscription(
            String,
            'battery_type',
            self.classify_callback,
            10)
        
        # Creates a publisher for sending arm commands to "arm_command" topic
        self.arm_pub = self.create_publisher(String, 'arm_command', 10)

    # Each time a message is received, self.classify_callback is called
    def classify_callback(self, msg):
        decision = msg.data.lower()     # Normalise message (input) to lowercase
        command = String()  # Prepares theb outgoing command message as a String type

        # Decision logic: Determine the appropriate action based on the battery classification
        if decision == 'safe':
            command.data = 'place_in_bin_a'
        elif decision == 'hazardous':
            command.data = 'place_in_bin_b'
        else:
            command.data = 'discard_to_safe_zone'   # Default action for 'damaged' or unknown battery classifications

        self.arm_pub.publish(command)   # Publishes the decision to the "arm_command" topic
        self.get_logger().info(f"Received: {msg.data} | Sent: {command.data}")      # Log the interaction

# Entry point when running this node as a script
def main(args=None):
    rclpy.init(args=args)   # Initialise ROS
    node = OrchestratorNode()   # Instantiate Orchestrator Node
    rclpy.spin(node)    # Keeps the node running
    rclpy.shutdown()    # Shutdown
