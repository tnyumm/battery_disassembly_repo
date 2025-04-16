# This launch file starts all three nodes together in a single command
# It's useful to simulate the full workflow in one go

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():  # This function is called by the launch system to define what runs
    return LaunchDescription([
        Node(
            package='vision_classifier',  # This is the package that contains the battery classifier node
            executable='battery_classifier',  # The name of the script to run as a node
            name='battery_classifier'  # Node name (optional, but helps debugging)
        ),
        Node(
            package='battery_disassembly_framework',
            executable='orchestrator_node',
            name='orchestrator_node'
        ),
        Node(
            package='arm_driver',
            executable='arm_driver_node',
            name='arm_driver_node'
        )
    ])
