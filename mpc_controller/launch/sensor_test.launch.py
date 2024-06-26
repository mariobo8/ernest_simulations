from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mpc_controller',
            executable='pid_node',
            name='pid_node'
        ),
        
        Node(
            package='mpc_controller',
            executable='sensors_node',
            name='sensors_node'
        ),
    ])