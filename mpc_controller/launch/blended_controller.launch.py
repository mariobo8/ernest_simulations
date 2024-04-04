import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='mpc_controller',
            executable='pid_node',
            name='pid_node'
        ),
        Node(
            package='mpc_controller',
            executable='mpc_node_blended',
            name='mpc_node_blended'
        ),
        Node(
            package='mpc_controller',
            executable='sensors_node',
            name='sensors_node'
        ),
    ])
