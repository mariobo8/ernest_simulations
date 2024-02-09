import launch
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    path_tracking_mpc = Node(
    package='controller',  # replace with your actual package name
    executable='MPC_node',  # replace with your actual node executable name
    name='PathTrackingMPC',  # replace with your desired node name
    output='screen',
    )
        
    return LaunchDescription([
        path_tracking_mpc,
    ])
