import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    output_bag_file = DeclareLaunchArgument(
        'output_bag_file',
        default_value='~/ros2_bags',
        description='Output bag file name'
    )
    topics = DeclareLaunchArgument(
        'topics',
        default_value='"/joint_states"',
        description='Topics to record'
    )


    return LaunchDescription([
        Node(
            package='mpc_controller',
            executable='pid_node',
            name='pid_node'
        ),
        Node(
            package='mpc_controller',
            executable='mpc_node',
            name='mpc_node'
        ),
        Node(
            package='mpc_controller',
            executable='sensors_node',
            name='sensors_node'
        ),
        #launch.actions.ExecuteProcess(
        ##    cmd=['ros2', 'bag', 'record','-o', '../ros2_bags/prova',
        #          '/joint_states',
        #          '/clock',
        ##          '/steer_controller/commands',
        #          '/wheel_controller/commands',   ],
            #cmd=['ros2', 'bag', 'record','-a','-o', '../ros2_bags/prova'],
        #    output='screen'
        #)
    ])
