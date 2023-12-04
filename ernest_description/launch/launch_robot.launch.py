import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix

import xacro


def generate_launch_description():

    #pkg_box_bot_gazebo = get_package_share_directory('asterix_gazebo')
    description_package_name = "ernest_description"
    install_dir = get_package_prefix(description_package_name)

    # This is to find the models inside the models folder in my_box_bot_gazebo package
    #gazebo_models_path = os.path.join(pkg_box_bot_gazebo, 'models')
    #if 'GAZEBO_MODEL_PATH' in os.environ:
    #    os.environ['GAZEBO_MODEL_PATH'] = os.environ['GAZEBO_MODEL_PATH'] + \
    #        ':' + install_dir + '/share' + ':' + gaz#ebo_models_path
    #else:
    #    os.environ['GAZEBO_MODEL_PATH'] = install_dir + \
    #        "/share" + ':' + gazebo_models_path
#
    #if 'GAZEBO_PLUGIN_PATH' in os.environ:
    #    os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + \
    #        ':' + install_dir + '/lib'
    #else:
    #    os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'
#
    #print("GAZEBO MODELS PATH=="+str(os.environ["GAZEBO_MODEL_PATH"]))
    #print("GAZEBO PLUGINS PATH=="+str(os.environ["GAZEBO_PLUGIN_PATH"])) 
#
    #gazebo = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource([os.path.join(
    #        get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
    #    launch_arguments={"verbose": "false", 'pause': 'true'}.items(),
    #)

    robot_model_path = os.path.join(
        get_package_share_directory('ernest_description'))

    xacro_file = os.path.join(robot_model_path, 'urdf', 'ernest.urdf.xacro')

    # convert XACRO file into URDF
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

#    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
#                        arguments=['-entity', 'asterix', 
#                                   '-topic', 'robot_description'],
#                        output='screen')

 #   rviz_config_file = PathJoinSubstitution(
 #       [FindPackageShare(description_package), "config", "stm_view.rviz"]
 #   )


    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
       # arguments=["-d", rviz_config_file],
    )

    return LaunchDescription([
        #gazebo,
        robot_state_publisher,
        #spawn_entity
        rviz_node,
    ])