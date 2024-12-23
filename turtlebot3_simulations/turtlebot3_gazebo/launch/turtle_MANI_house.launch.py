#!/usr/bin/env python3
#
# ...
#

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='-0.5')
    z_pose = LaunchConfiguration('z_pose', default='1.0')  # Note: was previously 'z_pose' using 'y_pose' value

    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'turtlebot3_house.world'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'z_pose': z_pose
        }.items()
    )

    # Path to your manipulator URDF (xacro)
    manipulator_urdf = os.path.join(
        get_package_share_directory('arduinobot_description'),
        'urdf',
        'arduinobot.urdf.xacro'
    )

    # Publish manipulator's robot_description parameter
    manipulator_robot_description = ParameterValue(
        Command(['xacro ', manipulator_urdf]),
        value_type=str
    )

    manipulator_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='manipulator_state_publisher',
        parameters=[{'robot_description': manipulator_robot_description}]
    )

    # Spawn the manipulator far away (for example, at x=100, y=0, z=0)
    spawn_manipulator_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'arduinobot',           # name of the manipulator
            '-topic', 'robot_description',     # manipulator robot description topic
            '-x', '0', '-y', '70', '-z', '0'  # spawn location
        ],
        output='screen'
    )

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(manipulator_state_publisher)
    ld.add_action(spawn_manipulator_cmd)

    return ld
