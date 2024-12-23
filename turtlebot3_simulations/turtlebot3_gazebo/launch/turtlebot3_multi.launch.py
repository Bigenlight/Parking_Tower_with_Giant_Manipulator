#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    # Directories
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Launch configurations for the first robot
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose1 = LaunchConfiguration('x_pose1', default='-2.0')
    y_pose1 = LaunchConfiguration('y_pose1', default='-0.5')
    #z_pose1 = LaunchConfiguration('z_pose1', default='0.0')

    # Launch configurations for the second robot
    x_pose2 = LaunchConfiguration('x_pose2', default='5.0')
    y_pose2 = LaunchConfiguration('y_pose2', default='0.5')
    #z_pose2 = LaunchConfiguration('z_pose2', default='0.0')

    # World file
    world = os.path.join(pkg_turtlebot3_gazebo, 'worlds', 'car.world')

    # Gazebo server and client
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py'))
    )

    # Robot State Publisher for first robot
    robot_state_publisher_1 = GroupAction([
        PushRosNamespace('robot1'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_turtlebot3_gazebo, 'launch', 'robot_state_publisher.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_turtlebot3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')),
            launch_arguments={
                'x_pose': x_pose1,
                'y_pose': y_pose1,
                #'z_pose': z_pose1,
                'robot_model': 'waffle_pi'
            }.items()
        )
    ])

    # Robot State Publisher for second robot
    robot_state_publisher_2 = GroupAction([
        PushRosNamespace('robot2'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_turtlebot3_gazebo, 'launch', 'robot_state_publisher.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_turtlebot3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')),
            launch_arguments={
                'x_pose': x_pose2,
                'y_pose': y_pose2,
                #'z_pose': z_pose2,
                'robot_model': 'waffle_pi'
            }.items()
        )
    ])

    # Launch description
    ld = LaunchDescription()

    # Add Gazebo server and client
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)

    # Add both robots
    ld.add_action(robot_state_publisher_1)
    ld.add_action(robot_state_publisher_2)

    return ld
