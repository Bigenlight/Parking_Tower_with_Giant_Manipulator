import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition, IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="True"
    )

    use_python_arg = DeclareLaunchArgument(
        "use_python",
        default_value="False",
    )

    use_python = LaunchConfiguration("use_python")
    is_sim = LaunchConfiguration("is_sim")

    # -- If not using Python, launch the C++ version of task_server --
    task_server_node = Node(
        package="arduinobot_remote",
        executable="task_server_node",
        condition=UnlessCondition(use_python),
        parameters=[{"use_sim_time": is_sim}],
        output="screen"
    )

    # -- If using Python, launch the Python version of task_server --
    task_server_node_py = Node(
        package="arduinobot_remote",
        executable="task_server.py",
        condition=IfCondition(use_python),
        parameters=[{"use_sim_time": is_sim}],
        output="screen"
    )

    # -- Build MoveIt configs if needed --
    moveit_config = (
        MoveItConfigsBuilder("arduinobot", package_name="arduinobot_moveit")
        .robot_description(
            file_path=os.path.join(
                get_package_share_directory("arduinobot_description"),
                "urdf",
                "arduinobot.urdf.xacro"
            )
        )
        .robot_description_semantic(file_path="config/arduinobot.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .moveit_cpp(file_path="config/planning_python_api.yaml")
        .to_moveit_configs()
    )

    # -- Always launch the manipulator_manager (mani_manager) from arduinobot_py_example --
    manipulator_manager_node = Node(
        package="arduinobot_py_example",
        executable="mani_manager",    # The entry point you created
        output="screen",
        parameters=[
            moveit_config.to_dict(),  # If you want it to also have the same MoveIt parameters
            {"use_sim_time": is_sim}
        ]
    )

    return LaunchDescription([
        is_sim_arg,
        use_python_arg,
        task_server_node,
        task_server_node_py,
        manipulator_manager_node
    ])
