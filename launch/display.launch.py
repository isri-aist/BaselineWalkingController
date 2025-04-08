# Standard library
import os

# External library
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction


def launch_setup(context, *args, **kwargs):
    run_rviz = context.launch_configurations["run_rviz"]

    rviz_config_file = os.path.join(
        get_package_share_directory('baseline_walking_controller'),
        'rviz',
        'display.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(run_rviz)
    )

    return [
        rviz_node
    ]


def generate_launch_description():
    """! Generate launch description
    """
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "run_rviz",
            default_value="True",
            description="Run rviz",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)])