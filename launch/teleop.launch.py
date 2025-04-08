import os

from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions


def generate_launch_description():
    joy_config = launch.substitutions.LaunchConfiguration('joy_config')
    joy_dev = launch.substitutions.LaunchConfiguration('joy_dev')
    publish_stamped_twist = launch.substitutions.LaunchConfiguration('publish_stamped_twist')
    config_filepath = launch.substitutions.LaunchConfiguration('config_filepath')

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('joy_vel', default_value='cmd_vel'),
        launch.actions.DeclareLaunchArgument('joy_config', default_value='ps3', description='Joystick configuration'),
        launch.actions.DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0', description='Joystick device path'),
        launch.actions.DeclareLaunchArgument('publish_stamped_twist', default_value='false'),
        launch.actions.DeclareLaunchArgument('config_filepath', default_value=[
            launch.substitutions.TextSubstitution(text=os.path.join(
                get_package_share_directory('teleop_twist_joy'), 'config', '')),
            joy_config, launch.substitutions.TextSubstitution(text='.config.yaml')]),

        launch_ros.actions.Node(
            package='joy', executable='joy_node', name='joy_node',
            parameters=[{
                'device_id': joy_dev,
                'dev': launch.substitutions.LaunchConfiguration('joy_dev'),
                'deadzone': 0.1,
                'autorepeat_rate': 20
            }]),
        launch_ros.actions.Node(
            package='teleop_twist_joy', executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[{
                'axis_linear': {'x': 1, 'y': 0},
                'scale_linear': {'x': 0.15, 'y': 0.1},
                'scale_linear_turbo': {'x': 0.3, 'y': 0.2},
                'axis_angular': {'yaw': 2},
                'scale_angular': {'yaw': 0.1},
                'scale_angular_turbo': {'yaw': 0.2},
                'enable_button': 4,
                'enable_turbo_button': 5
            }, {'publish_stamped_twist': publish_stamped_twist}],
            remappings={('/cmd_vel', launch.substitutions.LaunchConfiguration('joy_vel'))},
            ),
    ])