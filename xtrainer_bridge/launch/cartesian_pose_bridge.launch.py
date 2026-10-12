from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arm_namespace = LaunchConfiguration('arm_namespace')
    command_type = LaunchConfiguration('command_type')

    return LaunchDescription([
        DeclareLaunchArgument(
            'arm_namespace', default_value='Arm1',
            description='Dobot arm namespace, e.g. Arm1 or Arm2'),
        DeclareLaunchArgument(
            'command_type', default_value='PoseStamped',
            description='Pose or PoseStamped'),
        Node(
            package='xtrainer_bridge',
            executable='cartesian_pose_bridge',
            name='cartesian_pose_bridge',
            namespace=arm_namespace,
            output='screen',
            parameters=[{'command_type': command_type}],
        ),
    ])
