"""dummyarm 数字样机显示: robot_state_publisher + 手动滑块 + RViz。"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('dummyarm_description')
    xacro_file = os.path.join(pkg, 'urdf', 'dummyarm.xacro')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': ParameterValue(
                Command(['xacro ', xacro_file]), value_type=str)}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(LaunchConfiguration('gui')),
            parameters=[{'zeros.Joint1': 0.0, 'zeros.Joint2': 2.844887, 'zeros.Joint3': -3.141593,
                         'zeros.Joint4': 0.0, 'zeros.Joint5': 0.0, 'zeros.Joint6': 0.0}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg, 'config', 'display.rviz')],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
