"""真机控制: robot_state_publisher + controller_manager + 控制器spawner (3b)。

安全：启动不使能(!START); 使能/回零走 /dummyarm_system/* 服务;
spawn_trajectory 默认 false (只读); 开启后轨迹执行前请先经管理服务使能与确认。
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_desc = get_package_share_directory('dummyarm_description')
    pkg_bring = get_package_share_directory('dummyarm_bringup')
    xacro_file = os.path.join(pkg_desc, 'urdf', 'dummyarm.xacro')
    controllers_yaml = os.path.join(pkg_bring, 'config', 'dummyarm_controllers.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('spawn_trajectory', default_value='false'),
        DeclareLaunchArgument('include_bridge', default_value='false'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': ParameterValue(
                Command(['xacro ', xacro_file]), value_type=str)}],
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            output='screen',
            parameters=[controllers_yaml],
        ),
        Node(
            package='dummyarm_bridge',
            executable='dummyarm_system',
            output='screen',
            parameters=[{'port': '/dev/ttyACM1'}],
            condition=IfCondition(LaunchConfiguration('include_bridge')),
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '-c', 'controller_manager'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_trajectory_controller', '-c', 'controller_manager'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('spawn_trajectory')),
        ),
    ])
