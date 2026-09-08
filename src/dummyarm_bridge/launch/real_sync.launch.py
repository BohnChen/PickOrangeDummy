"""真机联动: bridge(关节状态+管理服务) + robot_state_publisher + RViz。"""
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
    pkg_bridge = get_package_share_directory('dummyarm_bridge')
    xacro_file = os.path.join(pkg_desc, 'urdf', 'dummyarm.xacro')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('port', default_value='/dev/ttyACM1'),
        Node(
            package='dummyarm_bridge',
            executable='dummyarm_system',
            output='screen',
            parameters=[{'port': LaunchConfiguration('port')}],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': ParameterValue(
                Command(['xacro ', xacro_file]), value_type=str)}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg_desc, 'config', 'display.rviz')],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
