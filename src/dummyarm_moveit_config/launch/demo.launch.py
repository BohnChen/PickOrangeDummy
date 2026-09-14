"""
MoveIt 2 纯仿真演示 Launch 文件 (无需连接真机)
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml

def load_yaml(package_name, file_path):
    pkg_path = get_package_share_directory(package_name)
    abs_path = os.path.join(pkg_path, file_path)
    with open(abs_path, 'r') as f:
        return yaml.safe_load(f)

def load_file(package_name, file_path):
    pkg_path = get_package_share_directory(package_name)
    abs_path = os.path.join(pkg_path, file_path)
    with open(abs_path, 'r') as f:
        return f.read()

def generate_launch_description():
    desc_pkg = 'dummyarm_cad_description'
    moveit_pkg = 'dummyarm_moveit_config'

    desc_path = get_package_share_directory(desc_pkg)
    moveit_path = get_package_share_directory(moveit_pkg)

    xacro_file = os.path.join(desc_path, 'urdf', 'Dummy_URDF.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    robot_description_semantic = load_file(moveit_pkg, 'config/Dummy_URDF.srdf')
    kinematics_yaml = load_yaml(moveit_pkg, 'config/kinematics.yaml')
    joint_limits_yaml = load_yaml(moveit_pkg, 'config/joint_limits.yaml')
    ompl_planning_yaml = load_yaml(moveit_pkg, 'config/ompl_planning.yaml')
    controllers_yaml = os.path.join(moveit_path, 'config', 'ros2_controllers.yaml')
    moveit_controllers_yaml = load_yaml(moveit_pkg, 'config/moveit_controllers.yaml')
    rviz_config = os.path.join(moveit_path, 'config', 'moveit.rviz')

    # Planning pipeline parameters (namespaced properly for MoveIt 2 in ROS 2 Jazzy)
    planning_pipeline_params = {
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
        'ompl': ompl_planning_yaml,
    }

    # Planning context
    move_group_params = [
        {'robot_description': robot_description},
        {'robot_description_semantic': robot_description_semantic},
        {'robot_description_kinematics': kinematics_yaml},
        {'robot_description_planning': joint_limits_yaml},
        {'robot_description_planning.default_velocity_scaling_factor': 1.0},
        {'robot_description_planning.default_acceleration_scaling_factor': 1.0},
        planning_pipeline_params,
        moveit_controllers_yaml,
        {'trajectory_execution.allowed_execution_duration_scaling': 3.0},
        {'trajectory_execution.allowed_goal_duration_margin': 10.0},
        {'trajectory_execution.allowed_start_tolerance': 0.0},  # 0.0 禁用起点硬校验，彻底杜绝 start point deviates 误报中止
        {'publish_planning_scene': True},
        {'publish_geometry_updates': True},
        {'publish_state_updates': True},
        {'publish_transforms_updates': True},
    ]

    # Move Group Node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=move_group_params,
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # Controller Manager (with Mock hardware defined in CAD xacro)
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description}, controllers_yaml],
        output='screen',
    )

    # Controller Spawner (sequential activation to prevent race condition)
    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            'joint_trajectory_controller',
            '--controller-manager', '/controller_manager',
            '--param-file', controllers_yaml,
        ],
        output='screen',
    )

    # RViz2 with MoveIt plugin
    rviz_parameters = [
        {'robot_description': robot_description},
        {'robot_description_semantic': robot_description_semantic},
        {'robot_description_kinematics': kinematics_yaml},
        {'robot_description_planning': joint_limits_yaml},
        {'robot_description_planning.default_velocity_scaling_factor': 1.0},
        {'robot_description_planning.default_acceleration_scaling_factor': 1.0},
        planning_pipeline_params,
    ]

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=rviz_parameters,
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher,
        ros2_control_node,
        controller_spawner,
        move_group_node,
        rviz_node,
    ])
