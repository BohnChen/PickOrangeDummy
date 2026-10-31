"""整车静态 TF + 机械臂 robot_state_publisher。

设计要点（见 docs/architecture_v2.md §9.2）：
  - 所有静态变换参数来自 config/transforms.yaml，**不写死在 launch 里**
  - base_link 沿用现有 URDF 的根，MoveIt 配置零改动
  - odom -> base_footprint 先发单位变换占位

用法：
    ros2 launch pickorange_bringup tf.launch.py
    ros2 launch pickorange_bringup tf.launch.py with_arm:=false   # 只看车和相机
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _load_frames(path):
    with open(path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    # 顶层每个 key 是一组变换；跳过非 dict 的
    return [v for v in cfg.values() if isinstance(v, dict) and "parent" in v]


def generate_launch_description():
    bringup_share = get_package_share_directory("pickorange_bringup")
    desc_share = get_package_share_directory("dummyarm_description")

    transforms_yaml = os.path.join(bringup_share, "config", "transforms.yaml")
    xacro_file = os.path.join(desc_share, "urdf", "dummyarm.xacro")

    with_arm = LaunchConfiguration("with_arm")

    static_tfs = []
    for fr in _load_frames(transforms_yaml):
        xyz = [str(v) for v in fr["xyz"]]
        rpy = [str(v) for v in fr["rpy"]]
        static_tfs.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name=f"tf_{fr['parent']}_to_{fr['child']}",
                arguments=[
                    "--x", xyz[0], "--y", xyz[1], "--z", xyz[2],
                    "--roll", rpy[0], "--pitch", rpy[1], "--yaw", rpy[2],
                    "--frame-id", fr["parent"],
                    "--child-frame-id", fr["child"],
                ],
                output="log",
            )
        )

        # 相机 optical frame：ROS 光学系（z 前、x 右、y 下）
        # 由 base 系绕 X 轴 -90° 得到，再串到 camera_link
        optical = fr.get("optical_frame")
        if optical:
            static_tfs.append(
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name=f"tf_{fr['child']}_to_{optical}",
                    arguments=[
                        "--roll", "-1.5707963267948966",
                        "--pitch", "0", "--yaw", "-1.5707963267948966",
                        "--frame-id", fr["child"],
                        "--child-frame-id", optical,
                    ],
                    output="log",
                )
            )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": ParameterValue(
                Command(["xacro ", xacro_file]), value_type=str),
        }],
        condition=IfCondition(with_arm),
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="log",
        condition=IfCondition(with_arm),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "with_arm", default_value="true",
            description="是否同时发布机械臂 URDF（机器人状态）"),
        *static_tfs,
        robot_state_publisher,
        joint_state_publisher,
    ])
