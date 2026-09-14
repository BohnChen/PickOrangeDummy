import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 静态手眼外参 (Eye-to-Hand 直接建立 base_link 到相机光学中心的唯一变换)
    # camera_color_optical_frame: Z轴向前(光轴), X轴向右, Y轴向下
    cam_x_arg = DeclareLaunchArgument('cam_x', default_value='0.40', description='Camera Optical Frame X relative to base_link')
    cam_y_arg = DeclareLaunchArgument('cam_y', default_value='0.0', description='Camera Optical Frame Y relative to base_link')
    cam_z_arg = DeclareLaunchArgument('cam_z', default_value='0.30', description='Camera Optical Frame Z relative to base_link')
    cam_roll_arg = DeclareLaunchArgument('cam_roll', default_value='-2.094', description='Camera Roll')
    cam_pitch_arg = DeclareLaunchArgument('cam_pitch', default_value='0.0', description='Camera Pitch')
    cam_yaw_arg = DeclareLaunchArgument('cam_yaw', default_value='1.5708', description='Camera Yaw')
    gui_arg = DeclareLaunchArgument('gui', default_value='true', description='Launch local rqt_image_view GUI')

    # 1. 唯一且清晰的静态 TF: 直接广播 base_link -> camera_color_optical_frame
    # 彻底杜绝 camera_link 与 optical_frame 两套坐标轴重叠混乱
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_broadcaster',
        arguments=[
            '--x', LaunchConfiguration('cam_x'),
            '--y', LaunchConfiguration('cam_y'),
            '--z', LaunchConfiguration('cam_z'),
            '--roll', LaunchConfiguration('cam_roll'),
            '--pitch', LaunchConfiguration('cam_pitch'),
            '--yaw', LaunchConfiguration('cam_yaw'),
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_color_optical_frame'
        ]
    )

    # 2. 包含官方 realsense2_camera 启动文件 (零拷贝内存共享，适配本地显示)
    rs_launch_path = os.path.join(
        get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')

    config_path = os.path.join(
        get_package_share_directory('dummyarm_perception'), 'config', 'd435i.yaml')

    realsense_camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rs_launch_path),
        launch_arguments={
            'config_file': config_path,
            'align_depth.enable': 'true',
            'rgb_camera.color_profile': '640x480x30',
            'depth_module.depth_profile': '640x480x30',
            'enable_sync': 'true',
        }.items()
    )

    # 3. 柑橘实时检测与 3D 深度反投影节点
    orange_detector_node = Node(
        package='dummyarm_perception',
        executable='orange_detector_node',
        name='orange_detector_node',
        parameters=[{
            'color_topic': '/camera/camera/color/image_raw',
            'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',
            'filter_window_size': 5,
            'stability_threshold_m': 0.015,
        }],
        output='screen'
    )

    # 4. 本地轻量原生图像视窗 (rqt_image_view 纯 Qt/GTK 渲染，无 Ogre 冲突)
    rqt_image_view_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_view',
        arguments=['/dummyarm/vision_debug_image'],
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui')),
        output='screen'
    )

    return LaunchDescription([
        cam_x_arg,
        cam_y_arg,
        cam_z_arg,
        cam_roll_arg,
        cam_pitch_arg,
        cam_yaw_arg,
        gui_arg,
        static_tf_node,
        realsense_camera_node,
        orange_detector_node,
        rqt_image_view_node,
    ])
