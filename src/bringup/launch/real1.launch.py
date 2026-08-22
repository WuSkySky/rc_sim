import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_pkg = get_package_share_directory('bringup')
    interfaces_pkg = get_package_share_directory('robot_r2_interfaces')
    odin_driver_pkg = get_package_share_directory('odin_ros_driver')
    odin_data_postprocess_pkg = get_package_share_directory(
        'odin_data_postprocess')
    serial_pkg = get_package_share_directory('serial_pkg')
    control_pkg = get_package_share_directory('robot_r2_control')
    detect_pkg = get_package_share_directory('robot_r2_detect')
    # ArUco 识别对接当前不在 real1 启动，以下变量保留待恢复：
    # aruco_pkg = get_package_share_directory('robot_r2_aruco')
    # controller_pkg = get_package_share_directory('robot_r2_controller')
    fastdds_profile = os.path.join(
        interfaces_pkg,
        'config',
        'fastdds_camera.xml',
    )

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                bringup_pkg,
                'launch',
                'control.launch.py',
            )
        ),
        launch_arguments={
            'kfs_get_type_service': LaunchConfiguration(
                'kfs_get_type_service'),
            'enable_stage_one': 'true',
        }.items(),
    )

    odin_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                odin_driver_pkg,
                'launch',
                'odin1_ros2_no_rviz.launch.py',
            )
        )
    )

    camera_frame_config = os.path.join(
        odin_data_postprocess_pkg,
        'config',
        'camera_frame_postprocess.yaml',
    )
    camera_frame_postprocess = Node(
        package='odin_data_postprocess',
        executable='camera_frame_postprocess',
        parameters=[camera_frame_config],
        output='screen',
    )

    led_detect_config = os.path.join(
        detect_pkg,
        'config',
        'led_detect.yaml',
    )
    led_detect = Node(
        package='robot_r2_detect',
        executable='led_detect',
        name='led_detect',
        parameters=[led_detect_config],
        remappings=[
            ('/r2/led_detection/image', '/r2/rear_camera/image_raw'),
        ],
        output='screen',
    )

    odin_odometry_config = os.path.join(
        odin_data_postprocess_pkg,
        'config',
        'odometry_postprocess.yaml',
    )
    odin_odometry_postprocess = Node(
        package='odin_data_postprocess',
        executable='odometry_postprocess',
        parameters=[odin_odometry_config],
        remappings=[
            ('/r2/pose_feedback', '/r2/pose_feedback_odin'),
            ('/r2/set_base_pose', '/r2/set_base_pose_odin'),
        ],
        output='screen',
    )

    serial_bridge_config = os.path.join(
        serial_pkg,
        'config',
        'serial_bridge.yaml',
    )
    serial_bridge = Node(
        package='serial_pkg',
        executable='serial_bridge',
        parameters=[
            serial_bridge_config,
            {'receive_feedback_enabled': True},
        ],
        output='screen',
    )

    all_step_config = os.path.join(
        control_pkg,
        'config',
        'all_step_control.yaml',
    )
    all_step_control = Node(
        package='robot_r2_control',
        executable='all_step_control',
        parameters=[all_step_config],
        output='screen',
    )

    odometry_tf_config = os.path.join(
        serial_pkg,
        'config',
        'odometry_tf.yaml',
    )
    odometry_tf = Node(
        package='serial_pkg',
        executable='odometry_tf',
        parameters=[odometry_tf_config],
        output='screen',
    )

    alignment_config = os.path.join(
        control_pkg,
        'config',
        'alignment.yaml',
    )
    tip_alignment_config = os.path.join(
        control_pkg,
        'config',
        'tip_alignment.yaml',
    )
    kfs_alignment = Node(
        package='robot_r2_control',
        executable='alignment',
        name='kfs_alignment',
        parameters=[alignment_config],
        remappings=[
            ('/r2/alignment/detection', '/r2/kfs/roi'),
            ('/r2/alignment/cmd_vel', '/r2/cmd_vel'),
            ('/r2/alignment/align', '/r2/align_to_kfs'),
        ],
        output='screen',
    )
    tip_alignment = Node(
        package='robot_r2_control',
        executable='alignment',
        name='tip_alignment',
        parameters=[alignment_config, tip_alignment_config],
        remappings=[
            ('/r2/alignment/detection', '/r2/tip/roi'),
            ('/r2/alignment/cmd_vel', '/r2/cmd_vel'),
            ('/r2/alignment/align', '/r2/align_to_tip'),
        ],
        output='screen',
    )

    # ==================================================================
    # ArUco 二维码检测与专用底盘伺服（基于端头 MIPI 相机）均已注释停用，
    # 不在 real1 启动。需要恢复时取消本段及下方 LaunchDescription 中两行引用
    # 与 enable_aruco 参数声明的注释即可。
    #
    # aruco_subscribe.launch.py 启动三个节点：
    #   - aruco_detect      : 订阅端头相机图像，检测 ArUco 码并估计 6-DOF
    #                         位姿，广播 TF camera -> marker_<id>，发布
    #                         /r2/aruco/detections（调试图 /r2/aruco/debug）
    #   - aruco_camera_tf   : 按 YAML 外参发布静态 TF base_link -> camera，
    #                         使 TF 树 base_link -> camera -> marker_<id>
    #                         保持连通
    #   - aruco_pose_bridge : 查询 TF marker_<id> -> base_link（底盘在
    #                         marker 坐标系中的位姿），发布
    #                         /r2/aruco/pose_feedback
    #
    # marker 固定在静止的 R1 机器人上，其坐标系作为一次性对接的"世界"
    # 参考系。
    # ==================================================================
    # aruco_pipeline = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             aruco_pkg,
    #             'launch',
    #             'aruco_subscribe.launch.py',
    #         )
    #     ),
    #     launch_arguments={
    #         'image_topic': '/r2/tip_camera/image_raw',
    #         'camera_info_topic': '/r2/tip_camera/camera_info',
    #     }.items(),
    #     condition=IfCondition(LaunchConfiguration('enable_aruco')),
    # )
    #
    # # ArUco 专用底盘位置伺服：chassis_pose_servo 的独立实例，闭环来源为
    # # /r2/aruco/pose_feedback（marker 坐标系下的底盘位姿），对外提供
    # # /r2/aruco/move_to_pose 与 /r2/aruco/move_relative 一次性对接服务，
    # # 与常规 serial/Odin 位置伺服（/r2/move_to_pose 等）互不干扰。
    # aruco_servo_config = os.path.join(
    #     controller_pkg,
    #     'config',
    #     'aruco_chassis_pose_servo.yaml',
    # )
    # aruco_chassis_pose_servo = Node(
    #     package='robot_r2_controller',
    #     executable='chassis_pose_servo',
    #     name='aruco_chassis_pose_servo',
    #     parameters=[aruco_servo_config],
    #     remappings=[
    #         ('/r2/pose_feedback', '/r2/aruco/pose_feedback'),
    #         ('/r2/move_to_pose', '/r2/aruco/move_to_pose'),
    #         ('/r2/move_relative', '/r2/aruco/move_relative'),
    #     ],
    #     condition=IfCondition(LaunchConfiguration('enable_aruco')),
    #     output='screen',
    # )

    return LaunchDescription([
        SetEnvironmentVariable(
            'RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
        SetEnvironmentVariable(
            'RMW_FASTRTPS_USE_QOS_FROM_XML', '1'),
        SetEnvironmentVariable(
            'FASTDDS_DEFAULT_PROFILES_FILE', fastdds_profile),
        SetEnvironmentVariable(
            'FASTRTPS_DEFAULT_PROFILES_FILE', fastdds_profile),
        DeclareLaunchArgument(
            'kfs_get_type_service',
            default_value='/r2/detection/get_type',
            description='Remote KFS detection service used by control',
        ),
        # 与 ArUco 识别对接配套的开关，停用期间一并注释：
        # DeclareLaunchArgument(
        #     'enable_aruco',
        #     default_value='true',
        #     description='Start tip-camera ArUco detection and chassis servo',
        # ),
        odin_launch,
        camera_frame_postprocess,
        led_detect,
        odin_odometry_postprocess,
        control_launch,
        serial_bridge,
        all_step_control,
        odometry_tf,
        kfs_alignment,
        tip_alignment,
        # ArUco 识别对接（当前停用）：
        # aruco_pipeline,
        # aruco_chassis_pose_servo,
    ])
