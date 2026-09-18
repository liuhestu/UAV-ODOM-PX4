"""Start the local D435i driver and OpenVINS with the RealSense configuration.

The RealSense configuration in ``config/realsense`` subscribes to
``/camera/infra1/image_rect_raw``, ``/camera/infra2/image_rect_raw``, and
``/camera/imu``.  The camera driver therefore runs in the root namespace with
the node name ``camera``; using its package default namespace would instead
publish ``/camera/camera/...`` topics.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution


def generate_launch_description():
    ov_msckf_share = get_package_share_directory("ov_msckf")
    realsense_share = get_package_share_directory("realsense2_camera")

    serial_no = LaunchConfiguration("serial_no")
    namespace = LaunchConfiguration("namespace")
    rviz_enable = LaunchConfiguration("rviz_enable")

    realsense_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_share, "launch", "rs_launch.py")
        ),
        launch_arguments={
            # Keep the quotes in the string: otherwise ROS 2 parses this
            # numeric serial number as an integer instead of a string.
            "serial_no": serial_no,
            "camera_namespace": TextSubstitution(text="/"),
            "camera_name": TextSubstitution(text="camera"),
            "enable_color": TextSubstitution(text="false"),
            "enable_depth": TextSubstitution(text="false"),
            "enable_infra1": TextSubstitution(text="true"),
            "enable_infra2": TextSubstitution(text="true"),
            "depth_module.infra_profile": TextSubstitution(text="848x480x30"),
            # Keep the IR cameras for VIO, but disable the depth IR emitter
            # (laser/speckle projector) to avoid projecting the pattern.
            "depth_module.emitter_enabled": TextSubstitution(text="0"),
            "enable_gyro": TextSubstitution(text="true"),
            "enable_accel": TextSubstitution(text="true"),
            "gyro_fps": TextSubstitution(text="200"),
            "accel_fps": TextSubstitution(text="200"),
            "unite_imu_method": TextSubstitution(text="2"),
        }.items(),
    )

    openvins = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ov_msckf_share, "launch", "subscribe.launch.py")
        ),
        launch_arguments={
            "config": TextSubstitution(text="realsense"),
            "namespace": namespace,
            "rviz_enable": rviz_enable,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "serial_no",
            default_value=TextSubstitution(text="'339322073681'"),
            description="D435i serial number, preserved as a quoted ROS 2 string.",
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="ov_msckf",
            description="OpenVINS namespace.",
        ),
        DeclareLaunchArgument(
            "rviz_enable",
            default_value="false",
            description="Start RViz with the OpenVINS display configuration.",
        ),
        realsense_driver,
        openvins,
    ])
