import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


DEFAULT_FCU_URL = (
    "serial:///dev/serial/by-id/"
    "usb-3D_Robotics_PX4_FMU_v5.x_0-if00:57600"
)


def generate_launch_description():
    adapter_share = get_package_share_directory("estimator_adapter")
    mavros_share = get_package_share_directory("mavros")
    workspace_root = os.path.abspath(
        os.path.join(adapter_share, "..", "..", "..", "..")
    )
    default_geographiclib_data = os.environ.get(
        "GEOGRAPHICLIB_DATA", os.path.join(workspace_root, ".geographiclib")
    )

    output_enabled = LaunchConfiguration("output_enabled")
    replay_mode = LaunchConfiguration("replay_mode")
    start_mavros = LaunchConfiguration("start_mavros")
    fcu_url = LaunchConfiguration("fcu_url")
    geographiclib_data = LaunchConfiguration("geographiclib_data")

    mavros = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(mavros_share, "launch", "px4.launch")
        ),
        condition=IfCondition(start_mavros),
        launch_arguments={"fcu_url": fcu_url}.items(),
    )

    adapter = Node(
        package="estimator_adapter",
        executable="estimator_adapter_node",
        name="estimator_adapter",
        output="screen",
        parameters=[
            os.path.join(adapter_share, "config", "estimator_adapter.yaml"),
            os.path.join(adapter_share, "config", "extrinsics.yaml"),
            {
                "output_enabled": output_enabled,
                "replay_mode": replay_mode,
            },
        ],
    )

    return LaunchDescription([
        # 控制是否将转换后的里程计发布给 MAVROS/PX4；设为 false 时仅订阅输入。
        DeclareLaunchArgument(
            "output_enabled",
            default_value="true",
            description="Publish converted odometry to MAVROS/PX4.",
        ),
        # 控制时间戳模式；false 使用实时传感器时间，true 为 rosbag 回放重新打时间戳。
        DeclareLaunchArgument(
            "replay_mode",
            default_value="false",
            description="Restamp replayed rosbag data with the current ROS time.",
        ),
        # 控制是否随 Adapter 一起启动 MAVROS。
        DeclareLaunchArgument(
            "start_mavros",
            default_value="true",
            description="Start MAVROS together with the adapter.",
        ),
        # 设置 MAVROS 连接 PX4 飞控所使用的串口地址和波特率。
        DeclareLaunchArgument(
            "fcu_url",
            default_value=DEFAULT_FCU_URL,
            description="MAVROS flight-controller connection URL.",
        ),
        # 设置 MAVROS 查找 GeographicLib 地理数据集的目录。
        DeclareLaunchArgument(
            "geographiclib_data",
            default_value=default_geographiclib_data,
            description="Directory containing MAVROS GeographicLib datasets.",
        ),
        # 将 GeographicLib 数据目录写入本次 launch 进程的环境变量。
        SetEnvironmentVariable("GEOGRAPHICLIB_DATA", geographiclib_data),
        # 按 start_mavros 开关决定是否启动 MAVROS PX4 节点。
        mavros,
        # 启动 OpenVINS 到 MAVROS 的里程计转换 Adapter 节点。
        adapter,
    ])
