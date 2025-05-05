from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    laser_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("yahboomcar_nav"), "launch"),
                "/laser_bringup_launch.py",
            ]
        )
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
    )

    ctrl_node = Node(
        package="yahboomcar_ctrl",
        executable="yahboom_joy_X3",
    )

    # 添加 astra_camera 启动文件
    astra_camera_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("astra_camera"),
                "launch",
                "astro_pro_plus.launch.xml",
            )
        )
    )

    launch_description = LaunchDescription(
        [joy_node, ctrl_node, laser_bringup_launch, astra_camera_launch]
    )
    return launch_description
# ros2 launch data_collector collect_laser_camera.py