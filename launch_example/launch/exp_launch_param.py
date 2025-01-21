import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    animal = LaunchConfiguration("animal")
    animal_cmd = DeclareLaunchArgument(
        "animal",
        default_value="tiger",
        description="name an animal",
    )

    publish_test = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("publish_test"), "launch"),
                "/publish_test_launch.py",
            ]
        ),
        launch_arguments={"animal": animal}.items(),
    )

    return LaunchDescription([animal_cmd, publish_test])

# ros2 launch launch_example exp_launch_param.py animal:=lion