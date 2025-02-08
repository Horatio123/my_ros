import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    animal_cmd = DeclareLaunchArgument(
        "animal",
        default_value="Panthera",
        description="name an animal",
    )
    
    animal = LaunchConfiguration("animal")

    return launch.LaunchDescription(
        [
            animal_cmd,
            launch_ros.actions.Node(
                package="publish_test",
                executable="talker",
                name="talker",
                arguments=["--ros-args", "--log-level", "info"],
                parameters=[{"animal": animal}],
                # parameters=[{"animal": LaunchConfiguration("animal")}],
            ),
            # launch_ros.actions.Node(
            #     package="publish_test",
            #     executable="listener",
            #     name="listener",
            # ),
        ]
    )
# ros2 launch publish_test publish_test_launch.py animal:=lion