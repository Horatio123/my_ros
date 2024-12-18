import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="publish_test",
                executable="talker",
                name="talker",
            ),
            launch_ros.actions.Node(
                package="publish_test",
                executable="listener",
                name="listener",
            ),
        ]
    )
