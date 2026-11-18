"""Launch the simulated lift node."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="lift_sim_skeleton",
                executable="lift_node",
                name="lift_node",
                parameters=[
                    {
                        "lift_name": "lift1",
                        "floors": ["L1", "L2", "L3"],
                        "start_floor": "L1",
                        "seconds_per_floor": 2.0,
                        "door_seconds": 1.0,
                        "mode_switch_seconds": 1.0,
                    }
                ],
            )
        ]
    )
