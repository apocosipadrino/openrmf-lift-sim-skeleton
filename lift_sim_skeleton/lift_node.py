"""ROS 2 node: the thin glue between RMF topics and the lift simulator.

All lift behavior lives in :mod:`lift_sim_skeleton.lift_sim`. This node
only converts messages, owns the timers, and wires parameters to the
simulator, which is what makes the simulator testable without ROS.

Topics (defaults, overridable by parameter):
  * subscribes ``lift_requests`` (rmf_lift_msgs/msg/LiftRequest)
  * publishes ``lift_states`` (rmf_lift_msgs/msg/LiftState)

Fault parameters (``start_offline``, ``door_blocked``) exist so adapter
robustness can be exercised from the launch file; they can also be toggled
in tests through the simulator directly.
"""

from __future__ import annotations

import logging
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from rmf_lift_msgs.msg import LiftRequest, LiftState

from .lift_sim import LiftSimulator
from .protocol import LiftRequestData, LiftStateData


def ros_to_data(msg: LiftRequest) -> LiftRequestData:
    return LiftRequestData(
        lift_name=msg.lift_name,
        session_id=msg.session_id,
        request_type=msg.request_type,
        destination_floor=msg.destination_floor,
        door_state=msg.door_state,
    )


def data_to_ros(state: LiftStateData, stamp) -> LiftState:
    msg = LiftState()
    msg.lift_time = stamp
    msg.lift_name = state.lift_name
    msg.available_floors = state.available_floors
    msg.current_floor = state.current_floor
    msg.destination_floor = state.destination_floor
    msg.door_state = state.door_state
    msg.motion_state = state.motion_state
    msg.available_modes = state.available_modes
    msg.current_mode = state.current_mode
    msg.session_id = state.session_id
    return msg


class LiftNode(Node):
    """A simulated lift, speaking LiftRequest/LiftState on ROS topics."""

    def __init__(self) -> None:
        super().__init__("lift_node")
        self.declare_parameter("lift_name", "lift1")
        self.declare_parameter("floors", ["L1", "L2", "L3"])
        self.declare_parameter("start_floor", "L1")
        self.declare_parameter("seconds_per_floor", 2.0)
        self.declare_parameter("door_seconds", 1.0)
        self.declare_parameter("mode_switch_seconds", 1.0)
        self.declare_parameter("update_period", 0.1)
        self.declare_parameter("publish_period", 0.5)
        self.declare_parameter("request_topic", "lift_requests")
        self.declare_parameter("state_topic", "lift_states")
        self.declare_parameter("start_offline", False)
        self.declare_parameter("door_blocked", False)

        self._sim = LiftSimulator(
            lift_name=self.get_parameter("lift_name").value,
            floors=self.get_parameter("floors").value,
            start_floor=self.get_parameter("start_floor").value,
            seconds_per_floor=self.get_parameter("seconds_per_floor").value,
            door_seconds=self.get_parameter("door_seconds").value,
            mode_switch_seconds=self.get_parameter("mode_switch_seconds").value,
            logger=logging.getLogger("lift_node"),
        )
        self._sim.set_offline(self.get_parameter("start_offline").value)
        self._sim.set_door_blocked(self.get_parameter("door_blocked").value)
        self._latest: LiftStateData | None = None

        self._state_pub = self.create_publisher(
            LiftState,
            self.get_parameter("state_topic").value,
            qos_profile_system_default,
        )
        self._request_sub = self.create_subscription(
            LiftRequest,
            self.get_parameter("request_topic").value,
            self._request_callback,
            qos_profile_system_default,
        )
        self._update_timer = self.create_timer(
            self.get_parameter("update_period").value, self._update_callback
        )
        self._publish_timer = self.create_timer(
            self.get_parameter("publish_period").value, self._publish_callback
        )
        self.get_logger().info(
            "Lift node running for lift '%s'",
            self.get_parameter("lift_name").value,
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _request_callback(self, msg: LiftRequest) -> None:
        self._sim.handle_request(ros_to_data(msg), self._now())

    def _update_callback(self) -> None:
        self._latest = self._sim.tick(self._now())

    def _publish_callback(self) -> None:
        if self._latest is None:
            return
        self._state_pub.publish(
            data_to_ros(self._latest, self.get_clock().now().to_msg())
        )


def main(argv: list[str] | None = None) -> None:
    rclpy.init(args=argv or sys.argv)
    node = LiftNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
