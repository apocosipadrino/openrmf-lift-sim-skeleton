"""Demo requester: drives the lift node through one AGV session.

Publishes a LiftRequest for AGV mode, waits for the lift to arrive and open
its doors, then ends the session. Demo aid only -- not part of the lift.

Run it against a running lift node:

    ros2 run lift_sim_skeleton lift_client_demo
"""

from __future__ import annotations

import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from rmf_lift_msgs.msg import LiftRequest


class LiftClientDemo(Node):
    def __init__(self) -> None:
        super().__init__("lift_client_demo")
        self.declare_parameter("lift_name", "lift1")
        self.declare_parameter("destination_floor", "L3")
        self.declare_parameter("session_id", "demo")
        self.declare_parameter("request_topic", "lift_requests")
        self._pub = self.create_publisher(
            LiftRequest,
            self.get_parameter("request_topic").value,
            qos_profile_system_default,
        )
        self._step = 0
        self._timer = self.create_timer(1.0, self._timer_callback)
        self.get_logger().info("Lift client demo started")

    def _publish(self, request_type: int) -> None:
        msg = LiftRequest()
        msg.lift_name = self.get_parameter("lift_name").value
        msg.session_id = self.get_parameter("session_id").value
        msg.request_time = self.get_clock().now().to_msg()
        msg.request_type = request_type
        msg.destination_floor = self.get_parameter("destination_floor").value
        msg.door_state = LiftRequest.DOOR_OPEN
        self._pub.publish(msg)

    def _timer_callback(self) -> None:
        self._step += 1
        if self._step == 1:
            self.get_logger().info("Requesting AGV mode")
            self._publish(LiftRequest.REQUEST_AGV_MODE)
        elif self._step == 12:
            self.get_logger().info("Ending session")
            self._publish(LiftRequest.REQUEST_END_SESSION)
            self._timer.cancel()


def main(argv: list[str] | None = None) -> None:
    rclpy.init(args=argv or sys.argv)
    node = LiftClientDemo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
