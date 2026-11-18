"""Wiring test for the ROS 2 node, using stub rclpy/rmf_lift_msgs modules.

This verifies the glue code (parameters, topic wiring, message conversion,
timer callbacks) without a ROS 2 installation. It does not verify DDS
behavior; run the node against a live stack for that.
"""

import logging
import os
import sys
import types

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# ---------------------------------------------------------------------------
# Stub modules. They must be installed before importing the node module.
# ---------------------------------------------------------------------------


class _FakeTime:
    def __init__(self, nanoseconds):
        self.nanoseconds = nanoseconds

    def to_msg(self):
        return {"sec": self.nanoseconds // 1_000_000_000,
                "nanosec": self.nanoseconds % 1_000_000_000}


class _FakeClock:
    def __init__(self):
        self._now_ns = 0

    def set_seconds(self, seconds):
        self._now_ns = int(seconds * 1e9)

    def now(self):
        return _FakeTime(self._now_ns)


class _Parameter:
    def __init__(self, value):
        self.value = value


class _FakePublisher:
    def __init__(self):
        self.published = []

    def publish(self, msg):
        self.published.append(msg)


class _FakeNode:
    clock = _FakeClock()

    def __init__(self, name):
        self._name = name
        self._params = {}
        self._publishers = []
        self._subscriptions = {}
        self._timers = []

    def declare_parameter(self, name, default):
        self._params[name] = _Parameter(default)
        return self._params[name]

    def get_parameter(self, name):
        return self._params[name]

    def create_publisher(self, _msg_type, _topic, qos_profile=None):
        pub = _FakePublisher()
        self._publishers.append(pub)
        return pub

    def create_subscription(self, _msg_type, topic, callback, qos_profile=None):
        self._subscriptions[topic] = callback
        return callback

    def create_timer(self, period, callback):
        self._timers.append((period, callback))
        return callback

    def get_logger(self):
        return logging.getLogger("stub")

    def get_clock(self):
        return self.clock


def _install_stubs():
    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda *a, **k: None
    rclpy.spin = lambda *a, **k: None
    rclpy.shutdown = lambda *a, **k: None
    node_mod = types.ModuleType("rclpy.node")
    node_mod.Node = _FakeNode
    qos_mod = types.ModuleType("rclpy.qos")
    qos_mod.qos_profile_system_default = object()
    rclpy.node = node_mod
    rclpy.qos = qos_mod

    lift_msgs = types.ModuleType("rmf_lift_msgs")
    msg_mod = types.ModuleType("rmf_lift_msgs.msg")

    class LiftState:
        DOOR_CLOSED = 0
        DOOR_MOVING = 1
        DOOR_OPEN = 2
        MOTION_STOPPED = 0
        MOTION_UP = 1
        MOTION_DOWN = 2
        MOTION_UNKNOWN = 3
        MODE_UNKNOWN = 0
        MODE_HUMAN = 1
        MODE_AGV = 2
        MODE_FIRE = 3
        MODE_OFFLINE = 4
        MODE_EMERGENCY = 5

        def __init__(self):
            self.lift_time = None
            self.lift_name = ""
            self.available_floors = []
            self.current_floor = ""
            self.destination_floor = ""
            self.door_state = 0
            self.motion_state = 0
            self.available_modes = []
            self.current_mode = 0
            self.session_id = ""

    class LiftRequest:
        REQUEST_END_SESSION = 0
        REQUEST_AGV_MODE = 1
        REQUEST_HUMAN_MODE = 2
        DOOR_CLOSED = 0
        DOOR_OPEN = 2

        def __init__(self):
            self.lift_name = ""
            self.request_time = None
            self.session_id = ""
            self.request_type = 1
            self.destination_floor = ""
            self.door_state = 2

    msg_mod.LiftState = LiftState
    msg_mod.LiftRequest = LiftRequest
    lift_msgs.msg = msg_mod

    sys.modules["rclpy"] = rclpy
    sys.modules["rclpy.node"] = node_mod
    sys.modules["rclpy.qos"] = qos_mod
    sys.modules["rmf_lift_msgs"] = lift_msgs
    sys.modules["rmf_lift_msgs.msg"] = msg_mod
    return LiftState, LiftRequest


LiftState, LiftRequest = _install_stubs()

from lift_sim_skeleton.lift_node import LiftNode, data_to_ros, ros_to_data  # noqa: E402


def _drive(node, start, end, step=0.1):
    update = node._timers[0][1]
    publish = node._timers[1][1]
    t = start
    while t < end:
        t = min(end, t + step)
        _FakeNode.clock.set_seconds(t)
        update()
        publish()


def test_node_drives_full_session():
    _FakeNode.clock.set_seconds(0.0)
    node = LiftNode()
    req = LiftRequest()
    req.lift_name = "lift1"
    req.session_id = "fleet1"
    req.request_type = LiftRequest.REQUEST_AGV_MODE
    req.destination_floor = "L3"
    req.door_state = LiftRequest.DOOR_OPEN
    node._subscriptions["lift_requests"](req)
    _drive(node, 0.0, 8.0)

    published = node._publishers[0].published
    assert published, "node published nothing"
    assert published[0].session_id == "", "first states must gate the session"
    assert any(m.session_id == "fleet1" for m in published), "session never granted"
    last = published[-1]
    assert last.current_floor == "L3"
    assert last.door_state == LiftState.DOOR_OPEN
    assert last.current_mode == LiftState.MODE_AGV
    assert last.lift_time is not None


def test_node_reports_offline_fault():
    _FakeNode.clock.set_seconds(0.0)
    node = LiftNode()
    node._sim.set_offline(True)
    _drive(node, 0.0, 1.0)
    published = node._publishers[0].published
    assert published, "node published nothing"
    assert all(m.current_mode == LiftState.MODE_OFFLINE for m in published)
    assert all(m.session_id == "" for m in published)


def test_message_conversion_round_trip():
    req = LiftRequest()
    req.lift_name = "lift1"
    req.session_id = "s1"
    req.request_type = LiftRequest.REQUEST_HUMAN_MODE
    req.destination_floor = "L2"
    req.door_state = LiftRequest.DOOR_CLOSED
    data = ros_to_data(req)
    assert data.lift_name == "lift1"
    assert data.session_id == "s1"
    assert data.request_type == 2
    assert data.destination_floor == "L2"
    assert data.door_state == 0

    from lift_sim_skeleton.protocol import LiftStateData

    state = LiftStateData(
        lift_name="lift1",
        available_floors=["L1", "L2"],
        current_floor="L2",
        destination_floor="L2",
        door_state=2,
        motion_state=0,
        available_modes=[1, 2],
        current_mode=2,
        session_id="s1",
    )
    msg = data_to_ros(state, {"sec": 1, "nanosec": 0})
    assert msg.lift_name == "lift1"
    assert msg.available_floors == ["L1", "L2"]
    assert msg.session_id == "s1"
    assert msg.lift_time == {"sec": 1, "nanosec": 0}
