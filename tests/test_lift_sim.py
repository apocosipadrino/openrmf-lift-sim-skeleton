"""Tests for the lift simulator, driven with a virtual clock."""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from lift_sim_skeleton.lift_sim import LiftSimulator  # noqa: E402
from lift_sim_skeleton.protocol import (  # noqa: E402
    DOOR_CLOSED,
    DOOR_OPEN,
    MODE_AGV,
    MODE_HUMAN,
    MODE_OFFLINE,
    MOTION_STOPPED,
    MOTION_UNKNOWN,
    REQUEST_AGV_MODE,
    REQUEST_END_SESSION,
    REQUEST_HUMAN_MODE,
    LiftRequestData,
)


def make_sim():
    return LiftSimulator("lift1", ["L1", "L2", "L3"])


def advance(sim, start, end, step=0.1):
    state = None
    t = start
    while t < end:
        t = min(end, t + step)
        state = sim.tick(t)
    return state


def request(session, floor, request_type=REQUEST_AGV_MODE, door=DOOR_OPEN):
    return LiftRequestData(
        lift_name="lift1",
        session_id=session,
        request_type=request_type,
        destination_floor=floor,
        door_state=door,
    )


def test_full_agv_session():
    sim = make_sim()
    sim.handle_request(request("fleet1", "L3"), 0.0)

    # Mode switch is still pending: the session must NOT be reported yet.
    state = sim.tick(0.0)
    assert state.session_id == "", "session reported before AGV mode switch"
    assert state.current_mode == MODE_HUMAN

    state = advance(sim, 0.0, 1.5)
    assert state.session_id == "fleet1"
    assert state.current_mode == MODE_AGV

    state = advance(sim, 1.5, 8.0)
    assert state.current_floor == "L3"
    assert state.door_state == DOOR_OPEN
    assert state.motion_state == MOTION_STOPPED

    sim.handle_request(request("fleet1", "L3", REQUEST_END_SESSION), 8.0)
    state = sim.tick(8.0)
    assert state.session_id == ""
    state = advance(sim, 8.0, 10.0)
    assert state.current_mode == MODE_HUMAN
    assert sim.session_id == ""


def test_session_lock_rejects_other_requesters():
    sim = make_sim()
    sim.handle_request(request("fleetA", "L2"), 0.0)
    sim.handle_request(request("fleetB", "L3"), 0.0)
    state = advance(sim, 0.0, 6.0)
    assert state.destination_floor == "L2"
    assert state.current_floor == "L2"
    assert state.session_id == "fleetA"


def test_unknown_floor_rejected():
    sim = make_sim()
    sim.handle_request(request("fleet1", "L9"), 0.0)
    state = advance(sim, 0.0, 3.0)
    assert state.destination_floor == "L1"
    assert state.session_id == "", "rejected request must not open a session"


def test_end_session_from_non_owner_ignored():
    sim = make_sim()
    sim.handle_request(request("fleetA", "L2"), 0.0)
    advance(sim, 0.0, 2.0)
    sim.handle_request(request("fleetB", "L2", REQUEST_END_SESSION), 2.0)
    state = advance(sim, 2.0, 4.0)
    assert state.session_id == "fleetA"


def test_offline_withholds_session_and_recovers():
    sim = make_sim()
    sim.handle_request(request("fleet1", "L2"), 0.0)
    advance(sim, 0.0, 2.0)
    sim.set_offline(True)
    state = sim.tick(3.0)
    assert state.current_mode == MODE_OFFLINE
    assert state.motion_state == MOTION_UNKNOWN
    assert state.session_id == "", "offline lift must not claim a session"
    # Requests are ignored while offline.
    sim.handle_request(request("fleetB", "L3"), 3.0)
    sim.set_offline(False)
    state = advance(sim, 3.0, 6.0)
    assert state.session_id == "fleet1", "session survives the outage"
    assert state.current_floor == "L2"


def test_blocked_door_prevents_travel():
    sim = make_sim()
    sim.handle_request(request("fleet1", "L1"), 0.0)
    state = advance(sim, 0.0, 3.0)
    assert state.door_state == DOOR_OPEN
    sim.set_door_blocked(True)
    sim.handle_request(request("fleet1", "L3"), 3.0)
    state = advance(sim, 3.0, 9.0)
    assert state.current_floor == "L1", "lift moved with a stuck-open door"
    assert state.motion_state == MOTION_STOPPED
    sim.set_door_blocked(False)
    state = advance(sim, 9.0, 16.0)
    assert state.current_floor == "L3"


def test_human_mode_door_commands():
    sim = make_sim()
    sim.handle_request(
        request("fleet1", "L1", REQUEST_HUMAN_MODE, DOOR_OPEN), 0.0
    )
    state = advance(sim, 0.0, 2.0)
    assert state.current_mode == MODE_HUMAN
    assert state.door_state == DOOR_OPEN
    sim.handle_request(
        request("fleet1", "L1", REQUEST_HUMAN_MODE, DOOR_CLOSED), 2.0
    )
    state = advance(sim, 2.0, 4.0)
    assert state.door_state == DOOR_CLOSED
    assert state.session_id == "fleet1"
