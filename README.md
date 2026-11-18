# OpenRMF Lift Sim Skeleton

A skeleton [OpenRMF](https://github.com/open-rmf) lift simulator: a ROS 2
node that *is* a lift. It speaks `rmf_lift_msgs` on the `lift_requests` /
`lift_states` topics, simulating a cabin, doors, and modes with honest
timing, so lift adapters and fleet logic can be exercised end to end
without Gazebo or a physical elevator.

This is a community skeleton, not part of OpenRMF. It was written from the
public `rmf_lift_msgs` definitions and the OpenRMF workshop examples; it
contains no vendor or employer code.

## What it is

- A **lift simulator** with deterministic, clock-driven physics: travel
  time per floor, door transit time, mode-switch delay. Doors close before
  travel, open after arrival, and are held open while stopped in AGV mode.
- **Session rules from the lift's side**: session lock, session gating
  (the `session_id` is only reported once the mode switch completes),
  floor validation, and human-mode door commands.
- **Fault injection**: an offline mode and a stuck door, so adapter
  robustness can be tested without a real failure.
- A thin **ROS 2 node** (`rclpy`) that only converts messages, owns the
  timers, and wires parameters to the simulator — plus a small **demo
  requester** that drives one AGV session.

## What it is not

- Not a Gazebo replacement: it does not simulate shaft geometry, door
  sensors, or passenger traffic, and it does not read building maps.
- Not yet run against a live ROS 2 / OpenRMF stack. The simulator is unit
  tested and the node wiring is tested against stubs (see Testing); live
  validation is still to do.

## Where this fits: adapter vs node

In OpenRMF, "lift adapter" names a role, not a process: adapting one
specific lift to RMF's lift protocol. In a split deployment that role
spans two processes:

```
Fleet adapters ──LiftRequest──► Lift Adapter ──LiftRequest──► Lift Node ──API──► Lift Controller
                                (RMF-facing)   ◄──LiftState──  (hardware-facing)
```

- The **Lift Adapter** is the half that faces RMF: it receives requests,
  owns sessions, and publishes states.
- The **Lift Node** is the half that faces the hardware: it owns the
  vendor API and the physical controller.
- Together, the pair *is* the lift adapter for that elevator. Neither half
  alone is the whole adapter.

This repository simulates the whole right-hand side — Lift Node plus Lift
Controller — as a single node, so an adapter can be tested without
hardware. In the classic single-process pattern (see the
`roscon_workshop` reference below), one node speaks RMF messages on one
side and the vendor API on the other; the split above is what that looks
like when the two halves run as separate processes.

One vocabulary note: the `LiftRequest` messages are published by the fleet
adapter, not by RMF Core itself.

## Wiring the sim behind an adapter

To chain this sim behind a lift adapter the way the diagram shows, run the
sim on namespaced topics so its messages do not collide with the real
lift topics, and give the adapter a backend that bridges to those topics:

```bash
# Terminal 1: the simulated lift, on sim/ topics.
ros2 run lift_sim_skeleton lift_node --ros-args \
  -p request_topic:=sim/lift_requests -p state_topic:=sim/lift_states

# Terminal 2: the adapter, on the real topics, with a bridging backend.
# (The bridge backend is not in this repo: implement the adapter skeleton's
# LiftBackend interface against the sim/ topics. The topic parameters on
# both sides already allow this wiring.)
```

See `docs/ARCHITECTURE.md` for the full picture.

## Quickstart

Requires ROS 2 Humble or newer with `rmf_lift_msgs` (from OpenRMF's
`rmf_internal_msgs` repository) installed.

```bash
# Inside a ROS 2 workspace:
cd ~/ros2_ws/src
git clone https://github.com/apocosipadrino/openrmf-lift-sim-skeleton.git
cd ~/ros2_ws
colcon build --packages-select lift_sim_skeleton
source install/setup.bash
ros2 launch lift_sim_skeleton lift_sim.launch.py
```

Drive it with the demo requester from another terminal:

```bash
ros2 run lift_sim_skeleton lift_client_demo
ros2 topic echo /lift_states
```

Or publish requests directly:

```bash
# Request AGV mode, session "demo", destination L3:
ros2 topic pub --once /lift_requests rmf_lift_msgs/msg/LiftRequest \
  "{lift_name: lift1, session_id: demo, request_type: 1, destination_floor: L3, door_state: 2}"

# End the session:
ros2 topic pub --once /lift_requests rmf_lift_msgs/msg/LiftRequest \
  "{lift_name: lift1, session_id: demo, request_type: 0, destination_floor: L3, door_state: 0}"
```

`request_type`: 0 = end session, 1 = AGV mode, 2 = human mode.
`door_state`: 0 = closed, 2 = open.

## Fault injection

```bash
# A dead lift: reports MODE_OFFLINE, accepts nothing, keeps its session.
ros2 run lift_sim_skeleton lift_node --ros-args -p start_offline:=true

# A stuck door: the cabin cannot travel until the door is freed.
ros2 run lift_sim_skeleton lift_node --ros-args -p door_blocked:=true
```

The fault parameters are read at startup. In tests, the same faults are
toggled directly on the simulator.

## Layout

```
lift_sim_skeleton/
  protocol.py      Message shapes mirroring rmf_lift_msgs (no ROS imports)
  lift_sim.py      LiftSimulator: session rules + cabin/door/mode physics
  lift_node.py     rclpy node: topics, timers, parameters, conversion
  client_demo.py   Demo requester: one AGV session, then end-session
launch/            Launch file (3 floors)
tests/             Unit tests + stub wiring test, runnable without ROS
docs/              Architecture notes and protocol rationale
```

## Testing

```bash
python3 tests/run_tests.py   # no ROS, no pytest needed
pytest tests/                # also works
```

The suite covers the full session lifecycle, the session lock, floor
validation, non-owner end-session, offline withhold-and-recover, the stuck
door, human-mode door commands, and the node wiring (against stub `rclpy` /
`rmf_lift_msgs` modules).

## Companion repository

[openrmf-lift-adapter-skeleton](https://github.com/apocosipadrino/openrmf-lift-adapter-skeleton)
is the other side of the same conversation: the adapter that sits between
RMF and a lift, with the session protocol in a testable core and a
swappable backend. Run the two together (see Wiring above) to exercise an
adapter end to end without Gazebo or a physical elevator.

## References

- `rmf_lift_msgs` definitions: `rmf_lift_msgs/msg/LiftState.msg` and
  `LiftRequest.msg` in
  [open-rmf/rmf_internal_msgs](https://github.com/open-rmf/rmf_internal_msgs).
- Adapter pattern reference:
  [open-rmf/roscon_workshop](https://github.com/open-rmf/roscon_workshop)
  (`roscon_lift_adapter`: adapter / API / manager split).
- Session-gating rationale: OpenRMF Discourse, "Lift session issues — robot
  enters lift while lift is still in human mode (#513)".
- Paper-level background: OpenRMF, "Robot Middleware Framework"
  ([open-rmf.org](https://www.open-rmf.org/)).

## License

MIT. See `LICENSE`.
