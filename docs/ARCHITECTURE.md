# Architecture

## Components

```
  rmf_lift_msgs topics                this repo
  ─────────────────────   ───────────────────────────────────

  lift_requests  ────────►  lift_node  ──►  lift simulator
  (LiftRequest)             (rclpy glue)     (session rules + physics)
  lift_states  ◄────────  lift_node  ◄──  lift simulator
  (LiftState)

  lift_client_demo ──► lift_requests   (demo aid only)
```

| Component | Responsibility | Depends on ROS? |
|---|---|---|
| `protocol.py` | Message shapes + constants mirroring `rmf_lift_msgs` | No |
| `lift_sim.py` | Session rules, cabin/door/mode physics, fault injection | No |
| `lift_node.py` | Topics, timers, parameters, ROS/dataclass conversion | Yes (`rclpy`) |
| `client_demo.py` | Publishes one AGV session, then ends it | Yes (`rclpy`) |

The split is deliberate: everything about lift *behavior* lives in code
that runs under plain `python3`, and the ROS node is too thin to hide a
behavior bug. The protocol shapes are a self-contained copy of the ones in
the sibling adapter repository, so each skeleton stands alone.

## Adapter vs node: where this repo fits

"lift adapter" names a role, not a process: adapting one specific lift to
RMF's lift protocol. In a split deployment that role spans two processes:

```
Fleet adapters ──LiftRequest──► Lift Adapter ──LiftRequest──► Lift Node ──API──► Lift Controller
                                (RMF-facing)   ◄──LiftState──  (hardware-facing)
```

- The **Lift Adapter** faces RMF: it receives requests, owns sessions,
  and publishes states. (That half is the
  [openrmf-lift-adapter-skeleton](https://github.com/apocosipadrino/openrmf-lift-adapter-skeleton)
  repository.)
- The **Lift Node** faces the hardware: it owns the vendor API and the
  physical controller.
- Together, the pair *is* the lift adapter for that elevator. Calling
  either half "the adapter" on its own invites exactly the confusion this
  split creates: both halves participate in adapting the lift, but neither
  is the whole adapter.

This repository simulates the whole right-hand side — Lift Node plus Lift
Controller — as one node, so the RMF-facing half can be tested without
hardware. In the classic single-process pattern (the `roscon_workshop`
reference), one node speaks RMF messages on one side and the vendor API on
the other, with a manager behind a REST API; the diagram above is what
that looks like when the two halves run as separate processes and happen
to exchange `rmf_lift_msgs` between them.

Vocabulary note: the `LiftRequest` messages are published by the fleet
adapter, not by RMF Core itself. RMF Core (task dispatch, traffic
scheduling) decides *that* a robot needs the lift; the fleet adapter is
the component that speaks the lift protocol.

## Wiring the sim behind an adapter

To chain the sim behind a lift adapter the way the diagram shows:

1. Run the sim on namespaced topics (`sim/lift_requests`,
   `sim/lift_states`) via its `request_topic` / `state_topic` parameters,
   so its messages never collide with the real lift topics.
2. Give the adapter a backend that bridges to those topics: implement the
   adapter skeleton's `LiftBackend` interface (`get_state`, `command`) as
   a small ROS client subscribed to `sim/lift_states` and publishing to
   `sim/lift_requests`.

The bridge backend is deliberately not included here: it belongs to the
adapter side, and the topic parameters on both sides already allow the
wiring. Until it exists, drive the sim directly with the demo requester
or `ros2 topic pub`, as in the README.

## Session lifecycle, from the lift's side

```
Requester (fleet / adapter)           Lift simulator
      │                                      │
      │  AGV, session S, floor F             │
      │ ────────────────────────────────────►│  lock S, start mode switch
      │                                      │
      │  LiftState, session ""               │  switch in progress:
      │ ◄────────────────────────────────────│  session withheld
      │                                      │
      │  LiftState, session S                │  MODE_AGV reached:
      │ ◄────────────────────────────────────│  session granted
      │                                      │
      │  ... doors close, cabin travels, doors open on arrival ...
      │                                      │
      │  END_SESSION, session S              │
      │ ────────────────────────────────────►│  release S, back to HUMAN
      │  LiftState, session ""               │
      │ ◄────────────────────────────────────│
```

## Protocol rules and rationale

1. **Session lock.** The lift obeys exactly one session at a time. While
   locked, requests from any other `session_id` are ignored — including
   an end-session request. Only the owner releases the lift.
2. **Session gating.** The fleet enters when it sees its own `session_id`
   together with the right floor, open doors, and AGV mode. The simulator
   reports the `session_id` only once the mode switch has actually
   completed. Reporting it early would let a robot enter a lift that is
   still in human mode. The same rule is enforced on the adapter side in
   the companion repository; both sides agreeing is what makes the
   protocol safe.
3. **Floor validation.** `destination_floor` "must be one of the values
   returned in a LiftState" (`LiftRequest.msg`). Unknown floors are
   rejected, and a rejected request does not open a session.
4. **Door semantics.** In AGV mode "the doors are always open when the lift
   is stopped" (`LiftRequest.msg`); explicit door requests are only
   meaningful in human mode. The cabin never travels with the doors open:
   a travel request first closes the doors, then moves, then reopens on
   arrival in AGV mode.
5. **End of session.** Releasing the session returns the lift to human
   mode. The cabin finishes its current trip first; ending a session
   mid-shaft does not reverse the cabin.

## Faults

| Fault | Reported state | Request handling | Recovery |
|---|---|---|---|
| Offline | `MODE_OFFLINE`, `MOTION_UNKNOWN`, session withheld | All requests ignored | Session survives; reporting resumes |
| Door blocked | Door position frozen (often `DOOR_MOVING`) | Requests accepted, travel waits for closed doors | Door resumes; queued trip proceeds |

Faults exist so the *other* side of the conversation — an adapter, a fleet
adapter, a test harness — can be exercised against failure without a real
one. An offline lift withholding its session (rather than dropping it) is
the behavior an adapter must handle: the session is still owned, the lift
is just not talking.

## Design decisions

- **Virtual clock.** `tick(now)` / `handle_request(req, now)` take time as
  a float. Tests drive the whole lifecycle deterministically with no
  sleeps and no ROS clock. The node supplies ROS time at the boundary.
- **Honest sequencing.** Doors close before travel, travel takes time,
  doors open after arrival, mode switches take time. A simulator that
  teleports would let protocol bugs hide on both sides; this one cannot.
- **One lift per node.** The simulator is single-lift on purpose. Scaling
  to many lifts is a launch-file problem (one node per lift), not a reason
  to complicate the session logic.
- **Polling publish.** The node advances the simulator on a fast timer and
  publishes on a slower one. Publish rate stays independent of simulation
  rate, and the published state is always a complete snapshot.

## What to add for a fuller simulation

Multiple cabins per shaft, per-floor door sides, passenger call buttons,
building-map (`traffic editor`) integration, and Gazebo parity for the
`rmf_building_sim` lift plugin. None of those change the session rules;
they extend the physics and the configuration surface.
