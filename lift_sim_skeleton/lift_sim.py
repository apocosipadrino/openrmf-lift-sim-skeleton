"""Lift simulator: the building side of the lift protocol.

:class:`LiftSimulator` behaves like a lift as seen on the ``lift_requests``
/ ``lift_states`` topics: it accepts sessions, switches modes, travels
between floors, and operates its doors, all driven by a virtual clock so
tests can drive it deterministically with no sleeps and no ROS.

It enforces the protocol from the lift's side:

* **Session lock.** Once a session is accepted, requests from any other
  ``session_id`` are ignored until the owner sends ``REQUEST_END_SESSION``.
* **Session gating.** The ``session_id`` is only reported once the mode
  switch into the requested mode has actually completed. Reporting it
  early would let a robot enter a lift that is still in human mode.
* **Door semantics.** In AGV mode the doors are held open whenever the
  cabin is stopped. In human mode the doors follow the last explicit door
  command. The cabin never travels with the doors open.
* **Faults.** :meth:`set_offline` and :meth:`set_door_blocked` simulate a
  dead lift and a stuck door, so adapter robustness can be exercised
  without a real failure.

Time only advances when :meth:`tick` is called, which makes the simulator
trivially testable: the caller controls the clock.
"""

from __future__ import annotations

import logging

from .protocol import (
    DOOR_CLOSED,
    DOOR_MOVING,
    DOOR_OPEN,
    MODE_AGV,
    MODE_HUMAN,
    MODE_OFFLINE,
    MOTION_DOWN,
    MOTION_STOPPED,
    MOTION_UNKNOWN,
    MOTION_UP,
    REQUEST_AGV_MODE,
    REQUEST_END_SESSION,
    REQUEST_HUMAN_MODE,
    LiftRequestData,
    LiftStateData,
)

_REQUESTED_MODE = {
    REQUEST_AGV_MODE: MODE_AGV,
    REQUEST_HUMAN_MODE: MODE_HUMAN,
}


class LiftSimulator:
    """A simulated lift: session rules + cabin/door/mode physics."""

    def __init__(
        self,
        lift_name: str,
        floors: list[str],
        start_floor: str | None = None,
        seconds_per_floor: float = 2.0,
        door_seconds: float = 1.0,
        mode_switch_seconds: float = 1.0,
        logger: logging.Logger | None = None,
    ) -> None:
        if not floors:
            raise ValueError("floors must not be empty")
        if start_floor is not None and start_floor not in floors:
            raise ValueError(f"start_floor {start_floor!r} not in floors")
        self._lift_name = lift_name
        self._floors = list(floors)
        self._floor_idx = self._floors.index(start_floor) if start_floor else 0
        self._position = float(self._floor_idx)
        self._target_idx = self._floor_idx
        self._seconds_per_floor = seconds_per_floor
        self._door_seconds = door_seconds
        self._mode_switch_seconds = mode_switch_seconds
        self._log = logger or logging.getLogger(__name__)
        self._door_pos = 0.0  # 0.0 closed, 1.0 open
        self._door_target = 0.0
        self._door_command = DOOR_CLOSED
        self._mode = MODE_HUMAN
        self._pending_mode: int | None = None
        self._mode_ready_at: float | None = None
        self._motion = MOTION_STOPPED
        self._session_id = ""
        self._session_mode: int | None = None
        self._last_now: float | None = None
        self._offline = False
        self._door_blocked = False

    @property
    def session_id(self) -> str:
        """The session_id that currently owns the lift, or '' if free."""
        return self._session_id

    def set_offline(self, offline: bool) -> None:
        """Simulate a dead lift: it reports OFFLINE and accepts nothing."""
        self._offline = offline

    def set_door_blocked(self, blocked: bool) -> None:
        """Simulate a stuck door: the door position freezes where it is."""
        self._door_blocked = blocked

    def handle_request(self, req: LiftRequestData, now: float) -> None:
        """Apply one LiftRequest. Bad requests are ignored, never crash."""
        if req.lift_name != self._lift_name:
            self._log.debug("Ignoring request for lift %r", req.lift_name)
            return
        if self._offline:
            self._log.info("Lift %s offline; ignoring request", self._lift_name)
            return

        if req.request_type == REQUEST_END_SESSION:
            self._end_session(req, now)
            return

        target_mode = _REQUESTED_MODE.get(req.request_type)
        if target_mode is None:
            self._log.warning("Unknown request_type %r", req.request_type)
            return

        if self._session_id and req.session_id != self._session_id:
            self._log.info(
                "Lift %s locked by session %r; ignoring request from %r",
                self._lift_name,
                self._session_id,
                req.session_id,
            )
            return

        if req.destination_floor not in self._floors:
            self._log.info(
                "Floor %r not available on lift %s",
                req.destination_floor,
                self._lift_name,
            )
            return

        starting = not self._session_id
        self._session_id = req.session_id
        if starting or self._session_mode != target_mode:
            self._session_mode = target_mode
            if self._mode != target_mode:
                self._pending_mode = target_mode
                self._mode_ready_at = now + self._mode_switch_seconds
        self._target_idx = self._floors.index(req.destination_floor)
        self._door_command = req.door_state
        self._step(now)

    def tick(self, now: float) -> LiftStateData:
        """Advance the simulation to `now` and return the state to publish."""
        self._step(now)
        if self._offline:
            return LiftStateData(
                lift_name=self._lift_name,
                available_floors=list(self._floors),
                current_floor=self._floors[self._floor_idx],
                destination_floor=self._floors[self._target_idx],
                door_state=self._door_state(),
                motion_state=MOTION_UNKNOWN,
                available_modes=[MODE_HUMAN, MODE_AGV],
                current_mode=MODE_OFFLINE,
                session_id="",
            )
        granted = (
            self._session_id != ""
            and self._pending_mode is None
            and self._session_mode is not None
            and self._mode == self._session_mode
        )
        return LiftStateData(
            lift_name=self._lift_name,
            available_floors=list(self._floors),
            current_floor=self._floors[self._floor_idx],
            destination_floor=self._floors[self._target_idx],
            door_state=self._door_state(),
            motion_state=self._motion,
            available_modes=[MODE_HUMAN, MODE_AGV],
            current_mode=self._mode,
            session_id=self._session_id if granted else "",
        )

    def _end_session(self, req: LiftRequestData, now: float) -> None:
        if self._session_id and req.session_id != self._session_id:
            self._log.info(
                "Ignoring end-session from %r; lift %s is owned by %r",
                req.session_id,
                self._lift_name,
                self._session_id,
            )
            return
        if not self._session_id:
            return
        self._log.info(
            "Session %r ended on lift %s; returning to human mode",
            self._session_id,
            self._lift_name,
        )
        self._session_id = ""
        self._session_mode = None
        self._pending_mode = None
        self._mode_ready_at = None
        self._door_command = DOOR_CLOSED
        # The cabin finishes its current trip, then idles in human mode.
        if self._mode != MODE_HUMAN:
            self._pending_mode = MODE_HUMAN
            self._mode_ready_at = now + self._mode_switch_seconds
        self._step(now)

    def _door_state(self) -> int:
        if self._door_pos >= 1.0:
            return DOOR_OPEN
        if self._door_pos <= 0.0:
            return DOOR_CLOSED
        return DOOR_MOVING

    def _step(self, now: float) -> None:
        if self._last_now is None:
            self._last_now = now
            return
        dt = max(0.0, now - self._last_now)
        self._last_now = now
        if dt == 0.0 or self._offline:
            return

        # Mode switch completes on its own schedule.
        if (
            self._pending_mode is not None
            and self._mode_ready_at is not None
            and now >= self._mode_ready_at
        ):
            self._mode = self._pending_mode
            self._pending_mode = None
            self._mode_ready_at = None

        # Doors move toward their target unless blocked. In AGV mode the
        # target is always open while stopped; in human mode it follows
        # the last explicit command. Never travel with the doors open.
        if self._mode == MODE_AGV and self._motion == MOTION_STOPPED:
            self._door_target = 1.0
        elif self._mode != MODE_AGV:
            self._door_target = 1.0 if self._door_command == DOOR_OPEN else 0.0
        if self._target_idx != self._floor_idx:
            self._door_target = 0.0
        if not self._door_blocked:
            step = dt / self._door_seconds if self._door_seconds > 0 else 1.0
            if self._door_pos < self._door_target:
                self._door_pos = min(1.0, self._door_pos + step)
            elif self._door_pos > self._door_target:
                self._door_pos = max(0.0, self._door_pos - step)

        # Travel only with the doors fully closed.
        if self._target_idx == self._floor_idx or self._door_pos > 0.0:
            self._motion = MOTION_STOPPED
            return
        direction = 1 if self._target_idx > self._position else -1
        self._motion = MOTION_UP if direction > 0 else MOTION_DOWN
        travel = dt / self._seconds_per_floor
        self._position += direction * travel
        if (direction > 0 and self._position >= self._target_idx) or (
            direction < 0 and self._position <= self._target_idx
        ):
            self._position = float(self._target_idx)
            self._floor_idx = self._target_idx
            self._motion = MOTION_STOPPED
