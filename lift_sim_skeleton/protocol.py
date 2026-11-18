"""Message shapes mirroring rmf_lift_msgs.

These dataclasses intentionally mirror the public ROS 2 message definitions
in rmf_lift_msgs (open-rmf/rmf_internal_msgs) so the core logic can be
developed and tested without a ROS 2 installation. The ROS 2 node converts
between these dataclasses and the real messages at the topic boundary.

This module is intentionally a self-contained copy of the protocol shapes:
each skeleton repository stands alone, with no dependency on the sibling
adapter repository.

Source of truth for field names and constant values:
  rmf_lift_msgs/msg/LiftState.msg
  rmf_lift_msgs/msg/LiftRequest.msg
"""

from __future__ import annotations

from dataclasses import dataclass, field


# LiftState.door_state / LiftRequest.door_state
DOOR_CLOSED = 0
DOOR_MOVING = 1  # LiftState only; never sent in a LiftRequest
DOOR_OPEN = 2

# LiftState.motion_state
MOTION_STOPPED = 0
MOTION_UP = 1
MOTION_DOWN = 2
MOTION_UNKNOWN = 3

# LiftState.current_mode / available_modes
MODE_UNKNOWN = 0
MODE_HUMAN = 1
MODE_AGV = 2
MODE_FIRE = 3
MODE_OFFLINE = 4
MODE_EMERGENCY = 5

# LiftRequest.request_type
REQUEST_END_SESSION = 0
REQUEST_AGV_MODE = 1
REQUEST_HUMAN_MODE = 2


@dataclass
class LiftRequestData:
    """ROS-free mirror of rmf_lift_msgs/msg/LiftRequest."""

    lift_name: str = ""
    session_id: str = ""
    request_type: int = REQUEST_AGV_MODE
    destination_floor: str = ""
    door_state: int = DOOR_OPEN


@dataclass
class LiftStateData:
    """ROS-free mirror of rmf_lift_msgs/msg/LiftState."""

    lift_name: str = ""
    available_floors: list[str] = field(default_factory=list)
    current_floor: str = ""
    destination_floor: str = ""
    door_state: int = DOOR_CLOSED
    motion_state: int = MOTION_STOPPED
    available_modes: list[int] = field(default_factory=list)
    current_mode: int = MODE_UNKNOWN
    session_id: str = ""
