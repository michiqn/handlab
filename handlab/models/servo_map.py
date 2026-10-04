"""ServoMap: the flat servo list the control stack consumes. DERIVED from a Build by compose.py."""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class Servo:
    id: int                        # dynamixel id
    finger: str                    # finger instance id, e.g. "index"
    dof: str                       # dof key, e.g. "curl"
    actuator: str                  # composed MuJoCo actuator name, e.g. "index_curl"
    joint: str = ""                # composed MuJoCo joint behind it (position feedback)


@dataclass
class ServoMap:
    servos: list[Servo] = field(default_factory=list)

    @property
    def servo_count(self) -> int:
        return len(self.servos)

    def by_id(self, sid: int) -> Servo | None:
        return next((s for s in self.servos if s.id == sid), None)

    def by_finger_dof(self, finger: str, dof: str) -> Servo | None:
        return next((s for s in self.servos if s.finger == finger and s.dof == dof), None)
