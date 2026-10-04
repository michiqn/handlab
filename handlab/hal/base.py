"""HandDriver — the interface every driver implements (mock / mujoco / dynamixel).

Shared position convention (all drivers): raw ticks, 4096 per revolution,
CENTER (2048) = joint zero = the stretched rest pose.
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass

TICKS_PER_RAD = 4096 / (2.0 * math.pi)
CENTER = 2048


def ticks_to_rad(ticks: int) -> float:
    return (ticks - CENTER) / TICKS_PER_RAD


def rad_to_ticks(rad: float) -> int:
    return round(CENTER + rad * TICKS_PER_RAD)


def ticks_to_deg(ticks: int, nd: int = 1) -> float:
    """Raw ticks → joint DEGREES, rounded to `nd` places (0° = CENTER = stretched rest)."""
    return round(math.degrees(ticks_to_rad(ticks)), nd)


@dataclass
class ServoState:
    id: int
    present_position: int          # raw ticks
    present_current: int = 0
    present_temperature: int = 30
    present_voltage: float = 5.0
    moving: bool = False
    torque_enabled: bool = False
    hw_error: int = 0


class HandDriver(ABC):
    driver_type: str = "base"

    @abstractmethod
    def connect(self, port: str, baud: int) -> None: ...
    @abstractmethod
    def disconnect(self) -> None: ...
    @abstractmethod
    def ping_all(self) -> list[int]: ...
    @abstractmethod
    def read_states(self) -> dict[int, ServoState]: ...
    @abstractmethod
    def set_torque(self, servo_id: int, enable: bool) -> None: ...
    @abstractmethod
    def set_goal_position(self, servo_id: int, raw_ticks: int) -> None: ...
    @abstractmethod
    def read_position(self, servo_id: int) -> int: ...

    def set_goal_positions(self, goals: dict[int, int]) -> None:
        """Batch goal write — default just loops `set_goal_position` (mock/MuJoCo inherit this, so
        sim teleop keeps driving the twin). The dynamixel driver overrides it with ONE GroupSyncWrite
        broadcast so a 30 Hz teleop frame costs one TX-only packet instead of 9 blocking round-trips."""
        for sid, ticks in goals.items():
            self.set_goal_position(sid, ticks)

    def goals(self) -> dict[int, int]:
        """Last per-servo goal position (logical ticks, 2048 = zero) each driver was TOLD to hit —
        the commanded 'action'. INTENDED, not written/present: the dynamixel driver caches this
        BEFORE its offline/torque-off early-return, so it reflects what the controller asked for, not
        what actually reached the servo register. Used by the demo recorder as the IL action label.
        Default reads the driver's `_goal` cache (mock/mujoco/dynamixel all populate it)."""
        return dict(getattr(self, "_goal", {}))
