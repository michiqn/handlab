"""MockDriver — simulated servos, no physics. Default for zero-setup UI work."""

from __future__ import annotations

from ..models import ServoMap
from .base import HandDriver, ServoState


class MockDriver(HandDriver):
    driver_type = "mock"

    def __init__(self, servo_map: ServoMap):
        self._map = servo_map
        self._pos = {s.id: 2048 for s in servo_map.servos}
        self._goal = dict(self._pos)
        self._torque = {s.id: False for s in servo_map.servos}
        self._connected = False

    def connect(self, port: str, baud: int) -> None:
        self._connected = True

    def disconnect(self) -> None:
        self._connected = False

    def ping_all(self) -> list[int]:
        return sorted(self._pos) if self._connected else []

    def read_states(self) -> dict[int, ServoState]:
        # ease position toward goal so the UI shows motion (limp servos ease to the
        # elastic rest at 2048, mirroring the real hardware's rubber-band return)
        for sid, goal in self._goal.items():
            self._pos[sid] += int((goal - self._pos[sid]) * 0.3)
        return {
            sid: ServoState(id=sid, present_position=self._pos[sid],
                            torque_enabled=self._torque[sid],
                            moving=abs(self._goal[sid] - self._pos[sid]) > 6)
            for sid in self._pos
        }

    def set_torque(self, servo_id: int, enable: bool) -> None:
        was = self._torque.get(servo_id, False)
        self._torque[servo_id] = enable
        if was and not enable:
            self._goal[servo_id] = 2048            # limp -> elastic rest
        elif enable and not was:
            self._goal[servo_id] = self._pos[servo_id]   # hold current — never a jump

    def set_goal_position(self, servo_id: int, raw_ticks: int) -> None:
        self._goal[servo_id] = int(raw_ticks)
        if not self._torque[servo_id]:
            self._pos[servo_id] = int(raw_ticks)   # backdrive

    def read_position(self, servo_id: int) -> int:
        return self._pos[servo_id]
