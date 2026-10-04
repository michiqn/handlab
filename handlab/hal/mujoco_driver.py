"""MuJoCoDriver — drives the composed MuJoCo twin through the HandDriver interface.

The twin is the "hardware" until the motors arrive (and the always-available test rig
after). The Bridge owns the Twin (it also renders it); this driver is handed that same
instance so physics, UI sliders and telemetry all agree — never a second sim.

Conventions (identical to the Dynamixel driver, so the control stack is driver-agnostic):
  - positions are raw ticks, 4096/rev, center 2048 = joint zero:
        ticks = 2048 + rad * TICKS_PER_RAD
  - torque OFF: goals are ignored (a limp servo doesn't chase), and the finger relaxes
    into its printed rest pose — the real CRAFT finger has ELASTIC return (rubber bands
    pull it straight when the tendon goes slack), so the sim drives ctrl to 0 rad (the
    export rest pose) to model that. Re-engaging torque resumes the last goal.

Maps each servo to its PREFIXED actuator/joint ("A_curl" / "A_proximal_v2_curl") from the
ServoMap, so every configured finger drives the sim (no "first finger wins").
"""

from __future__ import annotations

from ..models import ServoMap
from .base import CENTER, HandDriver, ServoState, rad_to_ticks, ticks_to_rad


class MuJoCoDriver(HandDriver):
    driver_type = "mujoco"

    def __init__(self, servo_map: ServoMap, twin):
        self._map = servo_map
        self._twin = twin                  # shared with the Bridge's renderer
        self._connected = False
        self._torque = {s.id: True for s in servo_map.servos}   # sim servos boot holding
        self._goal = {s.id: CENTER for s in servo_map.servos}
        self._last_pos = dict(self._goal)
        self._relax: dict[int, float] = {}  # limp servos easing toward rest (rad)

    # ---- lifecycle (the Bridge owns the twin's lifecycle; connect is a handshake) ----
    def connect(self, port: str, baud: int) -> None:
        self._connected = self._twin is not None

    def disconnect(self) -> None:
        self._connected = False

    def ping_all(self) -> list[int]:
        return sorted(s.id for s in self._map.servos) if self._connected else []

    # ---- telemetry ----
    def read_states(self) -> dict[int, ServoState]:
        # Limp servos ease toward the rest pose, stepped on the telemetry beat (10 Hz):
        # an instant ctrl=0 would yank the chain and whip the weakly-damped spread joint.
        for sid in list(self._relax):
            s = self._map.by_id(sid)
            nv = self._relax[sid] * 0.65          # ~1 s to rest at 10 Hz
            if abs(nv) < 0.01:
                nv = 0.0
                del self._relax[sid]
            else:
                self._relax[sid] = nv
            if s is not None and self._twin is not None:
                self._twin.set_ctrl(s.actuator, nv)

        out: dict[int, ServoState] = {}
        # ONE lock hold for all joints (was one per servo → fought the physics-step thread each read)
        qpos = self._twin.qpos_many([s.joint for s in self._map.servos]) if self._twin is not None else {}
        for s in self._map.servos:
            pos = rad_to_ticks(qpos[s.joint]) if s.joint in qpos else self._goal[s.id]
            moving = abs(pos - self._last_pos.get(s.id, pos)) > 2
            self._last_pos[s.id] = pos
            out[s.id] = ServoState(
                id=s.id, present_position=pos,
                torque_enabled=self._torque[s.id], moving=moving,
            )
        return out

    # ---- control ----
    def set_torque(self, servo_id: int, enable: bool) -> None:
        was = self._torque.get(servo_id, True)
        self._torque[servo_id] = enable
        s = self._map.by_id(servo_id)
        if s is None or self._twin is None:
            return
        if was and not enable:
            # going limp: hold at the current angle, then EASE to the rest pose
            # (read_states steps the easing — gentle like the rubber bands, no whip)
            try:
                start = self._twin.qpos(s.joint)
            except Exception:
                start = ticks_to_rad(self._goal[servo_id])
            self._twin.set_ctrl(s.actuator, start)
            self._relax[servo_id] = start
        elif enable and not was:
            # re-engaging HOLDS the current position (goal := present) — never a jump.
            # Real X-series firmware does exactly this on torque enable.
            self._relax.pop(servo_id, None)
            try:
                cur = self._twin.qpos(s.joint)
            except Exception:
                cur = ticks_to_rad(self._goal[servo_id])
            self._goal[servo_id] = rad_to_ticks(cur)
            self._twin.set_ctrl(s.actuator, cur)

    def set_goal_position(self, servo_id: int, raw_ticks: int) -> None:
        self._goal[servo_id] = int(raw_ticks)
        s = self._map.by_id(servo_id)
        if s is not None and self._twin is not None and self._torque.get(servo_id, True):
            self._twin.set_ctrl(s.actuator, ticks_to_rad(int(raw_ticks)))

    def read_position(self, servo_id: int) -> int:
        s = self._map.by_id(servo_id)
        if s is None or self._twin is None:
            return self._goal.get(servo_id, CENTER)
        try:
            return rad_to_ticks(self._twin.qpos(s.joint))
        except Exception:
            return self._goal.get(servo_id, CENTER)
