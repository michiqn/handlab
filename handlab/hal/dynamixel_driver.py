"""DynamixelDriver — real XL330 servos over USB (protocol 2.0, X-series control table).

Driven by the generated ServoMap, so no hand layout is hardcoded. Select with
HANDLAB_DRIVER=dynamixel (port/baud/velocity from the build's `hardware:` block).
`dynamixel-sdk` is imported lazily — the app runs without it.

Position convention (shared, see hal/base.py): raw ticks, 4096/rev, CENTER (2048) =
joint zero = stretched rest. Per servo we apply CALIBRATION:
  - sign   (+1/-1): wiring direction, persistent.
  - offset (ticks): wire value at the rest pose, normally captured fresh on connect
    (delta-home — Extended Position wraps after a power cycle, so a stored absolute
    offset would point nowhere).
    logical -> wire : wire    = CENTER + offset + sign*(logical - CENTER)
    wire -> logical : logical = CENTER + sign*(wire - CENTER - offset)

Safety on real hardware:
  - Profile Velocity + Acceleration are written conservatively on connect (factory 0 =
    MAX speed!) and re-asserted on torque enable.
  - Torque enable writes Goal := Present FIRST, so re-engaging never jumps.
  - Current-based Position mode (5) + Goal Current: pressing an obstacle saturates at a
    bounded force (compliant) instead of stalling into an overload shutdown.
  - The protocol ALERT bit (latched hardware error) is NOT treated as an IO failure —
    reads/writes keep working; reg 70 is surfaced in telemetry and reboot() clears it.
  - Dropped servos are re-pinged periodically (bus hiccups heal without a reconnect);
    the torque flag is read back from the wire (self-shutdowns become visible).
"""

from __future__ import annotations

from ..models import ServoMap
from .base import CENTER, HandDriver, ServoState

# X-series (XC330/XL330) control table, protocol 2.0
_ADDR_OPERATING_MODE = 11      # 1 B · 3=position, 4=extended position, 5=current-based position
_ADDR_CURRENT_LIMIT = 38       # 2 B (EEPROM! write with torque OFF) · ~1 mA units
_ADDR_TORQUE_ENABLE = 64       # 1 B
_ADDR_HW_ERROR = 70            # 1 B
_ADDR_POS_D_GAIN = 80          # 2 B (RAM) · position D gain
_ADDR_POS_I_GAIN = 82          # 2 B (RAM) · position I gain
_ADDR_POS_P_GAIN = 84          # 2 B (RAM) · position P gain — the stiffness of position tracking
_ADDR_GOAL_CURRENT = 102       # 2 B (RAM) · force cap in current-based position mode
_ADDR_PROFILE_ACCEL = 108      # 4 B
_ADDR_PROFILE_VELOCITY = 112   # 4 B
_ADDR_GOAL_POSITION = 116      # 4 B
_ADDR_MOVING = 122             # 1 B
_ADDR_PRESENT_CURRENT = 126    # 2 B (signed, ~1 mA)
_ADDR_PRESENT_POSITION = 132   # 4 B (signed in extended mode)
_ADDR_PRESENT_VOLTAGE = 144    # 2 B (0.1 V)
_ADDR_PRESENT_TEMP = 146       # 1 B (°C)

# Absolute-home recovery is only unambiguous within ±180° (±2048 ticks) of home; at exactly ±2048
# the shaft delta flips sign. If a spring-less joint (thumb flex geometry [-5,190], tendon stretch)
# is left past this, a naive recovery homes it a full revolution wrong → commands the long way into
# the wall. So we REFUSE absolute-home when the delta is within this guard of the flip and fall back
# to delta-home (current pose = 0° — never a violent move) for that servo instead.
_HOME_FLIP_GUARD = 1900        # ticks (~167°): closer to the ±2048 flip than this → don't trust it

# one contiguous block for GroupSyncRead: moving .. temp (122..146 inclusive = 25 B)
_BLK_ADDR = _ADDR_MOVING
_BLK_LEN = _ADDR_PRESENT_TEMP - _ADDR_MOVING + 1

# Current-based Position Control: position servo with the applied current SATURATING at Goal
# Current — pressing an obstacle yields a bounded, compliant push instead of an overload trip.
# Position range behaves like extended position (multi-turn), so the tick/zero convention holds.
_CURRENT_POSITION_MODE = 5
_ALERT_BIT = 0x80              # protocol 2.0 status "alert" flag: a hardware error is LATCHED —
                               # it rides on every packet and is NOT itself a comm failure
_HOLD_AFTER_TICKS = 5          # "grasp firm, hold gentle": after this many not-moving telemetry
                               # ticks (~0.5 s at 10 Hz) a settled servo drops from goal_current
                               # (move force) to hold_current (gentle hold) — heat + supply budget


def _i16(u: int) -> int:
    return u - 0x10000 if u >= 0x8000 else u


def _i32(u: int) -> int:
    return u - 0x100000000 if u >= 0x80000000 else u


class DynamixelDriver(HandDriver):
    driver_type = "dynamixel"

    def __init__(self, servo_map: ServoMap, calibration: dict[int, dict] | None = None,
                 profile_velocity: int = 60, profile_acceleration: int = 20,
                 current_limit: int = 600, goal_current: int = 350,
                 hold_current: int | None = None,
                 pos_p: int = 900, pos_i: int = 0, pos_d: int = 0):
        self._map = servo_map
        self._port = None
        self._packet = None
        self._sync = None
        self._sync_write = None                      # GroupSyncWrite(GOAL_POSITION) — teleop batch path
        self._last_wire: dict[int, int] = {}         # last goal WIRE value per servo (batch dedup);
        #                                              invalidated wherever the register can change
        #                                              behind our back (_configure/reboot/torque-on)
        self._online: set[int] = set()
        self._torque = {s.id: False for s in servo_map.servos}
        self._sign: dict[int, int] = {s.id: 1 for s in servo_map.servos}
        self._offset: dict[int, int] = {s.id: 0 for s in servo_map.servos}
        self._profile_velocity = int(profile_velocity)
        self._profile_acceleration = int(profile_acceleration)
        self._current_limit = int(current_limit)     # EEPROM ceiling (~mA), written at connect
        # goal current is the RAM force cap (~mA); it must never exceed the EEPROM ceiling or the
        # servo silently clamps it — clamp here so the effective cap is what we think it is.
        if int(goal_current) > int(current_limit):
            print(f"[handlab] goal_current {goal_current} > current_limit {current_limit}; clamping")
        self._goal_current = min(int(goal_current), int(current_limit))  # MOVE-level force cap
        # "grasp firm, hold gentle": a SETTLED servo drops from goal_current to hold_current —
        # saves heat AND keeps the 5 V pack in budget (9x move-current can exceed a 10 A supply).
        # None -> feature off (hold == move); set hold_current < goal_current in the build to enable.
        self._hold_current = (min(int(hold_current), self._current_limit)
                              if hold_current is not None else self._goal_current)
        # Position PID gains (RAM — reset to firmware defaults on power-cycle, and any external tool
        # e.g. Dynamixel Wizard can leave arbitrary values). We PIN them at every connect so the hand
        # tracks identically each session: a weak Pos_P (e.g. a leftover 250 vs the 900 default) makes
        # small goal changes fall below the tendon break-loose force → a dead zone where fine moves do
        # nothing while large ones "jump" (measured 13.07.). I=0 by design: integral winds up against
        # the elastic return (a permanent load) and drifts the servo into its current cap.
        self._pos_p, self._pos_i, self._pos_d = int(pos_p), int(pos_i), int(pos_d)
        self._cur_level: dict[int, int] = {}   # Goal Current (mA) currently written per servo
        self._settle: dict[int, int] = {}      # consecutive not-moving telemetry ticks per servo
        self._tick = 0
        self._hw_error: dict[int, int] = {}
        # Absolute-encoder home: the shaft angle (raw % 4096) at the true home pose, stored PERSISTENTLY
        # (survives power-cycle — the XL330 magnetic encoder is absolute per revolution, verified 0.1°).
        # Lets apply_home_from_shaft() recover home on any connect WITHOUT the hand being at home —
        # essential because spread+flex have no return spring (they stay wherever left), so plain
        # delta-home (capture_zero) drifts. Unambiguous only within ±180° of home; a joint left past
        # that (e.g. thumb flex, geometry to +190°, no spring) is caught by _HOME_FLIP_GUARD and
        # delta-homed instead (see home_at_connect) rather than recovered a revolution wrong.
        self._home_shaft: dict[int, int] = {}
        self._goal: dict[int, int] = {}    # last INTENDED goal (logical ticks) per servo — see goals()
        self.set_calibration(calibration or {})

    # ---- calibration ----
    def set_calibration(self, cal: dict[int, dict]) -> None:
        for sid, c in cal.items():
            sid = int(sid)
            if "sign" in c and c["sign"] is not None:
                self._sign[sid] = int(c["sign"])
            if c.get("offset") is not None:
                self._offset[sid] = int(c["offset"])
            if c.get("home_shaft") is not None:
                self._home_shaft[sid] = int(c["home_shaft"])

    def set_sign(self, servo_id: int, sign: int) -> None:
        self._sign[int(servo_id)] = 1 if int(sign) >= 0 else -1

    def calibration(self) -> dict[int, dict]:
        # Persistable calibration = the wiring `sign` ONLY. The delta-home `offset` is RUNTIME:
        # it is re-measured by capture_zero() on every connect (Extended Position wraps after a
        # power cycle, so a frozen offset points nowhere). Emitting it here used to let
        # save_calibration() freeze it into calibration.yaml, which _make_driver then re-applied on
        # every rebuild-while-connected — clobbering the fresh capture_zero and making home diverge.
        # home_shaft IS persistable (unlike the runtime offset): the absolute encoder is power-cycle
        # stable, so a stored shaft angle recovers home on every connect (see apply_home_from_shaft).
        return {sid: {"sign": self._sign.get(sid, 1),
                      **({"home_shaft": self._home_shaft[sid]} if sid in self._home_shaft else {})}
                for sid in (s.id for s in self._map.servos)}

    def capture_zero(self, only_ids: set[int] | None = None) -> dict[int, int]:
        """Hand at the stretched rest pose (elastics, torque off) -> per-servo offset =
        present wire ticks - CENTER. Independent of sign (delta is 0 at rest).
        only_ids: restrict the capture to these servos (per-finger re-zero — e.g. anchor a
        freshly-strung thumb at its home pose without re-anchoring the calibrated index)."""
        for s in self._map.servos:
            if only_ids is not None and s.id not in only_ids:
                continue
            if s.id in self._online:
                self._offset[s.id] = self._read(s.id, _ADDR_PRESENT_POSITION, 4, signed=True) - CENTER
        return dict(self._offset)

    def capture_home_shaft(self, only_ids: set[int] | None = None) -> dict[int, int]:
        """Record the ABSOLUTE encoder shaft angle (raw % 4096) at the CURRENT pose as the persistent
        home reference — call with the hand held at the true home (stretched). Unlike capture_zero
        ('this pose is 0° right now'), the stored shaft angle lets apply_home_from_shaft() recover
        home on any LATER connect without the hand being at home. Also sets the live offset now."""
        for s in self._map.servos:
            if only_ids is not None and s.id not in only_ids:
                continue
            if s.id in self._online:
                raw = self._read(s.id, _ADDR_PRESENT_POSITION, 4, signed=True)
                self._home_shaft[s.id] = raw % 4096
                self._offset[s.id] = raw - CENTER          # home is here now → offset = plain delta
        return {sid: self._home_shaft[sid] for sid in self._home_shaft}

    def apply_home_from_shaft(self, only_ids: set[int] | None = None) -> dict[int, int]:
        """Recover home from the stored absolute shaft angles: set each servo's offset so logical 0°
        lands at the stored home shaft, REGARDLESS of the current pose. Reads the live position and
        wraps the shaft delta to ±180°. Returns ONLY the servos it confidently homed — a servo whose
        shaft sits within _HOME_FLIP_GUARD of the ±180° flip is skipped (ambiguous revolution), so the
        caller (home_at_connect) delta-homes it instead of risking a full-turn-wrong home."""
        done = {}
        for s in self._map.servos:
            if only_ids is not None and s.id not in only_ids:
                continue
            hs = self._home_shaft.get(s.id)
            if hs is None or s.id not in self._online:
                continue
            raw = self._read(s.id, _ADDR_PRESENT_POSITION, 4, signed=True)
            delta = ((hs - (raw % 4096) + CENTER) % 4096) - CENTER   # ticks from here to home, ±180°
            if abs(delta) >= _HOME_FLIP_GUARD:                       # too near the flip → don't trust
                continue
            self._offset[s.id] = raw + delta - CENTER                # place home at the stored shaft
            done[s.id] = hs
        return done

    def home_at_connect(self) -> dict[int, list]:
        """Connect-time home: recover from the stored absolute home_shaft where confident, and
        delta-home (capture_zero) EVERY other online servo — per servo, not all-or-nothing. A servo
        with no stored home, or one skipped near the ±180° flip, is delta-homed (current pose = 0°,
        never a violent move) rather than left at the bogus offset=0. Returns what happened."""
        homed = self.apply_home_from_shaft()
        online = {s.id for s in self._map.servos if s.id in self._online}
        near_flip = {s.id for s in self._map.servos                     # had a home but was skipped
                     if s.id in online and s.id in self._home_shaft and s.id not in homed}
        missing = online - set(homed)                                  # everything not absolute-homed
        if missing:
            self.capture_zero(only_ids=missing)
        # seed the intended-goal cache from the just-homed present position → idle action = hold
        # current pose (no empty/NaN action before the first real command). See goals().
        for sid in online:
            self._goal[sid] = self.read_position(sid)
        return {"absolute": sorted(homed), "delta": sorted(missing - near_flip),
                "near_flip": sorted(near_flip)}

    def _to_wire(self, sid: int, logical: int) -> int:
        return CENTER + self._offset.get(sid, 0) + self._sign.get(sid, 1) * (int(logical) - CENTER)

    def _to_logical(self, sid: int, wire: int) -> int:
        return CENTER + self._sign.get(sid, 1) * (int(wire) - CENTER - self._offset.get(sid, 0))

    # ---- lifecycle ----
    def connect(self, port: str, baud: int) -> None:
        import dynamixel_sdk as dxl   # lazy — optional dependency ([hardware] extra)
        self._dxl = dxl
        self._port = dxl.PortHandler(port)
        self._packet = dxl.PacketHandler(2.0)
        if not self._port.openPort():
            raise ConnectionError(f"could not open serial port {port!r}")
        if not self._port.setBaudRate(baud):
            self._port.closePort()
            raise ConnectionError(f"could not set baud rate {baud}")
        self._online = set(self.ping_all())
        self._sync = dxl.GroupSyncRead(self._port, self._packet, _BLK_ADDR, _BLK_LEN)
        self._sync_write = dxl.GroupSyncWrite(self._port, self._packet, _ADDR_GOAL_POSITION, 4)
        self._last_wire.clear()
        for sid in self._online:
            self._configure(sid)
            self._sync.addParam(sid)

    def _configure(self, sid: int) -> None:
        """Full per-servo setup (connect + after reboot): torque off, current-based position
        mode, EEPROM current ceiling, then the RAM caps. Order matters — mode + current limit
        are EEPROM and writable only with torque OFF."""
        self._last_wire.pop(sid, None)          # goal register state unknown after (re)config
        self._write1(sid, _ADDR_TORQUE_ENABLE, 0)                   # config needs torque off
        self._write1(sid, _ADDR_OPERATING_MODE, _CURRENT_POSITION_MODE)
        # Verify the mode actually took: current-based position is MULTI-TURN, which the thumb
        # flex NEEDS (it swings to +190deg = wire ticks > the single-turn 0..4095 range). If a
        # servo came up in single-turn Position mode (mode 3) — factory reset, a dropped EEPROM
        # write — a goal past 4095 would wrap and slam the joint. Refuse loudly instead.
        mode = self._read(sid, _ADDR_OPERATING_MODE, 1)
        if mode != _CURRENT_POSITION_MODE:
            raise IOError(f"servo {sid}: operating mode {mode} != current-based position "
                          f"({_CURRENT_POSITION_MODE}) — refusing (thumb multi-turn safety)")
        self._write2(sid, _ADDR_CURRENT_LIMIT, self._current_limit & 0xFFFF)
        # Pin the position PID gains (RAM) so tracking is deterministic every connect — see __init__.
        self._write2(sid, _ADDR_POS_P_GAIN, self._pos_p & 0xFFFF)
        self._write2(sid, _ADDR_POS_I_GAIN, self._pos_i & 0xFFFF)
        self._write2(sid, _ADDR_POS_D_GAIN, self._pos_d & 0xFFFF)
        self._write_motion_caps(sid)

    def _apply_current(self, sid: int, mA: int) -> None:
        """Write Goal Current (the force cap, reg 102) only when it changes — lets us switch the
        move<->hold levels without spamming the bus every telemetry tick."""
        if self._cur_level.get(sid) != mA:
            self._write2(sid, _ADDR_GOAL_CURRENT, mA & 0xFFFF)
            self._cur_level[sid] = mA

    def _write_motion_caps(self, sid: int) -> None:
        # factory default 0 = MAX speed; cap it so the first real moves are gentle.
        # goal current = the compliant force cap (RAM — resets on reboot, so re-assert here)
        self._write4(sid, _ADDR_PROFILE_ACCEL, self._profile_acceleration & 0xFFFFFFFF)
        self._write4(sid, _ADDR_PROFILE_VELOCITY, self._profile_velocity & 0xFFFFFFFF)
        self._cur_level.pop(sid, None)                 # RAM reset on reboot -> force a re-write
        self._apply_current(sid, self._goal_current)   # (re)start at the move-level force cap
        self._settle[sid] = 0

    def disconnect(self) -> None:
        if self._port is not None:
            for sid in list(self._online):
                try:
                    self._write1(sid, _ADDR_TORQUE_ENABLE, 0)       # leave the hand limp
                except Exception:
                    pass
            self._port.closePort()
        self._port = self._sync = self._sync_write = None
        self._last_wire.clear()
        self._online = set()

    def ping_all(self) -> list[int]:
        if self._port is None:
            return []
        found = []
        for s in self._map.servos:
            _, result, _ = self._packet.ping(self._port, s.id)
            if result == self._dxl.COMM_SUCCESS:
                found.append(s.id)
        return sorted(found)

    # ---- telemetry ----
    def _read_block(self) -> dict[int, dict]:
        """One GroupSyncRead of the moving..temp block for all online servos.
        Returns {id: {moving, current, position(wire), voltage, temp}}. Isolated so the
        field decode is testable without the SDK (override in tests)."""
        out: dict[int, dict] = {}
        if self._sync is None:
            return out
        self._sync.txRxPacket()
        for s in self._map.servos:
            if s.id not in self._online:
                continue
            if not self._sync.isAvailable(s.id, _BLK_ADDR, _BLK_LEN):
                self._online.discard(s.id)          # dropped — the periodic re-ping re-adopts it
                try:
                    self._sync.removeParam(s.id)    # so a later addParam works cleanly
                except Exception:
                    pass
                continue
            g = self._sync.getData
            out[s.id] = {
                "moving": g(s.id, _ADDR_MOVING, 1),
                "current": _i16(g(s.id, _ADDR_PRESENT_CURRENT, 2)),
                "position": _i32(g(s.id, _ADDR_PRESENT_POSITION, 4)),
                "voltage": g(s.id, _ADDR_PRESENT_VOLTAGE, 2) / 10.0,
                "temp": g(s.id, _ADDR_PRESENT_TEMP, 1),
            }
        return out

    def read_states(self) -> dict[int, ServoState]:
        self._tick += 1
        try:
            block = self._read_block()
        except Exception:
            block = {}
        # Housekeeping ROUND-ROBIN: one servo per tick instead of "every 10th tick, all servos" —
        # the old burst was 18 blocking reads + re-pings in one tick = a periodic 10-18 ms GUI
        # stutter (the driver lives on the GUI thread). Per-servo refresh stays ~0.9 s at 10 Hz.
        servos = self._map.servos
        if servos:
            s = servos[self._tick % len(servos)]
            if s.id in self._online:
                try:
                    # hw_error: readable now that _check tolerates the alert bit
                    self._hw_error[s.id] = self._read(s.id, _ADDR_HW_ERROR, 1)
                    # torque TRUTH from the wire — catches self-shutdown (overload) and
                    # brownout resets, where the servo dropped torque behind our back
                    self._torque[s.id] = bool(self._read(s.id, _ADDR_TORQUE_ENABLE, 1))
                except Exception:
                    pass
            else:
                # auto-re-ping: re-adopt a servo that fell out of the sync read (bus hiccup)
                try:
                    _, result, _ = self._packet.ping(self._port, s.id)
                    if result == self._dxl.COMM_SUCCESS:
                        self._online.add(s.id)
                        self._sync.addParam(s.id)
                except Exception:
                    pass
        out: dict[int, ServoState] = {}
        for s in self._map.servos:
            b = block.get(s.id)
            if b is None:
                out[s.id] = ServoState(id=s.id, present_position=0)
                continue
            out[s.id] = ServoState(
                id=s.id, present_position=self._to_logical(s.id, b["position"]),
                present_current=b["current"], present_temperature=b["temp"],
                present_voltage=b["voltage"], moving=bool(b["moving"]),
                torque_enabled=self._torque.get(s.id, False), hw_error=self._hw_error.get(s.id, 0),
            )
            # grasp firm, hold gentle: a settled (not-moving) servo drops to hold_current; a new
            # set_goal_position re-arms the move cap. Skipped entirely when the feature is off.
            if self._hold_current < self._goal_current and self._torque.get(s.id, False):
                if b["moving"]:
                    self._settle[s.id] = 0
                else:
                    self._settle[s.id] = self._settle.get(s.id, 0) + 1
                    if self._settle[s.id] >= _HOLD_AFTER_TICKS:
                        self._apply_current(s.id, self._hold_current)
        return out

    # ---- control ----
    def set_torque(self, servo_id: int, enable: bool) -> None:
        if servo_id in self._online:
            if enable:
                # HOLD the current position (Goal := Present) before enabling — never jump.
                present = self._read(servo_id, _ADDR_PRESENT_POSITION, 4, signed=True)
                self._write4(servo_id, _ADDR_GOAL_POSITION, present & 0xFFFFFFFF)
                self._last_wire[servo_id] = present & 0xFFFFFFFF   # keep the batch dedup coherent
                self._write_motion_caps(servo_id)   # re-assert caps (mode/reset safety)
                self._write1(servo_id, _ADDR_TORQUE_ENABLE, 1)
            else:
                self._write1(servo_id, _ADDR_TORQUE_ENABLE, 0)
        self._torque[servo_id] = enable

    def set_goal_position(self, servo_id: int, logical_ticks: int) -> None:
        # cache the INTENDED goal BEFORE the online/torque guard — the demo recorder's action label
        # must reflect what was commanded, not whether it reached the wire (see HandDriver.goals()).
        self._goal[int(servo_id)] = int(logical_ticks)
        if servo_id in self._online and self._torque.get(servo_id, False):
            if self._hold_current < self._goal_current:     # feature on: a new move -> full force
                self._apply_current(servo_id, self._goal_current)
                self._settle[servo_id] = 0
            wire = self._to_wire(servo_id, logical_ticks) & 0xFFFFFFFF
            self._write4(servo_id, _ADDR_GOAL_POSITION, wire)
            self._last_wire[servo_id] = wire                # keep the batch dedup cache coherent

    def set_goal_positions(self, goals: dict[int, int]) -> None:
        """Teleop batch path: ONE GroupSyncWrite broadcast (TX-only, no per-servo status round
        trips) instead of N blocking `_write4`s — a 30 Hz 9-servo frame becomes ~57 bytes, and a
        still hand becomes ZERO bytes (per-servo wire dedup via `_last_wire`). Same gates as the
        single write: offline/torque-off servos are skipped on the wire but their INTENDED goal is
        still cached for the recorder's action label."""
        if self._sync_write is None:                        # not connected → intended-goal cache only
            for sid, ticks in goals.items():
                self._goal[int(sid)] = int(ticks)
            return
        dxl = self._dxl
        self._sync_write.clearParam()
        added = False
        for sid, ticks in goals.items():
            sid = int(sid)
            self._goal[sid] = int(ticks)                    # action label BEFORE any gate
            if sid not in self._online or not self._torque.get(sid, False):
                continue                                    # same gate as set_goal_position
            wire = self._to_wire(sid, ticks) & 0xFFFFFFFF
            if self._last_wire.get(sid) == wire:
                continue                                    # unchanged → zero bytes for this servo
            if self._hold_current < self._goal_current:     # move-force re-arm (feature on)
                self._apply_current(sid, self._goal_current)
                self._settle[sid] = 0
            self._sync_write.addParam(sid, [
                dxl.DXL_LOBYTE(dxl.DXL_LOWORD(wire)), dxl.DXL_HIBYTE(dxl.DXL_LOWORD(wire)),
                dxl.DXL_LOBYTE(dxl.DXL_HIWORD(wire)), dxl.DXL_HIBYTE(dxl.DXL_HIWORD(wire))])
            self._last_wire[sid] = wire
            added = True
        if not added:
            return
        result = self._sync_write.txPacket()
        if result != dxl.COMM_SUCCESS:
            self._last_wire.clear()                         # wire state unknown → next frame resends all
            raise IOError(f"GroupSyncWrite failed: {self._packet.getTxRxResult(result)}")

    def read_position(self, servo_id: int) -> int:
        if servo_id not in self._online:
            return 0
        return self._to_logical(servo_id, self._read(servo_id, _ADDR_PRESENT_POSITION, 4, signed=True))

    # ---- recovery ----
    def reboot(self, servo_id: int) -> None:
        """In-band revive after a latched hardware error (overload/overheat): the REBOOT
        instruction is the only way to clear reg 70 short of a power cycle. Afterwards the
        servo boots torque-OFF with RAM regs reset (profiles = 0 = MAX!) and its position
        re-wrapped — so re-apply the full config and expect a per-finger re-zero next."""
        import time
        sid = int(servo_id)
        self._packet.reboot(self._port, sid)    # no _check: the reply still carries the alert bit
        time.sleep(0.6)                         # boot settle
        self._hw_error[sid] = 0
        self._torque[sid] = False               # it boots limp; user re-engages explicitly
        self._configure(sid)                    # mode + current ceiling + caps (torque stays off)
        if sid not in self._online:
            self._online.add(sid)
            try:
                self._sync.addParam(sid)
            except Exception:
                pass

    # ---- raw register access ----
    def _check(self, result: int, error: int) -> None:
        if result != self._dxl.COMM_SUCCESS:
            raise IOError(self._packet.getTxRxResult(result))
        # Mask the protocol-2.0 ALERT bit: it flags a LATCHED hardware error (read reg 70 for
        # details) and rides on every status packet — treating it as an IO failure used to make
        # EVERY write to an overloaded servo raise, and a reconnect against it dropped the whole
        # driver back to sim ("everything frozen"). Real command errors are the low bits.
        if error & ~_ALERT_BIT != 0:
            raise IOError(self._packet.getRxPacketError(error))

    def _read(self, sid: int, addr: int, size: int, signed: bool = False) -> int:
        fn = {1: self._packet.read1ByteTxRx, 2: self._packet.read2ByteTxRx,
              4: self._packet.read4ByteTxRx}[size]
        value, result, error = fn(self._port, sid, addr)
        self._check(result, error)
        return (_i16(value) if size == 2 else _i32(value)) if signed else value

    def _write1(self, sid: int, addr: int, value: int) -> None:
        result, error = self._packet.write1ByteTxRx(self._port, sid, addr, value)
        self._check(result, error)

    def _write2(self, sid: int, addr: int, value: int) -> None:
        result, error = self._packet.write2ByteTxRx(self._port, sid, addr, value)
        self._check(result, error)

    def _write4(self, sid: int, addr: int, value: int) -> None:
        result, error = self._packet.write4ByteTxRx(self._port, sid, addr, value)
        self._check(result, error)
