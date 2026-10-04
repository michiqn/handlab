"""HAL tests: the servo map carries joints, the mock driver behaves, the MuJoCo
driver really drives the composed twin, and calibration offsets round-trip."""

import time

import pytest

from handlab import compose, library_io
from handlab.hal.base import CENTER, rad_to_ticks, ticks_to_rad
from handlab.hal.mock_driver import MockDriver
from handlab.hal.mujoco_driver import MuJoCoDriver


def test_servo_map_carries_prefixed_joints():
    b = library_io.load_build("claw3f")
    sm = compose.derive_servo_map(b)
    # actuator index_curl is backed by the finger-prefixed claw joint
    curl = next(s for s in sm.servos if s.dof == "curl" and s.actuator.startswith("index_"))
    assert curl.actuator == "index_curl"
    assert curl.joint == "index_index_proximal_Umdrehung-7"


def test_ticks_rad_roundtrip():
    assert rad_to_ticks(0.0) == CENTER
    assert abs(ticks_to_rad(rad_to_ticks(1.0)) - 1.0) < 1e-2


def test_mock_driver_torque_and_backdrive():
    sm = compose.derive_servo_map(library_io.load_build("claw3f"))
    d = MockDriver(sm)
    d.connect("", 0)
    sid = sm.servos[0].id
    # torque off: goal teleports (backdrive)
    d.set_goal_position(sid, 3000)
    assert d.read_position(sid) == 3000
    # torque on: position eases toward goal
    d.set_torque(sid, True)
    d.set_goal_position(sid, 2000)
    for _ in range(30):
        d.read_states()
    assert abs(d.read_position(sid) - 2000) < 50


def test_mujoco_driver_drives_the_twin(tmp_path):
    pytest.importorskip("mujoco")
    from handlab.sim.twin import Twin

    b = library_io.load_build("claw3f")
    c = compose.compose(b, out_dir=tmp_path)
    twin = Twin(str(c.scene_path))
    twin.start()
    try:
        d = MuJoCoDriver(c.servo_map, twin)
        d.connect("", 0)
        assert d.ping_all() == sorted(s.id for s in c.servo_map.servos)

        flex = next(s for s in c.servo_map.servos
                    if s.dof == "flex" and s.actuator.startswith("index_"))
        d.set_torque(flex.id, True)
        target = rad_to_ticks(1.0)              # ~57 deg flex
        d.set_goal_position(flex.id, target)
        time.sleep(0.6)                          # let the position servo pull
        pos = d.read_states()[flex.id].present_position
        assert abs(pos - target) < 200, f"servo didn't track: {pos} vs {target}"

        # torque off: the elastic return EASES toward the rest pose (0 rad = CENTER ticks)
        # on the read_states beat, and new goals are ignored
        d.set_torque(flex.id, False)
        d.set_goal_position(flex.id, rad_to_ticks(1.6))   # must be ignored
        for _ in range(20):                                # ~2 s of telemetry beats
            d.read_states()
            time.sleep(0.1)
        pos2 = d.read_states()[flex.id].present_position
        assert abs(pos2 - CENTER) < 200, f"limp servo didn't relax to rest: {pos2}"
    finally:
        twin.stop()


def test_calibration_roundtrip(tmp_path, monkeypatch):
    # isolate BOTH the canonical user-dir path (writes/reads) and the bundled seed (read fallback)
    monkeypatch.setattr(library_io, "USER_CALIBRATION_FILE", tmp_path / "calibration.yaml")
    monkeypatch.setattr(library_io, "CALIBRATION_FILE", tmp_path / "bundled_seed.yaml")
    assert library_io.load_calibration() == {}
    # form: sign persists, offset only if set, range_deg defaults None
    library_io.save_calibration({1: {"sign": -1, "offset": 137}, 2: {"sign": 1}})
    cal = library_io.load_calibration()
    assert cal[1] == {"sign": -1, "offset": 137, "range_deg": None, "home_shaft": None}
    assert cal[2] == {"sign": 1, "offset": None, "range_deg": None, "home_shaft": None}
    # range_deg (the "edit limits" override) round-trips and survives a hardware-only save
    # (setServoSign passes only sign/offset from the driver) — it must NOT clobber the range.
    library_io.save_calibration({1: {"sign": 1, "range_deg": [-2, 180]}})
    library_io.save_calibration({1: {"sign": -1}})           # hardware sign flip, no range_deg key
    cal = library_io.load_calibration()
    assert cal[1] == {"sign": -1, "offset": None, "range_deg": [-2.0, 180.0], "home_shaft": None}
    library_io.save_calibration({1: {"sign": 1, "range_deg": None}})   # explicit None clears it
    assert library_io.load_calibration()[1]["range_deg"] is None
    # home_shaft (absolute-encoder home) round-trips AND is preserved by a sign-only save,
    # just like range_deg — else a setServoSign would wipe the stored home.
    library_io.save_calibration({1: {"sign": 1, "home_shaft": 2500}})
    library_io.save_calibration({1: {"sign": -1}})           # sign-only write, no home_shaft key
    assert library_io.load_calibration()[1]["home_shaft"] == 2500


def _dxl(sm, **cal):
    from handlab.hal.dynamixel_driver import DynamixelDriver
    d = DynamixelDriver(sm, calibration=cal.get("calibration"))
    d._online = set(s.id for s in sm.servos)
    for s in sm.servos:
        d._torque[s.id] = True
    return d


def test_absolute_home_recovers_regardless_of_pose():
    """Stored absolute shaft angle recovers home on connect from ANY pose (spread+flex have no
    return spring, so plain delta-home drifts). Home maps back to the stored shaft, the current
    pose reads its true offset, and the ±180° wrap is handled across the 0/4096 boundary."""
    sm = compose.derive_servo_map(library_io.load_build("claw3f"))
    d = _dxl(sm)
    pos = {}
    d._read = lambda sid, addr, size=1, signed=False: pos.get(sid, 0)
    # 1) capture home while the hand is AT home
    pos[1] = 2500                                  # home shaft 2500 (mid revolution)
    pos[4] = 100                                   # home near the 0/4096 boundary (wrap case)
    d.capture_home_shaft(only_ids={1, 4})
    assert d._home_shaft[1] == 2500 and d._home_shaft[4] == 100
    assert d._to_wire(1, CENTER) % 4096 == 2500    # commanding home 0° drives to the home shaft
    assert d._to_wire(4, CENTER) % 4096 == 100
    # 2) hand left elsewhere, fresh connect: recover home from the stored shaft, any pose
    pos[1] = 2600                                  # joint 1 now +100 ticks from home
    pos[4] = 4000                                  # joint 4 now -196 ticks, across the boundary
    d._offset[1] = d._offset[4] = 0                # wipe (as if fresh)
    homed = d.apply_home_from_shaft(only_ids={1, 4})
    assert set(homed) == {1, 4}
    assert d._to_wire(1, CENTER) % 4096 == 2500    # home STILL lands on the stored shaft
    assert d._to_wire(4, CENTER) % 4096 == 100
    assert d._to_logical(1, pos[1]) == CENTER + 100   # current pose reads its true angle
    assert d._to_logical(4, pos[4]) == CENTER - 196   # wrapped correctly across 0
    # a servo without a stored home is skipped (falls back to delta-home in the bridge)
    assert 2 not in d.apply_home_from_shaft(only_ids={2})


def test_home_at_connect_per_servo_fallback_and_flip_guard():
    """home_at_connect must be PER-SERVO, not all-or-nothing: a servo with no stored home, or one
    left near the ±180° flip, is delta-homed (current pose = 0° — safe) instead of left at the bogus
    offset=0. Covers sign=-1 home placement and the flip guard on a spring-less joint."""
    sm = compose.derive_servo_map(library_io.load_build("claw3f"))
    d = _dxl(sm)
    pos = {}
    d._read = lambda sid, addr, size=1, signed=False: pos.get(sid, 0)
    d._sign[4] = -1                        # real servo 4 (thumb spread) is wired -1
    d._home_shaft = {1: 2000, 4: 2000, 2: 2000}
    pos[1] = 2100                          # +100 from home  → absolute-homed (sign +1)
    pos[4] = 1700                          # -300 from home  → absolute-homed (sign -1)
    pos[2] = 2000 + 1950                   # 1950 ticks away → past _HOME_FLIP_GUARD → skip+delta
    pos[7] = 3000                          # no stored home  → delta-home (NOT left at offset=0)
    res = d.home_at_connect()
    assert 1 in res["absolute"] and 4 in res["absolute"]
    assert res["near_flip"] == [2] and 2 not in res["absolute"]
    assert 7 in res["delta"] and 2 not in res["delta"]
    # 1 & 4 recover home to the stored shaft regardless of sign
    assert d._to_wire(1, CENTER) % 4096 == 2000
    assert d._to_wire(4, CENTER) % 4096 == 2000
    assert d._to_logical(4, pos[4]) == CENTER + 300      # sign -1: -300 wire → +300 logical
    # near-flip (2) and no-home (7) fell back to delta-home → current pose reads 0°, no violent move
    assert d._to_logical(2, pos[2]) == CENTER
    assert d._to_logical(7, pos[7]) == CENTER


def test_dynamixel_sign_and_offset_translate_goals():
    """wire = CENTER + offset + sign*(logical - CENTER); read-back is the inverse."""
    sm = compose.derive_servo_map(library_io.load_build("claw3f"))
    d = _dxl(sm, calibration={1: {"sign": 1, "offset": 100}, 2: {"sign": -1, "offset": 0}})
    written = {}
    d._write4 = lambda sid, addr, value: written.update({(sid, addr): value})
    d.set_goal_position(1, CENTER + 200)       # offset only
    d.set_goal_position(2, CENTER + 200)       # inverted
    from handlab.hal.dynamixel_driver import _ADDR_GOAL_POSITION as G
    assert written[(1, G)] == CENTER + 100 + 200
    assert written[(2, G)] == CENTER - 200
    # round-trip: wire CENTER-200 on servo 2 reads back as logical CENTER+200
    assert d._to_logical(2, CENTER - 200) == CENTER + 200


def test_dynamixel_torque_enable_holds_position():
    """Torque enable must write Goal := Present FIRST (no jump), then enable."""
    sm = compose.derive_servo_map(library_io.load_build("claw3f"))
    d = _dxl(sm)
    d._torque[1] = False
    calls = []
    d._read = lambda sid, addr, size, signed=False: 2600     # present position (wire)
    d._write4 = lambda sid, addr, value: calls.append(("w4", addr, value))
    d._write2 = lambda sid, addr, value: calls.append(("w2", addr, value))
    d._write1 = lambda sid, addr, value: calls.append(("w1", addr, value))
    d.set_torque(1, True)
    from handlab.hal.dynamixel_driver import _ADDR_GOAL_POSITION as G, _ADDR_TORQUE_ENABLE as T
    goal_idx = next(i for i, c in enumerate(calls) if c[0] == "w4" and c[1] == G)
    torque_idx = next(i for i, c in enumerate(calls) if c == ("w1", T, 1))
    assert calls[goal_idx] == ("w4", G, 2600)               # goal := present
    assert goal_idx < torque_idx                            # before enabling torque


def test_dynamixel_velocity_caps_written_on_torque_enable():
    sm = compose.derive_servo_map(library_io.load_build("claw3f"))
    from handlab.hal.dynamixel_driver import (DynamixelDriver, _ADDR_PROFILE_VELOCITY,
                                              _ADDR_PROFILE_ACCEL, _ADDR_GOAL_CURRENT)
    d = DynamixelDriver(sm, profile_velocity=42, profile_acceleration=7, goal_current=222)
    d._online = {1}; d._torque[1] = False
    writes = []
    d._read = lambda *a, **k: 2048
    d._write4 = lambda sid, addr, value: writes.append((addr, value))
    d._write2 = lambda sid, addr, value: writes.append((addr, value))
    d._write1 = lambda *a, **k: None
    d.set_torque(1, True)
    assert (_ADDR_PROFILE_VELOCITY, 42) in writes
    assert (_ADDR_PROFILE_ACCEL, 7) in writes
    assert (_ADDR_GOAL_CURRENT, 222) in writes    # compliant force cap re-asserted with the caps


def test_dynamixel_grasp_firm_hold_gentle():
    """goal_current stays at the MOVE level while commanding/moving, then drops to hold_current
    once a servo settles (not moving for _HOLD_AFTER_TICKS) — a new goal re-arms MOVE."""
    sm = compose.derive_servo_map(library_io.load_build("claw3f"))
    from handlab.hal.dynamixel_driver import (DynamixelDriver, _ADDR_GOAL_CURRENT,
                                              _HOLD_AFTER_TICKS)
    d = DynamixelDriver(sm, current_limit=1500, goal_current=1000, hold_current=400)
    d._online = {1}; d._torque[1] = True
    d._read = lambda *a, **k: 0
    cur = []
    d._write2 = lambda sid, addr, v: cur.append(v) if addr == _ADDR_GOAL_CURRENT else None
    d._write4 = lambda *a, **k: None

    d.set_goal_position(1, 2200)                 # commanding a move -> MOVE cap
    assert cur[-1] == 1000
    d._read_block = lambda: {1: {"moving": 1, "current": 0, "position": 2200,
                                 "voltage": 5.0, "temp": 30}}
    d.read_states()                              # still moving -> stays at MOVE
    assert cur[-1] == 1000
    d._read_block = lambda: {1: {"moving": 0, "current": 0, "position": 2200,
                                 "voltage": 5.0, "temp": 30}}
    for _ in range(_HOLD_AFTER_TICKS):           # settled for the dwell -> drops to HOLD
        d.read_states()
    assert cur[-1] == 400
    d.set_goal_position(1, 1800)                 # a new move re-arms MOVE
    assert cur[-1] == 1000


def test_dynamixel_read_block_decodes_with_sign():
    """read_states maps the synced block to logical state (sign/offset applied)."""
    sm = compose.derive_servo_map(library_io.load_build("claw3f"))
    d = _dxl(sm, calibration={1: {"sign": -1, "offset": 10}})
    d._read_block = lambda: {1: {"moving": 1, "current": 5, "position": 1048,
                                 "voltage": 5.0, "temp": 31}}
    st = d.read_states()[1]
    assert st.present_position == d._to_logical(1, 1048)
    assert st.moving is True and st.present_temperature == 31


def test_dynamixel_check_masks_alert_bit():
    """The protocol-2.0 ALERT bit (0x80) flags a LATCHED hardware error — it is NOT a comm
    failure. Treating it as one made every write to an overloaded servo raise, and a reconnect
    against it dropped the whole hand to sim ('everything frozen')."""
    import types
    sm = compose.derive_servo_map(library_io.load_build("claw3f"))
    d = _dxl(sm)
    d._dxl = types.SimpleNamespace(COMM_SUCCESS=0)
    d._packet = types.SimpleNamespace(getTxRxResult=lambda r: f"comm {r}",
                                      getRxPacketError=lambda e: f"pkt {e}")
    d._check(0, 0x80)                        # alert alone: tolerated
    with pytest.raises(IOError):
        d._check(0, 0x80 | 0x01)             # alert + real command error: raises
    with pytest.raises(IOError):
        d._check(1, 0)                       # comm failure: raises


def test_dynamixel_housekeeping_syncs_torque_and_repings():
    """Housekeeping is ROUND-ROBIN now (perf: the old every-10th-tick burst of 18 reads was a
    periodic 10-18 ms GUI stutter): ONE servo per tick — online → hw_error + Torque Enable from
    the WIRE (catches overload self-shutdown / brownouts); offline → re-ping + re-adopt. Over a
    full cycle of len(servos) ticks every servo gets refreshed/re-pinged."""
    import types
    sm = compose.derive_servo_map(library_io.load_build("claw3f"))
    d = _dxl(sm)
    d._online = {1}
    d._torque[1] = True                                  # cache believes ON
    d._read_block = lambda: {1: {"moving": 0, "current": 0, "position": 2048,
                                 "voltage": 5.0, "temp": 30}}
    from handlab.hal.dynamixel_driver import _ADDR_HW_ERROR, _ADDR_TORQUE_ENABLE
    reads = {_ADDR_HW_ERROR: 32, _ADDR_TORQUE_ENABLE: 0}  # overload latched, torque actually OFF
    d._read = lambda sid, addr, size, signed=False: reads[addr]
    d._dxl = types.SimpleNamespace(COMM_SUCCESS=0)
    pinged = []
    d._packet = types.SimpleNamespace(ping=lambda port, sid: (pinged.append(sid), 0, 0))
    d._sync = types.SimpleNamespace(addParam=lambda sid: True)
    n = len(sm.servos)
    for _ in range(n):                                    # one full round-robin cycle
        st = d.read_states()
    assert d._hw_error[1] == 32 and st[1].hw_error == 32  # servo 1 refreshed from the wire...
    assert d._torque[1] is False and st[1].torque_enabled is False
    assert set(pinged) == {2, 3, 4, 5, 6, 7, 8, 9}        # ...dropped servos re-pinged once each
    assert d._online == {1, 2, 3, 4, 5, 6, 7, 8, 9}       # ...and re-adopted


def _fake_sync_write():
    import types
    calls = {"tx": 0, "params": [], "cleared": 0}

    def add(sid, data):
        calls["params"].append((sid, list(data)))
        return True

    sw = types.SimpleNamespace(
        clearParam=lambda: calls.__setitem__("cleared", calls["cleared"] + 1),
        addParam=add,
        txPacket=lambda: (calls.__setitem__("tx", calls["tx"] + 1), 0)[1])
    return sw, calls


def test_set_goal_positions_batches_one_txpacket():
    """The teleop batch path: N servo goals → ONE GroupSyncWrite broadcast; sign/offset applied
    per servo (same _to_wire as the single write); little-endian 4-byte params."""
    import types
    sm = compose.derive_servo_map(library_io.load_build("claw3f"))
    d = _dxl(sm, calibration={1: {"sign": -1, "offset": 10}})
    d._dxl = types.SimpleNamespace(
        COMM_SUCCESS=0,
        DXL_LOBYTE=lambda v: v & 0xFF, DXL_HIBYTE=lambda v: (v >> 8) & 0xFF,
        DXL_LOWORD=lambda v: v & 0xFFFF, DXL_HIWORD=lambda v: (v >> 16) & 0xFFFF)
    sw, calls = _fake_sync_write()
    d._sync_write = sw
    d.set_goal_positions({1: 2100, 2: 2200})
    assert calls["tx"] == 1                               # ONE broadcast, not per-servo writes
    assert len(calls["params"]) == 2
    by_sid = dict(calls["params"])
    wire1 = d._to_wire(1, 2100) & 0xFFFFFFFF              # sign -1 + offset 10 applied
    assert by_sid[1] == [wire1 & 0xFF, (wire1 >> 8) & 0xFF,
                         (wire1 >> 16) & 0xFF, (wire1 >> 24) & 0xFF]
    assert d.goals()[1] == 2100 and d.goals()[2] == 2200  # action label cached


def test_set_goal_positions_gates_and_dedups():
    """Torque-off servos are skipped on the wire but keep the action label; an identical repeat
    call sends ZERO packets (wire dedup); torque-enable re-syncs the dedup cache (Goal:=Present)."""
    import types
    sm = compose.derive_servo_map(library_io.load_build("claw3f"))
    d = _dxl(sm)
    d._dxl = types.SimpleNamespace(
        COMM_SUCCESS=0,
        DXL_LOBYTE=lambda v: v & 0xFF, DXL_HIBYTE=lambda v: (v >> 8) & 0xFF,
        DXL_LOWORD=lambda v: v & 0xFFFF, DXL_HIWORD=lambda v: (v >> 16) & 0xFFFF)
    sw, calls = _fake_sync_write()
    d._sync_write = sw
    d._torque[3] = False                                  # limp servo
    d.set_goal_positions({1: 2100, 3: 2300})
    assert [sid for sid, _ in calls["params"]] == [1]     # 3 skipped on the wire...
    assert d.goals()[3] == 2300                           # ...but the INTENDED goal is recorded
    d.set_goal_positions({1: 2100, 3: 2300})              # identical repeat (a still hand)
    assert calls["tx"] == 1                               # → zero additional packets
    # torque enable writes Goal:=Present → cache must re-sync so the next differing goal SENDS
    d._read = lambda sid, addr, size, signed=False: 2048
    d._write4 = lambda *a, **k: None
    d._write1 = lambda *a, **k: None
    d._write2 = lambda *a, **k: None
    d.set_torque(1, True)
    d.set_goal_positions({1: 2100})                       # same logical goal as before...
    assert calls["tx"] == 2                               # ...but the register changed → resent


def test_set_goal_positions_default_loops_singles():
    """hal.base default: drivers without a batch path (mock/MuJoCo) fall back to single calls —
    sim teleop keeps driving the twin."""
    from handlab.hal.base import HandDriver

    class _T(HandDriver):
        driver_type = "mock"

        def __init__(self):
            self.calls = []

        def connect(self, *a): ...
        def disconnect(self): ...
        def ping_all(self):
            return []

        def set_torque(self, sid, en): ...
        def set_goal_position(self, sid, ticks):
            self.calls.append((sid, ticks))

        def read_position(self, sid):
            return 2048

        def read_states(self):
            return {}

    t = _T()
    t.set_goal_positions({1: 2100, 2: 2200})
    assert t.calls == [(1, 2100), (2, 2200)]


def test_dynamixel_reboot_reconfigures_and_stays_limp(monkeypatch):
    """reboot() clears the latched error and re-applies the FULL config (profiles are RAM →
    reset to 0 = MAX speed by the reboot!), leaves torque OFF, and re-adopts the servo."""
    import types
    monkeypatch.setattr("time.sleep", lambda s: None)     # skip the boot settle in tests
    sm = compose.derive_servo_map(library_io.load_build("claw3f"))
    d = _dxl(sm)
    calls = []
    d._write1 = lambda sid, addr, v: calls.append((addr, v))
    d._write2 = lambda sid, addr, v: calls.append((addr, v))
    d._write4 = lambda sid, addr, v: calls.append((addr, v))
    rebooted = []
    d._packet = types.SimpleNamespace(reboot=lambda port, sid: rebooted.append(sid))
    d._sync = types.SimpleNamespace(addParam=lambda sid: True)
    from handlab.hal.dynamixel_driver import _CURRENT_POSITION_MODE as _CPM
    d._read = lambda sid, addr, size=1, signed=False: _CPM   # _configure verifies the mode readback
    d._online = set(); d._torque[1] = True; d._hw_error[1] = 32
    d.reboot(1)
    from handlab.hal.dynamixel_driver import (_ADDR_TORQUE_ENABLE, _ADDR_OPERATING_MODE,
        _ADDR_CURRENT_LIMIT, _ADDR_GOAL_CURRENT, _ADDR_PROFILE_VELOCITY, _CURRENT_POSITION_MODE,
        _ADDR_POS_P_GAIN, _ADDR_POS_I_GAIN, _ADDR_POS_D_GAIN)
    assert rebooted == [1]
    assert (_ADDR_TORQUE_ENABLE, 0) in calls              # boots limp, stays limp
    assert (_ADDR_OPERATING_MODE, _CURRENT_POSITION_MODE) in calls
    assert (_ADDR_CURRENT_LIMIT, 600) in calls            # EEPROM ceiling re-applied
    assert (_ADDR_GOAL_CURRENT, 350) in calls             # RAM force cap re-applied
    assert (_ADDR_POS_P_GAIN, 900) in calls               # position PID pinned (dead-zone fix)
    assert (_ADDR_POS_I_GAIN, 0) in calls
    assert (_ADDR_POS_D_GAIN, 0) in calls
    assert any(a == _ADDR_PROFILE_VELOCITY for a, _ in calls)   # speed caps re-applied
    assert d._torque[1] is False and d._hw_error[1] == 0 and 1 in d._online


def test_load_build_parses_hardware_knobs(tmp_path):
    """hardware: YAML knobs reach the Hardware dataclass (velocity/accel used to be silently
    ignored); omitted keys keep the conservative defaults."""
    y = tmp_path / "b.yaml"
    y.write_text("name: b\nmount: palm_mount\nfingers: []\n"
                 "hardware: {profile_velocity: 99, profile_acceleration: 9, "
                 "current_limit: 500, goal_current: 222, hold_current: 111}\n")
    hw = library_io.load_build(str(y)).hardware
    assert (hw.profile_velocity, hw.profile_acceleration) == (99, 9)
    assert (hw.current_limit, hw.goal_current, hw.hold_current) == (500, 222, 111)
    # a build that OMITS the knobs keeps the conservative Hardware() defaults (hold_current off)
    y2 = tmp_path / "b2.yaml"
    y2.write_text("name: b2\nmount: palm_mount\nfingers: []\nhardware: {model: X}\n")
    d = library_io.load_build(str(y2)).hardware
    assert (d.profile_velocity, d.profile_acceleration) == (60, 20)
    assert d.current_limit == 600 and d.goal_current == 350 and d.hold_current is None
    assert (d.pos_p, d.pos_i, d.pos_d) == (900, 0, 0)     # firm default tracking, no integral windup
    # PID knobs are parseable per build (a firmer or softer hand without touching the driver)
    y3 = tmp_path / "b3.yaml"
    y3.write_text("name: b3\nmount: palm_mount\nfingers: []\nhardware: {pos_p: 640, pos_d: 30}\n")
    p = library_io.load_build(str(y3)).hardware
    assert (p.pos_p, p.pos_i, p.pos_d) == (640, 0, 30)


def test_home_frame_targets_groups_and_holds_the_rest():
    """safeHome phase frames: target dof groups -> 0° (home), the others HOLD their measured
    angle; only scoped fingers' sockets appear (other fingers untouched)."""
    import math as m
    from handlab.bridge import Bridge
    from handlab import compose as _c
    b = library_io.load_build("claw3f")
    fake = Bridge.__new__(Bridge)                         # _home_frame uses no Qt state
    fake._home_scope = [b.fingers[0]]                     # index only
    fake._ftypes = {f.id: _c._try_load(f.type) for f in b.fingers}
    states = {sid: ServoStateStub(rad_to_ticks(m.radians(30))) for sid in b.fingers[0].servos}
    frame = Bridge._home_frame(fake, states, {"curl"})    # phase 1: curls -> home
    assert list(frame.keys()) == ["index"]
    spread, flex, curl = frame["index"]
    assert curl == 0.0 and abs(spread - 30) < 0.2 and abs(flex - 30) < 0.2
    frame2 = Bridge._home_frame(fake, states, {"curl", "flex"})   # phase 2 adds flex
    assert frame2["index"][1] == 0.0 and frame2["index"][2] == 0.0
    assert abs(frame2["index"][0] - 30) < 0.2


class ServoStateStub:
    def __init__(self, pos):
        self.present_position = pos


def test_trim_idle_frames():
    """stopRecording drops static lead-in/out, keeping a margin frame each side."""
    from handlab.bridge import Bridge
    rest = {"p1": [0.0, 0.0, 0.0]}
    mid = {"p1": [0.0, 50.0, 0.0]}
    end = {"p1": [0.0, 0.0, 90.0]}
    frames = [rest] * 10 + [mid, end] + [end] * 10      # 10 idle, motion, 10 idle
    # _trim_idle only uses self via the static _frame_delta, so an uninitialised instance is fine
    out = Bridge._trim_idle(Bridge.__new__(Bridge), frames)
    assert len(out) < len(frames)            # trimmed
    assert out[0] == rest and out[-1] == end # margins kept (one rest before motion, end after)
    assert mid in out and end in out
