"""WiLoR teleop stream (§41) — the UDP receiver + the 'wilor' frame builder/step, headless.

The tracker is external (tracking/wilor_teleop.py, torch/WiLoR — not imported here). We test only the
handlab side: that `TeleopStreamReceiver` parses a UDP frame, that `_teleop_frame_stream` maps norms
→ a {socket:[deg]} frame in effective range reusing the scalar path, and that `_teleop_step_wilor`
drives (or holds on stale) through the same clamp + `_estopped` gate.
"""
import json
import math
import socket
import time
import types

from handlab import compose, library_io
from handlab.bridge import Bridge
from handlab.teleop_stream import TeleopStreamReceiver


def _send(port, obj):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.sendto(json.dumps(obj).encode(), ("127.0.0.1", port))
    s.close()


def _send_raw(port, data: bytes):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.sendto(data, ("127.0.0.1", port))
    s.close()


def _wait_latest(recv, timeout=2.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        norms, age = recv.latest()
        if norms is not None:
            return norms, age
        time.sleep(0.01)
    return recv.latest()


def _free_port():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def test_receiver_roundtrip_and_reject_garbage():
    recv = TeleopStreamReceiver(port=_free_port())
    recv.start()
    try:
        norms0, age0 = recv.latest()
        assert norms0 is None and age0 > 1e6                 # nothing arrived yet
        good = {"index": {"flex": 0.7, "curl": 0.4, "spread": -0.2}}
        _send(recv.port, {"t": 1.0, "fingers": good})
        norms, age = _wait_latest(recv)
        assert norms == good and age < 1.0
        # raw garbage + wrong-shaped payloads must NOT clobber the last good frame
        _send_raw(recv.port, b"not json at all")
        _send(recv.port, {"fingers": "nope"})
        time.sleep(0.05)
        again, _ = recv.latest()
        assert again == good
    finally:
        recv.stop()


def _wilor_bridge():
    b = library_io.load_build("claw3f")
    sm = compose.derive_servo_map(b)
    ft = {f.id: compose._try_load(f.type) for f in b.fingers}
    fake = Bridge.__new__(Bridge)
    fake._build = b
    fake._servo_map = sm
    fake._eff_rad = {(f.id, d.key): d.range               # (lo, hi) in RADIANS, like the real bridge
                     for f in b.fingers for _sid, d in zip(f.servos, ft[f.id].dofs)}
    fake._teleop_on = True
    fake._teleop_mode = "wilor"
    fake._teleop_last_t = 0.0
    fake._teleop_filt = {}
    fake._teleop_prev_frame = None
    fake._estopped = False
    batches = []
    fake._driver = types.SimpleNamespace(set_goal_positions=lambda g: batches.append(dict(g)))
    frames = []
    fake.teleopFrame = types.SimpleNamespace(emit=frames.append)
    stream_events = []                                    # teleopStreamChanged emissions
    fake._teleop_stream_live = False
    fake.teleopStreamChanged = types.SimpleNamespace(emit=lambda: stream_events.append(1))
    fake._batches = batches
    fake._frames = frames
    fake._stream_events = stream_events
    return fake, b, sm


def test_frame_stream_maps_norms_into_range():
    fake, b, sm = _wilor_bridge()
    norms = {f.id: {"flex": 1.0, "curl": 1.0, "spread": 0.0} for f in b.fingers}   # full close
    frame = fake._teleop_frame_stream(norms, dt=0.033)
    for f in b.fingers:
        assert f.socket in frame                          # complete finger emitted
        for sid, deg in zip(f.servos, frame[f.socket]):
            lo, hi = fake._eff_rad[(f.id, sm.by_id(sid).dof)]        # radians
            assert lo - 1e-6 <= math.radians(deg) <= hi + 1e-6      # deg (frame) stays in range


def test_thumb_runs_one_path_between_two_saved_anchors():
    """The thumb is not per-DOF teleop: it runs REST -> a target. The operator's own flex/curl say
    HOW FAR along (✌️ f1.00 vs 3️⃣ f0.15), and the pinch aperture picks WHICH end — folded-in
    (_TUCK, spread 90.9/flex 197.4) or opposed at the fingertips (_PINCH, 62.8/140.5). Its tracked
    SPREAD is ignored outright: ~9° of operator span drove a 160° robot range and threw it around."""
    from handlab.bridge_teleop import _PINCH_FALLBACK, _TUCK_FALLBACK, _THUMB_REST
    fake, b, sm = _wilor_bridge()                         # claw3f: index + thumb + mid
    tf = next(f for f in b.fingers if f.id == "thumb")
    tdofs = [sm.by_id(sid).dof for sid in tf.servos]
    pinch, tuck = _PINCH_FALLBACK["thumb"], _TUCK_FALLBACK["thumb"]

    def thumb(opp, **perdof):
        fake._teleop_filt = {}                            # no OneEuro carry-over between probes
        fr = fake._teleop_frame_stream({"thumb": dict(opp=opp, **perdof)}, dt=0.033)
        return dict(zip(tdofs, fr[tf.socket]))

    def clamped(dof, deg):
        lo, hi = fake._eff_rad[("thumb", dof)]            # radians
        return int(round(max(math.degrees(lo), min(math.degrees(hi), deg))))

    # the tracked spread changes nothing, at either end of the path
    for opp in (0.0, 1.0):
        assert (thumb(opp, flex=1.0, curl=0.0, spread=1.0)
                == thumb(opp, flex=1.0, curl=0.0, spread=-1.0))

    # thumb open -> rest, wherever the aperture sits
    assert thumb(0.0, flex=0.0, curl=0.0, spread=0.0) == {d: clamped(d, _THUMB_REST) for d in tdofs}

    # ✌️ (folded, aperture wide open) -> the TUCK anchor;  🤏 -> the PINCH anchor
    peace = thumb(0.0, flex=1.0, curl=0.72, spread=1.0)
    assert peace == {d: clamped(d, tuck[d]) for d in tdofs}
    assert thumb(1.0, flex=1.0, curl=0.79, spread=0.0) == {d: clamped(d, pinch[d]) for d in tdofs}
    assert peace["spread"] != clamped("spread", _THUMB_REST)     # ✌️ does NOT leave the thumb out

    # 3️⃣ (thumb barely folded) -> barely off rest, and clearly short of ✌️
    three = thumb(0.0, flex=0.15, curl=0.0, spread=0.0)
    for dof in tdofs:
        assert clamped(dof, _THUMB_REST) <= three[dof] < peace[dof]

    # never past whichever anchor is selected, for any operator input
    for opp, anchor in ((0.0, tuck), (1.0, pinch)):
        got = thumb(opp, flex=1.0, curl=1.0, spread=1.0)
        for dof in tdofs:
            assert got[dof] <= clamped(dof, anchor[dof])

    # no thumb in the stream -> no thumb command at all (the servos hold where they are)
    fake._teleop_filt = {}
    frame = fake._teleop_frame_stream({"index": {"flex": 0.5, "curl": 0.0, "spread": 0.0}}, dt=0.033)
    assert tf.socket not in frame


def test_late_pinch_magnet_converges_the_fingers():
    """opp=1: the finger SPREADS converge on the pinch pose (the geometry a human pinch can't
    express) and the closure floor lifts flex/curl that fell short of it."""
    from handlab.bridge_teleop import _PINCH_FALLBACK
    fake, b, sm = _wilor_bridge()
    norms1 = {"thumb": {"opp": 1.0, "flex": 0.0, "curl": 0.0, "spread": 0.0},
              "index": {"flex": 0.3, "curl": 0.3, "spread": 0.0},
              "mid":   {"flex": 0.3, "curl": 0.3, "spread": 0.0}}
    frame = fake._teleop_frame_stream(norms1, dt=0.033)
    for fid in ("index", "mid"):
        f = next(x for x in b.fingers if x.id == fid)
        got = dict(zip([sm.by_id(sid).dof for sid in f.servos], frame[f.socket]))
        assert abs(got["spread"] - round(_PINCH_FALLBACK[fid]["spread"])) <= 1   # spread converged
        # flex/curl: the operator's n=0.3 lands BELOW the pose -> the closure FLOOR lifts them to it
        assert abs(got["flex"] - round(_PINCH_FALLBACK[fid]["flex"])) <= 1
        assert abs(got["curl"] - round(_PINCH_FALLBACK[fid]["curl"])) <= 1


def test_pose_target_loads_the_newest_saved_anchor():
    """Both thumb anchors come from saved poses (they are HOME-RELATIVE -> re-capture in the app
    after a re-home, no code edit): newest "<prefix>*" wins, {socket:[deg]} -> {finger:{dof}}, and
    the two prefixes must not pick up each other's poses."""
    from handlab.bridge_teleop import (_PINCH_FALLBACK, _PINCH_POSE_PREFIX,
                                       _TUCK_FALLBACK, _TUCK_POSE_PREFIX)
    fake, b, sm = _wilor_bridge()
    fake._poses = []                                       # nothing saved -> the baked-in fallbacks
    assert fake._pose_target(_PINCH_POSE_PREFIX, _PINCH_FALLBACK) == _PINCH_FALLBACK
    assert fake._pose_target(_TUCK_POSE_PREFIX, _TUCK_FALLBACK) == _TUCK_FALLBACK
    idx = next(f for f in b.fingers if f.id == "index")
    dofs = [sm.by_id(sid).dof for sid in idx.servos]
    fake._poses = [{"name": "pinch_v4", "kind": "pose", "dofs": {idx.socket: [1.0, 2.0, 3.0]}},
                   {"name": "pinch_v6", "kind": "pose", "dofs": {idx.socket: [4.0, 5.0, 6.0]}},
                   {"name": "peace_v1", "kind": "pose", "dofs": {idx.socket: [7.0, 8.0, 9.0]}},
                   {"name": "wave", "kind": "clip", "dofs": {idx.socket: [0.0, 0.0, 0.0]}}]
    got = fake._pose_target(_PINCH_POSE_PREFIX, _PINCH_FALLBACK)
    assert got["index"] == dict(zip(dofs, [4.0, 5.0, 6.0]))        # v6 > v4, clip + peace ignored
    tuck = fake._pose_target(_TUCK_POSE_PREFIX, _TUCK_FALLBACK)
    assert tuck["index"] == dict(zip(dofs, [7.0, 8.0, 9.0]))       # the peace pose, not the pinch


def test_closure_floor_never_opens_against_operator():
    """The floor is a max(): an operator curling DEEPER than the pose keeps their deeper value; and
    below the floor ramp (opp < 0.7) flex/curl are purely the operator's."""
    from handlab.bridge_teleop import _PINCH_FALLBACK
    fake, b, sm = _wilor_bridge()
    idx = next(f for f in b.fingers if f.id == "index")
    dofs = [sm.by_id(sid).dof for sid in idx.servos]
    # deep curl (n=1.0) at opp=1 → stays at the operator's (deeper) value, not pulled DOWN to the pose
    frame = fake._teleop_frame_stream(
        {"thumb": {"opp": 1.0, "flex": 0, "curl": 0, "spread": 0},
         "index": {"flex": 1.0, "curl": 1.0, "spread": 0.0}}, dt=0.033)
    got = dict(zip(dofs, frame[idx.socket]))
    assert got["curl"] >= round(_PINCH_FALLBACK["index"]["curl"])
    assert got["flex"] >= round(_PINCH_FALLBACK["index"]["flex"])
    # mid-pinch (opp=0.5, below the floor ramp) → flex/curl are the operator's only (n=0.3 ≠ pose)
    fake._teleop_filt = {}
    frame2 = fake._teleop_frame_stream(
        {"thumb": {"opp": 0.5, "flex": 0, "curl": 0, "spread": 0},
         "index": {"flex": 0.3, "curl": 0.3, "spread": 0.0}}, dt=0.033)
    got2 = dict(zip(dofs, frame2[idx.socket]))
    assert abs(got2["flex"] - _PINCH_FALLBACK["index"]["flex"]) > 5
    assert abs(got2["curl"] - _PINCH_FALLBACK["index"]["curl"]) > 5


def test_step_wilor_drives_and_holds_on_stale():
    fake, b, sm = _wilor_bridge()
    norms = {f.id: {"flex": 0.6, "curl": 0.6, "spread": 0.0} for f in b.fingers}
    # fresh frame → drives (one GroupSyncWrite batch)
    fake._teleop_stream = types.SimpleNamespace(latest=lambda: (norms, 0.01))
    fake._teleop_last_t = None
    Bridge._teleop_step_wilor(fake)
    assert len(fake._batches) == 1 and fake._batches[0]              # goals written
    # stale frame → holds (no new write)
    fake._teleop_stream = types.SimpleNamespace(latest=lambda: (norms, 5.0))
    Bridge._teleop_step_wilor(fake)
    assert len(fake._batches) == 1                                  # unchanged
    # e-stopped → no write even with a fresh frame
    fake._estopped = True
    fake._teleop_stream = types.SimpleNamespace(latest=lambda: (norms, 0.01))
    fake._teleop_prev_frame = None
    Bridge._teleop_step_wilor(fake)
    assert len(fake._batches) == 1                                  # still no new write


def test_stream_live_flag_tracks_the_tracker_and_only_emits_on_change():
    """`teleopStreamLive` is what the demo recorder's readiness check reads: teleop can be ON with
    nothing arriving (tracker not started, wrong port), and recording that state yields episodes
    where the hand never moves. The signal must fire on TRANSITIONS only — this runs at 30 Hz."""
    fake, b, sm = _wilor_bridge()
    norms = {f.id: {"flex": 0.6, "curl": 0.6, "spread": 0.0} for f in b.fingers}
    assert fake._teleop_stream_live is False

    fake._teleop_stream = types.SimpleNamespace(latest=lambda: (norms, 0.01))   # tracker delivering
    Bridge._teleop_step_wilor(fake)
    assert fake._teleop_stream_live is True
    assert len(fake._stream_events) == 1
    Bridge._teleop_step_wilor(fake)                       # still live -> no further churn
    Bridge._teleop_step_wilor(fake)
    assert len(fake._stream_events) == 1

    fake._teleop_stream = types.SimpleNamespace(latest=lambda: (norms, 5.0))    # tracker went away
    Bridge._teleop_step_wilor(fake)
    assert fake._teleop_stream_live is False
    assert len(fake._stream_events) == 2
    Bridge._teleop_step_wilor(fake)
    assert len(fake._stream_events) == 2

    # nothing ever received reads as not-live too (latest() -> (None, huge))
    fake._teleop_stream = types.SimpleNamespace(latest=lambda: (None, 1e9))
    Bridge._teleop_step_wilor(fake)
    assert fake._teleop_stream_live is False and len(fake._stream_events) == 2
