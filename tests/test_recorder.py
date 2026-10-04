"""Demo Recorder — headless tests (no camera, no .app): synthetic frame + fake telemetry/goals."""
import math

import numpy as np
import pytest

from handlab.recorder import DemoRecorder, aruco_from_dets, _rvec_to_quat


def _frame():
    return np.zeros((48, 64, 3), np.uint8)


def _det(mid, tvec=(0.01, 0.02, 0.3), rvec=(0.0, 0.0, 0.0), in_base=None):
    p = {"tvec": list(tvec), "rvec": list(rvec)}
    if in_base is not None:
        p["in_base"] = in_base
    return {"id": mid, "pose": p}


def test_aruco_from_dets_selection_and_nan():
    # nothing seen -> -1 + all-NaN
    oid, base7, cam7 = aruco_from_dets([], [10, 11])
    assert oid == -1 and all(math.isnan(x) for x in base7) and all(math.isnan(x) for x in cam7)
    # calibrated cam pose but no in_base -> cam real, base NaN
    oid, base7, cam7 = aruco_from_dets([_det(12)], [10, 11, 12])
    assert oid == 12 and not any(math.isnan(x) for x in cam7) and all(math.isnan(x) for x in base7)
    # in_base present -> base real (pos passes through)
    oid, base7, _ = aruco_from_dets([_det(12, in_base={"pos": [0.1, 0.2, 0.3], "rvec": [0, 0, 0]})], [12])
    assert base7[:3] == [0.1, 0.2, 0.3]
    # picks the FIRST id in the object list that is present
    oid, _, _ = aruco_from_dets([_det(12), _det(10)], [10, 12])
    assert oid == 10
    # a marker without a pose (uncalibrated) is skipped
    assert aruco_from_dets([{"id": 10, "center": [1, 2]}], [10])[0] == -1


def test_rvec_to_quat_identity():
    assert _rvec_to_quat([0, 0, 0]) == [0, 0, 0, 1]


def test_recorder_episode_roundtrip(tmp_path):
    pytest.importorskip("cv2")
    rec = DemoRecorder(tmp_path)
    meta = {"build": "claw3f", "task": "grasp v1", "control_rate_hz": 10,
            "motor_order": [{"id": i} for i in range(1, 10)]}      # (caller-built, like the bridge)
    assert rec.episode_count == 0
    rec.start("claw3f", "grasp v1", meta, object_ids=[10, 11, 12, 13, 14, 15])
    T = 5
    for k in range(T):
        goal = list(range(2048, 2048 + 9))           # distinct per servo, 2048 = zero
        online = [1] * 9 if k != 3 else [1] * 8 + [0]  # servo 9 "dropped" on step 3
        dets = [_det(11, in_base={"pos": [0.1, 0.2, 0.3], "rvec": [0, 0, 0]})] if k % 2 == 0 else []
        rec.on_step(_frame(), float(k) * 0.1, goal, [2048] * 9, [10] * 9, [0.0] * 9, online, dets, 100 + k)
    path = rec.stop(outcome="success", notes="n")
    assert path is not None

    assert len(sorted((path / "frames").glob("*.png"))) == T
    d = np.load(path / "episode.npz")
    for key in ("t", "frame_idx", "action_goal", "present_pos", "current", "dof_deg",
                "aruco_pose", "aruco_pose_cam", "object_id", "online", "cam_frame_no"):
        assert key in d and len(d[key]) == T
    assert d["action_goal"].shape == (T, 9) and d["action_goal"].dtype == np.int64
    assert list(d["action_goal"][0]) == list(range(2048, 2048 + 9))   # motor order + zero round-trip
    assert not np.isnan(d["aruco_pose"][0]).any()                     # marker seen on even steps
    assert np.isnan(d["aruco_pose"][1]).all()                        # absent on odd steps
    assert d["object_id"][0] == 11 and d["object_id"][1] == -1
    assert d["online"].shape == (T, 9) and d["online"][3, 8] == 0 and d["online"][0, 0] == 1
    assert list(d["cam_frame_no"]) == [100, 101, 102, 103, 104]      # staleness flag round-trips

    import yaml
    m = yaml.safe_load((path / "meta.yaml").read_text())
    for key in ("build", "task", "outcome", "episode_len_T", "motor_order", "control_rate_hz"):
        assert key in m
    assert m["outcome"] == "success" and m["episode_len_T"] == T and m["build"] == "claw3f"

    # a second episode increments the index; discard_last removes ONLY it
    rec.start("claw3f", "grasp v1", meta)
    rec.on_step(_frame(), 0.0, [2048] * 9, [2048] * 9, [0] * 9, [0.0] * 9, [1] * 9, [], 1)
    p2 = rec.stop("fail")
    assert p2 is not None and p2 != path
    assert rec.discard_last() is True and not p2.exists() and path.exists()


def test_discard_prefers_active_over_last_saved(tmp_path):
    """Review Finding 2: discard while a fresh take is running must abort the ACTIVE episode, not
    nuke the previously-saved one."""
    pytest.importorskip("cv2")
    rec = DemoRecorder(tmp_path)
    rec.start("b", "t", {})
    rec.on_step(_frame(), 0.0, [2048] * 9, [2048] * 9, [0] * 9, [0.0] * 9, [1] * 9, [], 1)
    a = rec.stop("success")                        # episode A saved
    rec.start("b", "t", {})
    rec.on_step(_frame(), 0.0, [2048] * 9, [2048] * 9, [0] * 9, [0.0] * 9, [1] * 9, [], 1)
    b_dir = rec._dir                               # episode B in progress
    assert rec.discard_last() is True              # aborts the ACTIVE take (B)…
    assert not b_dir.exists() and a.exists()       # …and keeps the previously-saved A
    assert rec.active is False


def test_empty_episode_is_discarded(tmp_path):
    rec = DemoRecorder(tmp_path)
    p = rec.start("b", "t", {})
    assert rec.stop("success") is None      # no frames pushed -> dropped, no npz/meta
    assert not p.exists()


def test_dynamixel_goals_caches_intended_before_guard():
    """action label = commanded goal even when offline/torque-off (cached before the early-return)."""
    from handlab import compose, library_io
    from handlab.hal.dynamixel_driver import DynamixelDriver
    sm = compose.derive_servo_map(library_io.load_build("claw3f"))
    d = DynamixelDriver(sm)
    d._write4 = lambda *a, **k: None
    d._online = set()                        # offline -> the wire write is skipped...
    d.set_goal_position(4, 2500)
    assert d.goals()[4] == 2500              # ...but the INTENDED goal is still recorded
