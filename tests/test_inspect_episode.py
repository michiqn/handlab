"""The episode inspector (tools/inspect_episode.py) — the gate between "recorded" and "usable".

Written against episodes produced by the REAL DemoRecorder, so the schema stays honest: the point of
the tool is to catch a quietly poisoned dataset (lagging observations, a servo off the bus, a servo
that never moved, an object out of frame) BEFORE an afternoon of recording, so each of those has to
actually trip.
"""
import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest

from handlab.recorder import DemoRecorder

_spec = importlib.util.spec_from_file_location(
    "inspect_episode", Path(__file__).resolve().parents[1] / "tools" / "inspect_episode.py")
insp = importlib.util.module_from_spec(_spec)
sys.modules["inspect_episode"] = insp
_spec.loader.exec_module(insp)

N = 12                                              # claw4f — the inspector is build-agnostic


def _frame():
    return np.zeros((36, 64, 3), np.uint8)


def _det(i):
    return {"id": i, "pose": {"tvec": [0.1, 0.0, 0.3], "rvec": [0.0, 0.0, 0.0],
                              "in_base": {"pos": [0.1, 0.0, 0.05], "rvec": [0.0, 0.0, 0.0]}}}


def _record(tmp_path, T=40, stale_every=0, offline_col=None, stuck_col=None, object_every=1):
    """One synthetic episode with optional defects, written through the real recorder."""
    rec = DemoRecorder(tmp_path)
    rec.start("claw4f", "grasp", {"build": "claw4f", "task": "grasp", "control_rate_hz": 10},
              object_ids=[10, 11], capture_wh=None)
    cam_no = 100
    for k in range(T):
        # a closing motion: every servo ramps, except one deliberately stuck column
        goal = [2048 + int(k * 12) for _ in range(N)]
        if stuck_col is not None:
            goal[stuck_col] = 2048
        online = [1] * N
        if offline_col is not None and k % 2 == 0:
            online[offline_col] = 0
        if not (stale_every and k % stale_every):    # else: reuse the previous camera frame id
            cam_no += 1
        dets = [_det(10)] if (object_every and k % object_every == 0) else []
        rec.on_step(_frame(), float(k) * 0.1, goal, [g - 5 for g in goal], [12] * N,
                    [0.0] * N, online, dets, cam_no)
    return rec.stop(outcome="success")


def test_clean_episode_reports_no_problems(tmp_path):
    pytest.importorskip("cv2")
    r = insp.analyze(_record(tmp_path))
    assert r["problems"] == []
    assert r["T"] == 40 and r["frames_on_disk"] == 40 and r["n_servos"] == N
    assert r["hz"] == pytest.approx(10.0, abs=0.2)         # the 10 Hz telemetry beat
    assert r["duration_s"] == pytest.approx(3.9, abs=0.1)
    assert r["stale_frac"] == 0.0 and r["offline_frac"] == 0.0
    assert r["object_seen_frac"] == 1.0
    assert r["track_err_median"] == pytest.approx(5.0)     # goal - present, by construction


def test_held_camera_frames_are_flagged(tmp_path):
    """A repeated cam_frame_no = the action moved while the observation stood still. That is what
    teaches a policy to lag, and it is invisible while recording."""
    pytest.importorskip("cv2")
    r = insp.analyze(_record(tmp_path, stale_every=2))
    assert r["stale_frac"] > insp.STALE_WARN
    assert any("held camera frames" in p for p in r["problems"])


def test_offline_servo_and_stuck_servo_are_flagged(tmp_path):
    pytest.importorskip("cv2")
    r = insp.analyze(_record(tmp_path, offline_col=4, stuck_col=7))
    assert any("offline" in p and "column 4" in p for p in r["problems"])
    assert r["stuck_servos"] == [7]
    assert any("never moved" in p for p in r["problems"])
    assert r["goal_span_ticks"][7] == 0 and r["goal_span_ticks"][0] > 0


def test_unseen_object_and_short_episode_are_flagged(tmp_path):
    pytest.importorskip("cv2")
    r = insp.analyze(_record(tmp_path, T=8, object_every=0))
    assert r["object_seen_frac"] == 0.0
    assert any("object marker" in p for p in r["problems"])
    assert any("mis-click" in p for p in r["problems"])     # 0.7s


def test_contact_sheet_renders(tmp_path):
    pytest.importorskip("cv2")
    import cv2
    ep = _record(tmp_path)
    out = insp.contact_sheet(ep, ep / "sheet.png")
    assert out is not None and out.exists()
    img = cv2.imread(str(out))
    assert img is not None and img.shape[0] > 220        # frame grid + the action-trace strip
