"""Bridge mixin: the Demo Recorder (visuomotor-IL episodes).

@Slot methods + plain helpers ONLY — @Property/@Signal live in bridge.py (PySide6 does not wire
Property(notify=...) across mixin boundaries). **observe-only:** gathers existing telemetry + camera
frames on the 10 Hz telemetry beat and pushes them to `recorder.DemoRecorder`; it NEVER issues a
motion command and never touches the control/safety path.
"""
from __future__ import annotations

import json
import os
import subprocess
import time
from pathlib import Path

from PySide6.QtCore import Slot

from . import library_io
from .hal.base import ticks_to_deg, CENTER

_OBJECT_MARKER_IDS = [10, 11, 12, 13, 14, 15]   # hex-prism faces (vision.yaml register)


def _capture_wh():
    """Logged-frame size (w,h). Downscaled by default to keep the dataset small (10 Hz × 1080p PNG
    ≈ 1-2 GB/min); override with HANDLAB_DEMO_WH='WxH', or 'native' for full resolution."""
    v = os.environ.get("HANDLAB_DEMO_WH", "640x360").lower().strip()
    if v in ("native", "0", "full"):
        return None
    try:
        w, h = (int(x) for x in v.split("x"))
        if w > 0 and h > 0:
            return (w, h)
    except Exception:
        pass
    return None                      # invalid/≤0 → native (never a zero-dim resize crash)


def _git_commit() -> str:
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "--short", "HEAD"],
            cwd=str(Path(library_io.__file__).parent.parent),
            stderr=subprocess.DEVNULL, timeout=2).decode().strip()
    except Exception:
        return "unknown"


class BridgeRecordMixin:

    def _demo_meta(self, task: str) -> dict:
        """Self-describing episode metadata (schema in the plan). Everything a policy needs to
        interpret the arrays unambiguously + reproduce the setup."""
        sm = self._servo_map
        cal = library_io.load_calibration()
        hw = self._build.hardware
        cam = self.camera
        w = h = None
        try:
            img = cam.current() if cam else None
            if img is not None:
                w, h = int(img.width()), int(img.height())
        except Exception:
            pass
        return {
            "build": self._pose_scope,
            "task": task,
            "driver": getattr(self._driver, "driver_type", "") if self._driver else "",
            "control_rate_hz": 10,
            "control_rate_note": "sampled on the telemetry beat (10 Hz); DEPLOYMENT control rate is "
                                 "bounded by this — raising it means raising the telemetry beat.",
            "motor_order": [{"id": s.id, "finger": s.finger, "dof": s.dof} for s in sm.servos],
            "motor_order_note": "action_goal/present_pos/current/dof_deg 9-vectors are in this order "
                                "(motor ids 1..9 = index, thumb, mid; each spread, flex, curl).",
            "action_note": "action_goal = logical goal ticks (2048=zero), the exact units "
                           "set_goal_position consumes → replays with no decoder. DEPLOY: re-enter "
                           "the effective-range clamp before set_goal_position (in-distribution a "
                           "no-op; keeps out-of-distribution predictions inside the safety envelope).",
            "calibration": {int(k): v for k, v in cal.items()},
            "hardware": {"goal_current": hw.goal_current, "current_limit": hw.current_limit,
                         "hold_current": hw.hold_current, "pos_p": hw.pos_p, "baud": hw.baud},
            "camera": {"device": getattr(cam, "device_id", None) if cam else None,
                       "width": w, "height": h, "capture_wh": _capture_wh(),
                       "calibrated": bool(getattr(cam, "calibrated", False)) if cam else False,
                       "intrinsics_ref": str(Path.home() / ".handlab" / "intrinsics.yaml")},
            "object_marker_ids": list(_OBJECT_MARKER_IDS),
            "handlab_git": _git_commit(),
            "start_wallclock": time.strftime("%Y-%m-%dT%H:%M:%S"),
        }

    def _demo_step(self, states) -> None:
        """One timestep on the telemetry beat: latest raw cam frame + ArUco + the 9 servo columns
        (commanded goal / present / current / degrees) → recorder. Skips when no camera frame.
        Reads only; issues nothing."""
        cam = self.camera
        if cam is None or not self._demo.active:
            return
        try:
            from . import vision
            img = cam.current()
            frame = vision.qimage_to_bgr(img) if img is not None else None
        except Exception:
            frame = None
        if frame is None:
            return
        sm = self._servo_map
        goals = self._driver.goals() if self._driver else {}

        def col(fn, default):
            return [fn(states[s.id]) if states.get(s.id) is not None else default for s in sm.servos]

        online = getattr(self, "_online", set())
        self._demo.on_step(
            frame, time.monotonic(),
            [int(goals.get(s.id, CENTER)) for s in sm.servos],
            col(lambda st: int(st.present_position), 0),
            col(lambda st: int(st.present_current), 0),
            col(lambda st: ticks_to_deg(st.present_position, 2), 0.0),
            [1 if s.id in online else 0 for s in sm.servos],   # mask: 0 = dropped this beat
            cam.latest_detections(), cam.frame_no,
        )

    @Slot(str, result=str)
    def startDemo(self, task: str) -> str:
        if self._demo.active:
            return json.dumps({"ok": False, "error": "already recording"})
        task = (task or "task").strip() or "task"
        try:
            path = self._demo.start(self._pose_scope, task, self._demo_meta(task),
                                    object_ids=_OBJECT_MARKER_IDS, capture_wh=_capture_wh())
        except Exception as e:
            return json.dumps({"ok": False, "error": str(e)})
        self.demoRecordingChanged.emit()
        return json.dumps({"ok": True, "path": str(path)})

    @Slot(str, result=str)
    def stopDemo(self, outcome: str) -> str:
        """Finalize the episode with an outcome label (success|fail). Empty episodes are dropped."""
        outcome = outcome if outcome in ("success", "fail") else "success"
        try:
            path = self._demo.stop(outcome)
        except Exception as e:
            self.demoRecordingChanged.emit()
            return json.dumps({"ok": False, "error": str(e)})
        self.demoRecordingChanged.emit()
        return json.dumps({"ok": True, "outcome": outcome,
                           "path": (str(path) if path else None),
                           "episodes": self._demo.episode_count})

    @Slot(result=str)
    def discardLastDemo(self) -> str:
        ok = self._demo.discard_last()
        self.demoRecordingChanged.emit()
        return json.dumps({"ok": bool(ok), "episodes": self._demo.episode_count})
