"""Bridge mixin: poses, recording, clip playback, phased safe-home.

Slot/helper mixin for the Bridge (moved verbatim out of bridge.py). ONLY @Slot methods and
plain helpers live here — @Property/@Signal stay in bridge.py: PySide6 does not wire
Property(notify=...) across mixin boundaries (verified empirically), and signals must be
declared on the QObject subclass. Methods here use self.<signal>/self.<state> at runtime.
"""
from __future__ import annotations

import json

from PySide6.QtCore import Slot

from . import library_io
from .hal.base import ticks_to_deg


class BridgePosesMixin:

    def _grouped_dofs(self, states) -> dict:
        """Measured joint DEGREES grouped by socket, in dof order (matches the QML mapping)."""
        out = {}
        for f in self._build.fingers:
            out[f.socket] = [ticks_to_deg(states[sid].present_position)
                             for sid in f.servos if sid in states]
        return out

    def _save_poses(self) -> None:
        library_io.save_poses(self._pose_scope, self._poses)
        self.posesChanged.emit()

    def _find_pose(self, name: str):
        return next((p for p in self._poses if p.get("name") == name), None)

    @Slot(str, result=str)
    def capturePose(self, name: str) -> str:
        name = (name or "").strip() or f"Pose {len(self._poses) + 1}"
        if self._driver is None:
            return json.dumps({"ok": False, "error": "no driver"})
        dofs = self._grouped_dofs(self._driver.read_states())
        self._poses = [p for p in self._poses if p.get("name") != name]   # overwrite by name
        self._poses.append({"name": name, "kind": "pose", "dofs": dofs})
        self._save_poses()
        return json.dumps({"ok": True, "name": name})

    @Slot(str)
    def applyPose(self, name: str) -> None:
        p = self._find_pose(name)
        if p and p.get("kind", "pose") == "pose":
            self.playbackFrame.emit(json.dumps(p.get("dofs", {})))

    @Slot(str)
    def deletePose(self, name: str) -> None:
        self._poses = [p for p in self._poses if p.get("name") != name]
        self._save_poses()

    @Slot(str, str)
    def renamePose(self, old: str, new: str) -> None:
        new = (new or "").strip()
        p = self._find_pose(old)
        if p and new:
            p["name"] = new
            self._save_poses()

    @Slot(str)
    def reorderPoses(self, names_json: str) -> None:
        """Reorder the saved poses/clips to match the given list of names (drag-and-drop in the UI).
        Robust: reorders by name, drops unknown names, and appends any not mentioned at the end (so a
        stale UI order can never lose an entry). No-op unless the result is a permutation of the set."""
        try:
            order = json.loads(names_json)
        except Exception:
            return
        by_name = {p.get("name"): p for p in self._poses}
        new = [by_name[n] for n in order if n in by_name]
        seen = {n for n in order if n in by_name}
        new += [p for p in self._poses if p.get("name") not in seen]   # keep any not listed
        if len(new) == len(self._poses) and new != self._poses:
            self._poses = new
            self._save_poses()

    @Slot()
    def startRecording(self) -> None:
        import time
        self._rec_buf = []
        self._rec_t0 = time.monotonic()
        self._recording = True
        self.posesChanged.emit()

    @staticmethod
    def _frame_delta(a: dict, b: dict) -> float:
        return max((abs(x - y) for s in a for x, y in zip(a[s], b.get(s, a[s]))), default=0.0)

    def _trim_idle(self, frames: list[dict], eps: float = 0.6) -> list[dict]:
        """Keep only the active window: from just before the hand starts MOVING to just
        after it stops. Motion-based (frame-to-frame), so a jittery/drifting hold in the
        idle sections is still recognised as idle (endpoint-similarity would miss it)."""
        if len(frames) < 3:
            return frames
        moves = [self._frame_delta(frames[i], frames[i - 1]) for i in range(1, len(frames))]
        moving = [i for i, m in enumerate(moves, start=1) if m > eps]
        if not moving:
            return frames[:1]
        start = max(0, moving[0] - 1)
        end = min(len(frames) - 1, moving[-1] + 1)
        return frames[start:end + 1]

    @Slot(str, result=str)
    def stopRecording(self, name: str) -> str:
        self._recording = False
        frames = self._trim_idle([d for _, d in self._rec_buf])
        if len(frames) < 2:
            self.posesChanged.emit()
            return json.dumps({"ok": False, "error": "nothing recorded"})
        ts = [t for t, _ in self._rec_buf]
        dt = round((ts[-1] - ts[0]) / max(1, len(ts) - 1), 3) or 0.1
        name = (name or "").strip() or f"Clip {len(self._poses) + 1}"
        self._poses = [p for p in self._poses if p.get("name") != name]
        self._poses.append({"name": name, "kind": "clip", "dt": dt, "frames": frames})
        self._save_poses()
        return json.dumps({"ok": True, "name": name, "frames": len(frames)})

    @Slot(str, bool)
    def playClip(self, name: str, loop: bool = False) -> None:
        p = self._find_pose(name)
        if not p or p.get("kind") != "clip" or not p.get("frames"):
            return
        self.stopHoming()                       # mutually exclusive with the home sequencer
        self._play_frames = p["frames"]
        self._play_dt = float(p.get("dt", 0.1)) or 0.1
        self._play_loop = bool(loop)
        self._play_i = 0
        self._play_step()

    def _play_step(self) -> None:
        if self._play_i >= len(self._play_frames):
            if self._play_loop and self._play_frames:
                self._play_i = 0            # loop: start over
            else:
                self.posesChanged.emit()    # isPlaying -> false
                return
        self.playbackFrame.emit(json.dumps(self._play_frames[self._play_i]))
        self._play_i += 1
        if self._play_i == 1:
            self.posesChanged.emit()        # isPlaying -> true
        self._play_timer.start(int(self._play_dt * 1000))

    @Slot()
    def stopPlayback(self) -> None:
        self._play_timer.stop()
        self.posesChanged.emit()

    @Slot(int)
    def safeHome(self, finger_ord: int = -1) -> None:
        """Collision-aware return home (finger_ord = -1: whole hand). Homes in PHASES —
        curl → flex → spread — waiting for the involved servos to settle between phases.
        Frames go through playbackFrame → QML applyPoseFrame (e-stop guarded, sliders animate)."""
        if self._driver is None:
            return
        self.stopPlayback()                     # mutually exclusive with clip playback
        if finger_ord < 0:
            self._home_scope = list(self._build.fingers)
        elif finger_ord < len(self._build.fingers):
            self._home_scope = [self._build.fingers[finger_ord]]
        else:
            return
        self._home_groups = ["curl", "flex", "spread"]
        self._home_done = set()
        self._home_next_phase()

    def _home_frame(self, states, target_groups: set) -> dict:
        """Full frame for the scoped fingers: target groups → home (0°), the rest HOLD their
        measured angle. Only scoped sockets appear, so other fingers are untouched."""
        frame = {}
        for f in self._home_scope:
            ft = self._ftypes.get(f.id)
            if ft is None:
                continue
            vals = []
            for sid, d in zip(f.servos, ft.dofs):
                if d.key in target_groups or sid not in states:
                    vals.append(0.0)
                else:
                    vals.append(ticks_to_deg(states[sid].present_position))
            frame[f.socket] = vals
        return frame

    def _home_next_phase(self) -> None:
        import time
        if not self._home_groups:
            self.posesChanged.emit()            # homing done -> refresh pose-state bindings
            return
        group = self._home_groups.pop(0)
        self._home_done.add(group)              # earlier groups stay pinned at home
        try:
            states = self._driver.read_states()
        except Exception:
            states = {}
        self.playbackFrame.emit(json.dumps(self._home_frame(states, self._home_done)))
        self._home_involved = set()
        for f in self._home_scope:
            ft = self._ftypes.get(f.id)
            if ft is not None:
                self._home_involved.update(sid for sid, d in zip(f.servos, ft.dofs) if d.key == group)
        self._home_t0 = time.monotonic()
        self.posesChanged.emit()                # phase started -> refresh pose-state bindings
        self._home_timer.start(150)

    def _home_step(self) -> None:
        import time
        elapsed = time.monotonic() - self._home_t0
        settled = elapsed >= 3.0                # phase timeout — never hang the sequence
        if not settled and elapsed >= 0.6:      # give the goal time to actually start moving
            try:
                states = self._driver.read_states()
                settled = all(not states[sid].moving
                              for sid in self._home_involved if sid in states)
            except Exception:
                settled = False
        if settled:
            self._home_next_phase()
        else:
            self._home_timer.start(150)

    @Slot()
    def stopHoming(self) -> None:
        self._home_timer.stop()
        self._home_groups = []
        self.posesChanged.emit()
