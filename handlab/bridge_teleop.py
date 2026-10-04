"""Bridge mixin: WiLoR hand-tracking teleop (your hand -> the robot claw).

@Slot methods + plain helpers ONLY -- @Property/@Signal live in bridge.py (PySide6 does not wire
Property(notify=...) across mixin boundaries). Motion is applied DIRECTLY in Python (perf
fix): each step clamps degrees to the SAME effective range jogNorm uses (`_eff_rad`) ->
`rad_to_ticks` -> `driver.set_goal_positions` (ONE GroupSyncWrite batch; the driver's torque/online
gate skips limp servos, velocity/accel/force caps unchanged). `teleopFrame` is emitted ONLY for the
QML display mirror (sliders/twin), and only when the frame changed.

The human side lives in an EXTERNAL process (tracking/wilor_teleop.py): WiLoR reconstructs the hand,
normalizes each DOF against the operator's own range, and streams per-finger norms over UDP. handlab
owns the ROBOT side -- it maps those norms to degrees through the existing safe seam and applies all
the safety (E-STOP via `_estopped`, torque + online gate, velocity/force caps). Smoothing is the
OneEuro filter here (the tracker adds none -> no double-smoothing).

Safety layers: default OFF (conscious enable); `_estopped` gates the write AND the loop is stopped
in every E-STOP path; smoothing here + the hardware velocity cap doubly damp jitter. Teleop needs no
ARM (that gates the *agent*, not you).
"""
from __future__ import annotations

import json
import math
import re
import time

from PySide6.QtCore import Slot

from ._filters import OneEuro
from .hal.base import rad_to_ticks
from .perf import PERF
from .teleop_stream import DEFAULT_PORT as _STREAM_PORT, TeleopStreamReceiver

_TELEOP_MS = 33                                     # ~30 Hz motion loop
_RENDER_MS = 33                                     # twin render interval, normal (~30 fps)
_RENDER_TELEOP_MS = 100                             # twin render while teleop runs (~10 fps -- the GUI
#                                                     thread must serve the twin + serial; user decision)
_STREAM_STALE_S = 0.5                               # skip driving on stream frames older than this

# ── Pinch synergy ────────────────────────────────────────────────────────────────────────────────
# Target = the newest SAVED pose named "pinch*" (capture it in the app; teleop picks it up on the
# next enable). Poses are HOME-RELATIVE, so a re-home invalidates the numbers — loading the saved
# pose makes re-capturing in the app the WHOLE fix, no code edit (this bit us twice). The constants
# below are only the fallback for a build with no saved pinch pose.
_PINCH_POSE_PREFIX = "pinch"
_PINCH_FALLBACK = {          # = saved pose "pinch_v6": thumb tip TOUCHING all three fingers
    "thumb": {"spread": 62.8, "flex": 140.5, "curl": 123.0},
    "index": {"spread": 84.8, "flex": 115.8, "curl": 76.9},
    "mid":   {"spread": -1.0, "flex": 109.9, "curl": 55.0},
    "ring":  {"spread": -75.1, "flex": 112.9, "curl": 43.0},
}
# The thumb's SECOND anchor, from a saved "peace*" pose: folded in against the palm (✌️/✊), the far
# end of its travel. Only its thumb entry is read — the fingers of a ✌️ pose are per-DOF teleop.
_TUCK_POSE_PREFIX = "peace"
_TUCK_FALLBACK = {"thumb": {"spread": 90.9, "flex": 197.4, "curl": 120.9}}   # = "peace_v1"

# THUMB, split by what the tracker can actually measure:
#   SPREAD is untrackable — opposition and abduction are both in-plane, so the decomposition cannot
#     separate them, and the operator's span is ~9° (open vs splayed capture) against a 160° robot
#     range: the norm saturated at ±1 and threw the thumb around on any gesture that moved it.
#     It therefore rides the pinch aperture 'opp' alone (thumb-tip↔index-tip distance, the one
#     cleanly separated thumb channel: 0.95 open vs 0.14 touching), REST -> the pinch pose.
#   FLEX + CURL are honest (29.8° / 58.6° operator spans, curl is a real IP joint angle) and they
#     carry HOW FAR the thumb folds in: ✌️ reads f1.00 c0.66 where 3️⃣ reads f0.15 c0.00 — 'opp'
#     cannot tell those apart (both sit far from the index tip).
# So the thumb runs one 1-D path with two ends, both read from saved poses that were verified on the
# real hand: REST (0/0/0, stretched out) -> a target. The operator's flex/curl say HOW FAR along, and
# 'opp' picks WHICH target — folded-in (_TUCK) or opposed at the fingertips (_PINCH). The measured
# poses show why a single anchor could not work: ✌️ needs spread 90.9 / flex 197.4, the pinch only
# 62.8 / 140.5, and REST would have left the thumb sticking out on ✌️ altogether.
_THUMB_REST = 0.0            # home = stretched/open, all three thumb dofs
_THUMB_START = 0.10          # opp below this -> thumb rests
_THUMB_FULL = 0.85           # opp above this -> thumb fully on the pinch pose
# The FINGERS stay per-DOF teleop (that is what makes it feel like teleop). The pose only pulls the
# convergence geometry a human pinch cannot express — the SPREADS (index +85°, ring −75°!) — and only
# late in the aperture: assist weight ramps 0→1 (smoothstep) between these opp values.
_PINCH_ASSIST_START = 0.5
_PINCH_ASSIST_FULL = 0.9
# Closure floor (fingers' flex/curl): later still, guarantee the fingers are AT LEAST as closed as the
# touching pose — max(operator, pose), never opening against the operator. Per-DOF closure lands only
# "roughly" on the pose, so the tips used to stop short / pass by (live feedback).
_PINCH_CLOSE_START = 0.7
_PINCH_CLOSE_FULL = 0.95


def _ramp(opp: float, start: float, full: float) -> float:
    """Smoothstep 0→1 over opp ∈ [start, full]."""
    t = (opp - start) / (full - start)
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)


def _thumb_w(opp: float) -> float:
    """Thumb synergy weight: REST → the pinch pose over the whole aperture."""
    return _ramp(opp, _THUMB_START, _THUMB_FULL)


def _assist_w(opp: float) -> float:
    """Spread-convergence assist weight (fingers), late in the pinch aperture."""
    return _ramp(opp, _PINCH_ASSIST_START, _PINCH_ASSIST_FULL)


def _close_w(opp: float) -> float:
    """Closure-floor weight (fingers' flex/curl), the last stretch of the pinch."""
    return _ramp(opp, _PINCH_CLOSE_START, _PINCH_CLOSE_FULL)


class BridgeTeleopMixin:

    # ---- motion loop (self-rescheduling QTimer, mirrors _play_step) ----
    def _teleop_step(self) -> None:
        if not self._teleop_on:
            return
        with PERF.lap("teleop.step"):
            self._teleop_step_wilor()
        if self._teleop_on:
            self._teleop_timer.start(_TELEOP_MS)

    def _teleop_apply(self, frame: dict) -> None:
        """Shared tail: clamp + batch-write + mirror to QML. DIRECT apply in Python (perf SS37) --
        the `_eff_rad` clamp (in `_teleop_goals`), the driver torque/online gate, and the `_estopped`
        gate; QML only MIRRORS."""
        if not frame:
            return
        goals = self._teleop_goals(frame)
        if goals and not self._estopped and self._driver is not None:
            try:
                with PERF.lap("serial.write"):
                    self._driver.set_goal_positions(goals)
            except Exception:
                pass                                # a bus hiccup must never kill the teleop loop
        if frame != self._teleop_prev_frame:        # a still hand -> zero QML churn
            self._teleop_prev_frame = frame
            self.teleopFrame.emit(json.dumps(frame))

    def _teleop_step_wilor(self) -> None:
        """Consume the latest UDP frame from the external WiLoR tracker (per-finger norms) and drive
        it. No hand camera -- WiLoR runs in its own process (see `tracking/`). Holds (drives nothing)
        on a stale/absent stream."""
        recv = self._teleop_stream
        if recv is None:
            return
        norms, age = recv.latest()
        now = time.monotonic()
        dt = (now - self._teleop_last_t) if self._teleop_last_t else _TELEOP_MS / 1000.0
        self._teleop_last_t = now
        live = norms is not None and age <= _STREAM_STALE_S
        self._set_stream_live(live)
        if not live:                                 # no fresh frame -> don't drive on stale data
            return
        self._teleop_apply(self._teleop_frame_stream(norms, dt))

    def _set_stream_live(self, live: bool) -> None:
        """Track whether the tracker is actually delivering, for the demo recorder's readiness check.
        Emits ONLY on a transition -- this runs at 30 Hz, and a per-tick signal would churn QML."""
        if live != self._teleop_stream_live:
            self._teleop_stream_live = live
            self.teleopStreamChanged.emit()

    def _pose_target(self, prefix: str, fallback: dict) -> dict:
        """The newest saved "<prefix>*" pose -> {finger_id: {dof: deg}} (the poses store is
        {socket: [deg per servo]}). Picks the highest trailing number ("pinch_v6" > "pinch_v4"), so
        re-capturing after a re-home is the only step needed. Falls back to `fallback`."""
        best, best_v = None, -2
        for item in getattr(self, "_poses", None) or []:
            name = str(item.get("name", ""))
            if item.get("kind", "pose") != "pose" or not name.startswith(prefix):
                continue
            m = re.search(r"(\d+)$", name)
            v = int(m.group(1)) if m else -1
            if v >= best_v:                          # ties -> the later entry (= last captured)
                best, best_v = item, v
        dofs = (best or {}).get("dofs") or {}
        out = {}
        for f in self._build.fingers:
            vals = dofs.get(f.socket)
            if not vals or len(vals) != len(f.servos):
                continue
            got = {}
            for sid, deg in zip(f.servos, vals):
                s = self._servo_map.by_id(sid)
                if s is not None:
                    got[s.dof] = float(deg)
            out[f.id] = got
        return out or dict(fallback)

    def _teleop_frame_stream(self, norms: dict, dt: float) -> dict:
        """External per-finger norms ({finger:{dof:norm}}) -> degrees. The tracker already did the
        human->robot normalization, so this reuses the norm->deg half only: OneEuro `_teleop_smooth`
        -> `_teleop_dof_deg`, keeping the complete-finger positional invariant."""
        # Pinch aperture (thumb 'opp' scalar), OneEuro-smoothed. It drives (a) the thumb's SPREAD
        # outright (its only usable source -- see the synergy block above) plus the blend of the
        # thumb onto the pinch pose, and (b) the fingers' late spread convergence + closure floor.
        # The fingers' flex/curl ALWAYS follow the operator: teleop must feel like teleop.
        tn = norms.get("thumb") or {}
        opp = self._teleop_smooth("thumb", "opp", float(tn.get("opp", 0.0)), dt) if tn else 0.0
        wt = _thumb_w(opp)
        w = _assist_w(opp)
        wc = _close_w(opp)
        pinch = getattr(self, "_teleop_pinch", None) or _PINCH_FALLBACK
        frame = {}
        for f in self._build.fingers:
            pose = pinch.get(f.id) or {}
            if f.id == "thumb":
                if not tn:                           # no thumb tracked -> leave the thumb where it is
                    continue
                tuck_pose = (getattr(self, "_teleop_tuck", None) or _TUCK_FALLBACK).get("thumb", {})
                # how far along the path: the operator's own thumb closure (either dof will do -- a
                # tucked thumb flexes, a curled one curls); how far the pinch pulls counts too, so a
                # closing aperture still lands the thumb even with a lazily-read flex.
                fold = max(self._teleop_smooth(f.id, "flex", float(tn.get("flex", 0.0)), dt),
                           self._teleop_smooth(f.id, "curl", float(tn.get("curl", 0.0)), dt))
                drive = max(0.0, min(1.0, max(fold, wt)))
                vals = []
                for sid in f.servos:
                    s = self._servo_map.by_id(sid)
                    if s is None:
                        break                        # positional contract -> drop the whole finger
                    # which end of the path: folded-in vs opposed, chosen by the pinch aperture
                    target = (1.0 - wt) * tuck_pose.get(s.dof, _THUMB_REST) + wt * pose.get(s.dof, _THUMB_REST)
                    deg = (1.0 - drive) * _THUMB_REST + drive * target
                    # the thumb writes pose degrees (no `_teleop_dof_deg` range mapping), so clamp
                    # HERE too -- `_teleop_goals` clamps the servo write, this keeps the mirror honest
                    lo, hi = self._eff_rad.get((f.id, s.dof), (0.0, 0.0))
                    vals.append(int(round(max(math.degrees(lo), min(math.degrees(hi), deg)))))
                if len(vals) == len(f.servos):
                    frame[f.socket] = vals
                continue
            fn = norms.get(f.id)
            if not fn:
                continue
            vals = []
            for sid in f.servos:
                s = self._servo_map.by_id(sid)
                if s is None:
                    break                            # positional contract -> drop the whole finger
                n = self._teleop_smooth(f.id, s.dof, float(fn.get(s.dof, 0.0)), dt)
                deg = self._teleop_dof_deg(f.id, s.dof, n)
                if w > 0.0 and s.dof == "spread":    # late convergence magnet (lateral only)
                    deg = (1.0 - w) * deg + w * pose.get(s.dof, 0.0)
                elif wc > 0.0 and s.dof in ("flex", "curl"):
                    # closure FLOOR: at least as closed as the touching pose on the last stretch;
                    # never opens against the operator (max), so a deeper curl still wins
                    deg = max(deg, wc * pose.get(s.dof, 0.0))
                vals.append(int(round(deg)))
            if len(vals) == len(f.servos):
                frame[f.socket] = vals               # clamp to _eff_rad happens in _teleop_goals
        return frame

    def _teleop_goals(self, frame: dict) -> dict[int, int]:
        """{socket: [int deg,...]} -> {servo_id: logical_ticks}, clamped to the SAME effective range
        jogNorm uses (`_eff_rad`)."""
        goals: dict[int, int] = {}
        for f in self._build.fingers:
            vals = frame.get(f.socket)
            if not vals:
                continue
            for sid, deg in zip(f.servos, vals):
                s = self._servo_map.by_id(sid)
                if s is None:
                    continue
                lo, hi = self._eff_rad.get((f.id, s.dof), (0.0, 0.0))
                goals[sid] = rad_to_ticks(max(lo, min(hi, math.radians(deg))))
        return goals

    def _teleop_dof_deg(self, claw: str, dof: str, n: float) -> float:
        """One (finger,dof) NORM -> degrees. spread is bipolar around the 0 REST (n in [-1,1]:
        +1->hi, -1->lo); flex/curl map REST(0)->closed(hi)."""
        lo, hi = self._eff_rad.get((claw, dof), (0.0, 0.0))
        if dof == "spread":
            return math.degrees(n * (hi if n >= 0 else -lo))
        rest = min(max(0.0, lo), hi)
        return math.degrees(rest + n * (hi - rest))

    def _teleop_smooth(self, claw: str, dof: str, norm: float, dt: float) -> float:
        key = (claw, dof)
        filt = self._teleop_filt.get(key)
        if filt is None:
            filt = OneEuro()
            self._teleop_filt[key] = filt
        return filt(norm, dt)

    # ---- slots (QML) ----
    @Slot(result=str)
    def enableTeleop(self) -> str:
        """Turn teleop ON: start the UDP receiver for the external WiLoR tracker + the motion loop.
        No hand camera -- the tracker is a separate process (tracking/wilor_teleop.py). Manual
        control, no ARM needed; nothing moves until torque is on and a fresh stream frame arrives.
        Default OFF: this is the conscious enable."""
        if self._teleop_stream is None:
            self._teleop_stream = TeleopStreamReceiver(port=_STREAM_PORT)
        try:
            self._teleop_stream.start()
        except Exception as e:
            return json.dumps({"ok": False,
                               "error": f"stream receiver failed to bind :{_STREAM_PORT} -- {e}"})
        self._teleop_on = True
        self._teleop_last_t = None
        self._set_stream_live(False)                  # nothing has arrived yet on this run
        self._teleop_pinch = self._pose_target(_PINCH_POSE_PREFIX, _PINCH_FALLBACK)
        self._teleop_tuck = self._pose_target(_TUCK_POSE_PREFIX, _TUCK_FALLBACK)   # both re-read on
        #                                       every enable -> re-capture in the app, no code edit
        self._teleop_filt = {}                        # fresh smoothing (no carry-over from last run)
        self._teleop_prev_frame = None                # first frame always syncs the QML mirror
        self._teleop_timer.start(_TELEOP_MS)
        try:                                          # twin drops to ~10 fps while teleop runs (perf)
            self._timer.setInterval(_RENDER_TELEOP_MS)
        except Exception:
            pass
        self.teleopStateChanged.emit()
        return json.dumps({"ok": True, "teleop": True, "mode": "wilor", "port": _STREAM_PORT})

    @Slot(result=str)
    def disableTeleop(self) -> str:
        """Stop the motion loop + the UDP receiver. Also the E-STOP hook."""
        self._teleop_on = False
        self._teleop_timer.stop()
        self._set_stream_live(False)                  # the receiver is going away
        recv = getattr(self, "_teleop_stream", None)
        if recv is not None:
            try:
                recv.stop()
            except Exception:
                pass
        try:                                          # restore the full twin render rate
            self._timer.setInterval(_RENDER_MS)
        except Exception:
            pass
        self.teleopStateChanged.emit()
        return json.dumps({"ok": True, "teleop": False})
