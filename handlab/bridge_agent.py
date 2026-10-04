"""Bridge mixin: the AGENT control plane (external controller / MCP server / Claude).

Slot/helper mixin for the Bridge (see bridge_hardware.py for why only @Slot + plain helpers
live in a mixin — @Property/@Signal stay on the Bridge QObject in bridge.py). These are the
*only* entry points an external controller uses, and they are the ONE place the ARM gate is
enforced: a disarmed hand refuses every move here, so a buggy MCP client can't drive it. The
human UI keeps using the ungated slots (jogNorm / applyPose / safeHome / setTorqueAll), so
arming never gets in the operator's way.

Everything routes through the existing safety: agentSetDof clamps to the SAME effective range
as jogNorm (`_eff_rad`); agentApplyPose/agentSafeHome delegate to the existing pose + phased
safe-home path; the driver keeps the force/velocity caps + no-jump torque enable. estop and
torque-off are always allowed (they only make the hand safe).
"""
from __future__ import annotations

import json
import math

from PySide6.QtCore import Slot

from .hal.base import rad_to_ticks, ticks_to_deg


def _err(msg: str) -> str:
    return json.dumps({"ok": False, "error": msg})


class BridgeAgentMixin:

    # ---- arm gate (armed property + armedChanged signal live on the Bridge QObject) ----
    def _emit_armed(self) -> None:
        """Isolated so tests on a bare Bridge.__new__ instance (no QObject) can stub it."""
        self.armedChanged.emit()

    @Slot(bool)
    def setArmed(self, on: bool) -> None:
        """Arm/disarm the agent control plane. Arming is a HUMAN action (the Agent-tab toggle);
        the remote control API can only DISARM, never arm — see control_server.set_arm."""
        on = bool(on)
        if on == self._armed:
            return
        self._armed = on
        self._emit_armed()

    def _finger_ord(self, name: str) -> int:
        for i, f in enumerate(self._build.fingers):
            if f.id == name:
                return i
        return -1

    # ---- observe (read-only, always allowed) ----
    @Slot(result=str)
    def agentState(self) -> str:
        """One JSON snapshot for the controller: driver, armed, online ids, raw servo telemetry,
        a finger/dof map (current degrees + effective range, so Claude knows the addressing and
        the limits) and the saved-pose names (the skill library). Uses the cached 10 Hz telemetry
        (`_last_tele`) — no extra bus traffic."""
        tele = getattr(self, "_last_tele", {}) or {}
        driver = getattr(self._driver, "driver_type", "") if self._driver else ""
        dofs = []
        for s in self._servo_map.servos:
            lo, hi = self._eff_rad.get((s.finger, s.dof), (0.0, 0.0))
            st = tele.get(str(s.id))
            # only report an angle for an ONLINE servo — a dropped one reads pos=0, which would
            # otherwise surface as a phantom −180° instead of "unknown".
            deg = (ticks_to_deg(st["pos"])
                   if st and st.get("online") else None)
            dofs.append({"finger": s.finger, "dof": s.dof, "servo": s.id, "deg": deg,
                         "range_deg": [round(math.degrees(lo)), round(math.degrees(hi))]})
        return json.dumps({
            "driver": driver,
            "armed": bool(self._armed),
            "online": sorted(self._online),
            "servos": tele,
            "dofs": dofs,
            "poses": [{"name": p["name"], "kind": p.get("kind", "pose")} for p in self._poses],
        })

    # ---- act (gated on armed) ----
    @Slot(str, str, float, result=str)
    def agentSetDof(self, finger: str, dof: str, deg: float) -> str:
        """Command one DOF to `deg`, CLAMPED to its effective range (same source as jogNorm's
        `_eff_rad`), then converted to ticks + sent through the driver (caps intact)."""
        if not self._armed:
            return _err("disarmed — arm the hand in the Agent panel first")
        if self._driver is None:
            return _err("no driver")
        rng = self._eff_rad.get((finger, dof))
        if rng is None:
            return _err(f"unknown dof {finger!r}/{dof!r}")
        servo = self._servo_map.by_finger_dof(finger, dof)
        if servo is None:
            return _err(f"no servo for {finger!r}/{dof!r}")
        lo, hi = rng
        want = math.radians(float(deg))
        val = max(lo, min(hi, want))
        self._driver.set_goal_position(servo.id, rad_to_ticks(val))
        return json.dumps({"ok": True, "finger": finger, "dof": dof, "servo": servo.id,
                           "deg": round(math.degrees(val), 1),
                           "clamped": abs(val - want) > 1e-9,
                           "range_deg": [round(math.degrees(lo)), round(math.degrees(hi))]})

    @Slot(str, result=str)
    def agentApplyPose(self, name: str) -> str:
        """Recall a saved POSE by name (goes through the same clamped playbackFrame path the
        Poses panel uses). Refuses a recorded clip — agentState lists both, but applyPose only
        plays poses; use agentPlayClip for a clip (else this would falsely report success)."""
        if not self._armed:
            return _err("disarmed — arm the hand in the Agent panel first")
        p = self._find_pose(name)
        if p is None:
            return _err(f"no pose named {name!r}")
        if p.get("kind", "pose") != "pose":
            return _err(f"{name!r} is a recorded clip, not a pose — use agentPlayClip")
        self.applyPose(name)
        return json.dumps({"ok": True, "pose": name})

    @Slot(str, bool, result=str)
    def agentPlayClip(self, name: str, loop: bool = False) -> str:
        """Play a recorded motion CLIP ('skill') by name; optionally loop. Gated on armed."""
        if not self._armed:
            return _err("disarmed — arm the hand in the Agent panel first")
        p = self._find_pose(name)
        if p is None:
            return _err(f"no clip named {name!r}")
        if p.get("kind") != "clip":
            return _err(f"{name!r} is a pose, not a clip — use agentApplyPose")
        self.playClip(name, bool(loop))
        return json.dumps({"ok": True, "clip": name, "loop": bool(loop)})

    @Slot(str, result=str)
    def agentSafeHome(self, finger: str = "") -> str:
        """Phased collision-aware return home (curl → flex → spread). `finger` = "" / "all" homes
        the whole hand, else a finger id (index/thumb/mid)."""
        if not self._armed:
            return _err("disarmed — arm the hand in the Agent panel first")
        ord_ = -1
        if finger and finger.lower() not in ("all", "*"):
            ord_ = self._finger_ord(finger)
            if ord_ < 0:
                return _err(f"unknown finger {finger!r}")
        self.safeHome(ord_)
        return json.dumps({"ok": True, "finger": finger or "all"})

    @Slot(bool, result=str)
    def agentTorque(self, on: bool) -> str:
        """Enable/disable torque on all servos. Enabling needs arm (it can move the hand on
        re-engage settle); disabling is always allowed (it only makes the hand limp/safe)."""
        on = bool(on)
        if on and not self._armed:
            return _err("disarmed — arm before enabling torque")
        self.setTorqueAll(on)
        return json.dumps({"ok": True, "torque": on})

    @Slot(result=str)
    def agentEstop(self) -> str:
        """Always allowed. Abort any homing, drop torque on every servo (elastics stretch the
        hand), and DISARM — releasing the stop never moves anything; the human re-arms."""
        try:
            self.stopHoming()
        except Exception:
            pass
        try:
            self.disableTeleop()                     # kill the hand-teleop loop too (no motion under stop)
        except Exception:
            pass
        if self._driver is not None:
            self.setTorqueAll(False)
        self._armed = False
        self._emit_armed()
        return json.dumps({"ok": True, "estop": True, "armed": False})

    # ---- snapshot (plain method, not a QML slot — returns raw bytes for the control server) ----
    def _current_snapshot_image(self):
        """The QImage the snapshot encodes: the webcam when it's the selected source AND the camera
        is LIVE with a real frame, else the twin. Gating on liveness (not just a non-null frame) is
        deliberate: a stopped/unplugged/errored camera keeps its last frame around, and serving that
        frozen frame as if live would let the agent reason on a stale view of the physical hand. So a
        cam that's selected but not delivering (starting / TCC pending / dead) falls back to the twin
        rather than returning a stale image. Runs on the Qt thread."""
        if getattr(self, "_snapshot_source", "twin") == "cam":
            cam = getattr(self, "camera", None)
            if cam is not None and cam.active:
                img = cam.current()
                if img is not None and not img.isNull():
                    return img
        return self.image_provider.current()

    def agent_snapshot_jpeg(self, max_px: int = 768, quality: int = 70) -> bytes:
        """Latest frame as JPEG bytes — the live webcam when selected, else the rendered TWIN
        (Phase 1 default). Runs on the Qt thread (via the control-server invoker), so it touches
        the QImage safely."""
        from PySide6.QtCore import QBuffer, QByteArray, QIODevice, Qt
        img = self._current_snapshot_image()
        if img is None or img.isNull():
            return b""
        if max_px and (img.width() > max_px or img.height() > max_px):
            img = img.scaled(max_px, max_px, Qt.AspectRatioMode.KeepAspectRatio,
                             Qt.TransformationMode.SmoothTransformation)
        ba = QByteArray()
        buf = QBuffer(ba)
        buf.open(QIODevice.OpenModeFlag.WriteOnly)
        img.save(buf, "JPEG", int(quality))
        return bytes(ba)

    # ---- marker detection (read-only perception; no arm needed) ----
    def agent_detections(self) -> dict:
        """Latest ArUco detections from the LIVE camera as a plain dict (the control server JSON-
        encodes it). Read-only — perception never needs arm. Empty when the camera is off / not
        delivering, or when opencv (the [vision] extra) isn't installed. Runs on the Qt thread."""
        src = getattr(self, "_snapshot_source", "twin")   # tools check this: /snapshot.jpg serves
        cam = getattr(self, "camera", None)               # the TWIN unless the source is "cam"
        if cam is None or not cam.active:
            return {"ok": True, "active": False, "calibrated": False, "device": "",
                    "snapshot_source": src, "count": 0, "markers": []}
        markers = cam.latest_detections()
        return {"ok": True, "active": True, "calibrated": bool(cam.calibrated),
                "device": cam.device_id, "snapshot_source": src,
                "count": len(markers), "markers": markers}
