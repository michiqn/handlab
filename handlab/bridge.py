"""Bridge — the QObject that connects handlab's Python core to QML.

Owns the live build + the MuJoCo twin. The build is fixed at boot from builds/<HANDLAB_BUILD>.yaml
(the single source of truth for the servo mapping) → the Bridge composes the scene + ServoMap and
starts the twin. Control sliders call `jogNorm(...)` to drive actuators. One process, direct calls —
no HTTP.
"""

from __future__ import annotations

import json
import math

import numpy as np

from PySide6.QtCore import QObject, QTimer, Signal, Slot, Property
from PySide6.QtGui import QImage
from PySide6.QtQuick import QQuickImageProvider

from . import compose, library_io
from .hal.base import rad_to_ticks, ticks_to_rad
from .perf import PERF
from .bridge_hardware import BridgeHardwareMixin
from .bridge_poses import BridgePosesMixin
from .bridge_view import BridgeViewMixin
from .bridge_agent import BridgeAgentMixin
from .bridge_record import BridgeRecordMixin
from .bridge_teleop import BridgeTeleopMixin, _RENDER_MS
from .recorder import DemoRecorder
from .control_server import QtInvoker
from .models import FingerType

# Theme DOF colour groups; a key that doesn't normalise to one of these falls back to accent.
_DOF_GROUPS = {"spread", "flex", "curl", "bend", "curve", "elong"}


def _group_from_key(key: str) -> str:
    """Normalise a dof key to a Theme colour group (e.g. 'mcp_spread' -> 'spread')."""
    tail = key.rsplit("_", 1)[-1].lower()
    return tail if tail in _DOF_GROUPS else key


def _catalog_entry(ft: FingerType) -> dict:
    """Map a library FingerType -> the QML ADD-FINGER catalog dict, honouring `ui:` hints
    and synthesising sensible defaults (degrees from the radian range) when they're absent."""
    ui = ft.ui or {}
    ui_dofs = ui.get("dofs", {})
    mech = ui.get("mech", "tendon-driven")
    dofs = []
    for d in ft.dofs:
        u = ui_dofs.get(d.key, {})
        mn = u.get("min", round(math.degrees(d.range[0])))
        mx = u.get("max", round(math.degrees(d.range[1])))
        home = u.get("home", 0 if mn <= 0 <= mx else mn)
        dofs.append({
            "id": u.get("id", d.key), "label": d.label,
            "group": u.get("group", _group_from_key(d.key)),
            "unit": u.get("unit", "°"), "min": mn, "max": mx, "home": home,
            "coupled": bool(d.coupled),
        })
    return {
        "id": ui.get("id", ft.name), "name": ft.label, "mech": mech, "motors": ft.motors,
        "blurb": ui.get("blurb", f"{mech} · {len(ft.dofs)} dof"),
        "adapter": ui.get("adapter", ""),   # only shown when a type declares one
        "slot": ft.slot,
        "dofs": dofs,
    }


class FrameImageProvider(QQuickImageProvider):
    """Latest-frame image provider, shared by the twin render and the webcam (camera.py aliases
    it as CamImageProvider). QML always needs a valid QImage; the agent snapshot must distinguish
    "no frame yet" from a real frame, so the placeholder stays separate and _img stays None until
    the first real frame (→ snapshot returns 503 rather than a fake image the model might reason
    over). set_frame(None) clears back to that state (camera teardown)."""

    def __init__(self):
        super().__init__(QQuickImageProvider.ImageType.Image)
        self._placeholder = QImage(8, 8, QImage.Format.Format_RGB888)
        self._placeholder.fill(0x101216)
        self._img: QImage | None = None

    def set_frame(self, img: QImage | None) -> None:
        self._img = img

    def current(self) -> QImage | None:
        return self._img

    def requestImage(self, id, size, requestedSize):
        return self._img if self._img is not None else self._placeholder


class Bridge(BridgeHardwareMixin, BridgePosesMixin, BridgeViewMixin, BridgeAgentMixin,
             BridgeRecordMixin, BridgeTeleopMixin, QObject):
    structureChanged = Signal()
    frameTick = Signal(int)
    telemetryChanged = Signal(str)         # JSON {servo_id: {pos,torque,moving,temp,volt,online}}
    posesChanged = Signal()
    playbackFrame = Signal(str)            # JSON {socket: [joint_deg,...]} — QML drives it
    armedChanged = Signal()                # agent control plane armed/disarmed (Agent tab)
    agentActivity = Signal(str)            # JSON {tool,detail,ok} — one line per remote command (Agent-tab log)
    cameraFrame = Signal(int)              # per delivered webcam frame -> Agent-tab live Image poll
    cameraDevicesChanged = Signal()        # videoInputs hotplug -> refresh the device picker
    cameraStateChanged = Signal()          # camera active / snapshot source changed
    cameraError = Signal(str)              # e.g. macOS camera permission denied -> Agent-tab hint
    cameraDetectionsChanged = Signal()     # new ArUco marker detections -> Agent-tab count/overlay
    demoRecordingChanged = Signal()        # demo recorder start/stop/discard -> Demo card state
    teleopFrame = Signal(str)              # JSON {socket:[joint_deg,...]} -> QML syncPoseFrame (display
    #                                        mirror only; teleop applies motion directly in Python, §37)
    teleopStateChanged = Signal()          # teleop enabled / disabled
    teleopStreamChanged = Signal()         # WiLoR stream went live / went stale (recorder readiness)
    twinRenderChanged = Signal()           # twin render paused/resumed (perf toggle)

    def __init__(self, build_name: str = "claw3f", parent: QObject | None = None):
        super().__init__(parent)
        self.image_provider = FrameImageProvider()
        # webcam (Phase 2): the snapshot source can be the twin or a live camera frame. Built
        # guarded so a missing/broken QtMultimedia never takes the whole Bridge down (the app
        # would else fall back to mock and lose hardware control).
        self._snapshot_source = "twin"          # "twin" | "cam"
        self.camera = None
        try:
            from .camera import CameraController
            self.camera = CameraController(self)
            self.camera.frameReady.connect(self.cameraFrame)
            self.camera.devicesChanged.connect(self.cameraDevicesChanged)
            self.camera.stateChanged.connect(self.cameraStateChanged)
            self.camera.errorText.connect(self.cameraError)
            self.camera.detectionsChanged.connect(self.cameraDetectionsChanged)
        except Exception as e:                  # noqa: BLE001 — camera is optional
            print(f"[handlab] camera unavailable ({e}); snapshot stays on the twin.")
        self._frame_no = 0
        self._twin = None
        self._cam_dirty = True              # gizmo az/el cache (bridge_view._cam_orientation)
        self._cam_cache = (45.0, 20.0)
        self._driver = None
        self._physics = False
        self._online: set[int] = set()
        self._armed = False                 # agent control plane starts disarmed (human arms it)
        self._agent_port = 0                # control-server port (app.py sets it; 0 = agent API off)
        self._last_tele: dict = {}          # cached 10 Hz telemetry, served to the agent read path
        self._invoker = QtInvoker()         # marshals control-server calls onto this (Qt) thread
        self._eff_rad: dict = {}            # (finger.id, dof.key) -> (lo,hi) rad, set in _rebuild
        self._dof_ranges_json = "{}"        # cached dofRanges / dofUserRanges JSON, rebuilt in _rebuild
        self._dof_user_ranges_json = "{}"   #   → the getters serve a string, no per-read YAML disk I/O
        self._visible = True                # window-visibility gate for the render tick (perf)
        self._catalog: list[dict] = []
        self._adapter_catalog: list[dict] = []
        self._build_catalog()
        self._build = library_io.load_build(build_name)
        self._rebuild()

        # poses + recordings (per build; the loaded build name is the stable scope)
        self._pose_scope = build_name
        self._poses = library_io.load_poses(build_name)
        for _p in self._poses:                      # clean legacy clips' idle (idempotent)
            if _p.get("kind") == "clip" and _p.get("frames"):
                _p["frames"] = self._trim_idle(_p["frames"])
        self._recording = False
        self._rec_buf: list[tuple[float, dict]] = []
        self._rec_t0 = 0.0
        # Demo recorder (visuomotor-IL episodes) — observe-only; samples on the telemetry beat below.
        import os as _os
        self._demo = DemoRecorder(_os.environ.get("HANDLAB_DEMOS",
                                                  _os.path.expanduser("~/handlab_demos")))
        self._play_timer = QTimer(self)
        self._play_timer.setSingleShot(True)
        self._play_timer.timeout.connect(self._play_step)
        self._play_frames: list[dict] = []
        self._play_dt = 0.1
        self._play_i = 0
        self._play_loop = False

        # WiLoR hand-tracking teleop (default OFF; self-rescheduling loop mirrors _play_step). The
        # human side is an external process streaming per-finger norms over UDP (§41).
        self._teleop_on = False
        self._teleop_stream = None          # UDP receiver for the external WiLoR tracker
        self._teleop_stream_live = False    # tracker actually delivering (recorder readiness check)
        self._estopped = False              # mirrors QML eStopped (set FIRST in toggleStop) — gates
        #                                     the direct teleop write path (QML only mirrors now)
        self._teleop_prev_frame = None      # last emitted display frame (identity gate, no churn)
        self._teleop_last_t = 0.0
        self._teleop_filt: dict = {}                 # (claw,dof) -> OneEuro smoothing filter
        self._teleop_timer = QTimer(self)
        self._teleop_timer.setSingleShot(True)
        self._teleop_timer.timeout.connect(self._teleop_step)

        # phased safe-home sequencer (curl -> flex -> spread with settle waits)
        self._home_timer = QTimer(self)
        self._home_timer.setSingleShot(True)
        self._home_timer.timeout.connect(self._home_step)
        self._home_groups: list[str] = []           # remaining dof groups to home
        self._home_scope: list = []                 # FingerInstance list being homed
        self._home_done: set[str] = set()           # groups already at home
        self._home_involved: set[int] = set()       # servo ids settling in the current phase
        self._home_t0 = 0.0                         # current phase start (monotonic)

        self._twin_paused = False          # perf toggle: freeze the twin render (teleop still drives)
        self._timer = QTimer(self)
        self._timer.setInterval(_RENDER_MS)   # ~30 fps render (shared with teleop's restore)
        self._timer.timeout.connect(self._tick)
        self._timer.start()

        self._tele_timer = QTimer(self)
        self._tele_timer.setInterval(100)  # 10 Hz servo telemetry (20 Hz lagged: blocking serial IO
        #                                    on the GUI thread; for smoother → raise baud, not the rate)
        self._tele_timer.timeout.connect(self._tele_tick)
        self._tele_timer.start()

    # ---- (re)compose + hot-reload the twin ----
    def _rebuild(self) -> None:
        self._ftypes = {}
        for f in self._build.fingers:
            ft = compose._try_load(f.type)
            if ft is not None:
                self._ftypes[f.id] = ft
        cal = library_io.load_calibration()
        port_ranges = self._adapter_port_ranges()  # read the adapter yaml ONCE per rebuild
        dof_ranges = {sid: c["range_deg"] for sid, c in cal.items() if c.get("range_deg")}
        self._composed = compose.compose(self._build, physics=self._physics,
                                         dof_ranges=dof_ranges)
        self._servo_map = self._composed.servo_map
        self._compute_eff_ranges(cal, port_ranges)   # effective (finger,dof) ranges for jogNorm + UI
        self._build_dof_range_caches(cal, port_ranges)  # cache the DOF-range JSON (no per-getter disk read)
        scene = str(self._composed.scene_path)
        if self._twin is None:
            try:
                from .sim import Twin
                self._twin = Twin(scene, width=900, height=720)
                self._twin.start()
            except Exception as e:
                print(f"[handlab] twin not active ({e}); viewport stays placeholder.")
        else:
            try:
                self._twin.reload(scene)
            except Exception as e:
                print(f"[handlab] twin reload failed ({e}).")
        self._cam_dirty = True                      # camera state may have reset with the scene
        self._make_driver()

    def _adapter_port_ranges(self) -> dict:
        """{port_id: dof_ranges_deg} of the current build's adapter (guarded load; {} without one)
        — shared by _compute_eff_ranges and the dofRanges property."""
        adapter = None
        try:
            if self._build.adapter:
                adapter = library_io.load_adapter_type(self._build.adapter)
        except Exception:
            adapter = None
        return {p.id: p.dof_ranges_deg for p in (adapter.ports if adapter else [])}

    def _compute_eff_ranges(self, cal: dict, port_ranges: dict) -> None:
        """(finger.id, dof.key) -> (lo_rad, hi_rad): the effective range (user override > adapter
        port override > base), the single source shared with compose so slider/jogNorm/twin agree."""
        self._eff_rad = {}
        for f in self._build.fingers:
            ft = self._ftypes.get(f.id)
            if ft is None:
                continue
            over = port_ranges.get(f.socket, {})
            for sid, d in zip(f.servos, ft.dofs):
                self._eff_rad[(f.id, d.key)] = compose.effective_range_rad(
                    d, over, cal.get(sid, {}).get("range_deg"))

    def _build_dof_range_caches(self, cal: dict, port_ranges: dict) -> None:
        """Precompute the dofRanges + dofUserRanges JSON once per rebuild. The QML getters are hit
        per jog/playback/home frame (via Main.qml dofsFor), so serving a cached string instead of
        re-reading calibration.yaml + the adapter yaml on every read removes the §37 disk-thrash from
        the jog path. Same values the getters used to compute inline."""
        ranges, user = {}, {}
        for fo, f in enumerate(self._build.fingers):
            ft = self._ftypes.get(f.id)
            if ft is None:
                continue
            over = port_ranges.get(f.socket, {})
            for do, (sid, d) in enumerate(zip(f.servos, ft.dofs)):
                u = cal.get(sid, {}).get("range_deg")
                if u:
                    ranges[f"{fo}:{do}"] = [round(u[0]), round(u[1])]
                    user[f"{fo}:{do}"] = True                 # user 'edit limits' (resettable)
                elif d.key in over:
                    ranges[f"{fo}:{do}"] = [round(over[d.key][0]), round(over[d.key][1])]
        self._dof_ranges_json = json.dumps(ranges)
        self._dof_user_ranges_json = json.dumps(user)

    def _resolve_servo(self, finger_ord: int, dof_ord: int):
        """(finger_ord, dof_ord) -> (servo_id, dof) or None — shared by jog/invert/range slots."""
        if not (0 <= finger_ord < len(self._build.fingers)):
            return None
        f = self._build.fingers[finger_ord]
        ft = self._ftypes.get(f.id)
        if ft is None or not (0 <= dof_ord < len(ft.dofs)):
            return None
        d = ft.dofs[dof_ord]
        servo = self._servo_map.by_finger_dof(f.id, d.key)
        return None if servo is None else (servo.id, d)

    # ---- driver: the control plane behind the UI (HANDLAB_DRIVER=mock|mujoco|dynamixel) ----

    # ---- hardware connection (status chip -> Connect panel) ----
    @Property(str, notify=telemetryChanged)
    def servoSigns(self) -> str:
        from .hal.dynamixel_driver import DynamixelDriver
        if isinstance(self._driver, DynamixelDriver):
            return json.dumps({str(k): v["sign"] for k, v in self._driver.calibration().items()})
        return "{}"

    @Property(str, notify=structureChanged)
    def dofRanges(self) -> str:
        """{"<fingerOrd>:<dofOrd>": [min_deg, max_deg]} for DOFs with an active range override
        (user 'edit limits' or an adapter port) — the slider's effective bounds. Served from the
        cache built in _rebuild (no per-read disk I/O; the jog path hits this every frame)."""
        return self._dof_ranges_json

    @Property(str, notify=structureChanged)
    def dofUserRanges(self) -> str:
        """{"<fingerOrd>:<dofOrd>": true} for DOFs the USER range-edited (resettable) — as opposed
        to a structural adapter-port mirror. Served from the _rebuild cache (no per-read disk I/O)."""
        return self._dof_user_ranges_json

    @Slot(int, int, int, int, result=str)
    def setDofRange(self, finger_ord: int, dof_ord: int, min_deg: int, max_deg: int) -> str:
        """User 'edit limits': set a DOF's effective [min,max]° — the sim slider/joint AND the
        commanded servo travel. Persists as range_deg in calibration.yaml; recomposes the twin."""
        r = self._resolve_servo(finger_ord, dof_ord)
        if r is None:
            return json.dumps({"ok": False, "error": "bad dof"})
        servo_id, _d = r
        lo, hi = int(min_deg), int(max_deg)
        if lo >= hi:
            return json.dumps({"ok": False, "error": "min must be < max"})
        if not (lo <= 0 <= hi):                     # keep home (0° = stretched rest) reachable
            return json.dumps({"ok": False, "error": "range must include 0° (home/rest)"})
        cal = library_io.load_calibration()
        cal.setdefault(servo_id, {"sign": 1, "offset": None, "range_deg": None})["range_deg"] = [lo, hi]
        library_io.save_calibration(cal)
        self._rebuild()
        self.structureChanged.emit()
        return json.dumps({"ok": True, "range_deg": [lo, hi]})

    @Slot(int, int, result=str)
    def resetDofRange(self, finger_ord: int, dof_ord: int) -> str:
        """Clear a DOF's user range override → back to the base/adapter range."""
        r = self._resolve_servo(finger_ord, dof_ord)
        if r is None:
            return json.dumps({"ok": False, "error": "bad dof"})
        servo_id, _d = r
        cal = library_io.load_calibration()
        if servo_id in cal and cal[servo_id].get("range_deg"):
            cal[servo_id]["range_deg"] = None
            library_io.save_calibration(cal)
            self._rebuild()
            self.structureChanged.emit()
        return json.dumps({"ok": True})

    @Property(int, notify=structureChanged)
    def hwBaud(self) -> int:
        return self._build.hardware.baud

    @Property(str, notify=structureChanged)
    def buildSpec(self) -> str:
        """The loaded build (from builds/<name>.yaml) as QML-shaped JSON, so the UI SEEDS its
        assembly from the yaml — the yaml is the single source of truth for the servo mapping,
        not a hardcoded QML preset. For built-in finger types the library folder name == the
        catalog id the QML `typeById` expects."""
        return json.dumps({
            "name": self._pose_scope,
            "mount": self._build.mount,
            "adapter": self._build.adapter or "",
            "fingers": [{"socket": f.socket, "type": f.type, "servos": list(f.servos)}
                        for f in self._build.fingers],
        })

    # ---- part catalogs (the sources the QML ADD-FINGER / CENTER-ADAPTER tiles render) ----
    def _build_catalog(self) -> None:
        """Scan the library and build the QML-shaped finger + adapter catalogs."""
        lib = library_io.list_library()
        cat: list[dict] = []
        for name in lib["fingers"]:
            try:
                ft = library_io.load_finger_type(name)
            except Exception:
                continue
            cat.append(_catalog_entry(ft))
        self._catalog = cat

        adapters: list[dict] = []
        for name in lib.get("adapters", []):
            try:
                at = library_io.load_adapter_type(name)
            except Exception:
                continue
            adapters.append({
                "id": at.name, "name": at.label, "fits": at.fits,
                "blurb": f"{at.fits} norm · {len(at.ports)} finger" + ("s" if len(at.ports) != 1 else ""),
                # port label prefers the slot name (Index slot) over the generic ordinal
                "ports": [{"id": p.id,
                           "label": (p.id.capitalize() + " slot") if p.accepts else f"Adapter port {i + 1}",
                           "short": f"P{i + 1}",
                           "accepts": p.accepts,
                           "ranges": {k: list(v) for k, v in p.dof_ranges_deg.items()}}
                          for i, p in enumerate(at.ports)],
            })
        self._adapter_catalog = adapters

    # The catalogs are fixed at boot (the library isn't mutated at runtime) → notify on
    # structureChanged (a recompose is the only thing that could change them).
    @Property(str, notify=structureChanged)
    def fingerCatalog(self) -> str:
        return json.dumps(self._catalog)

    @Property(str, notify=structureChanged)
    def adapterCatalog(self) -> str:
        return json.dumps(self._adapter_catalog)

    # ---- control (slider -> driver; the driver decides what "hardware" means) ----
    @Slot(int, int, float)
    def jogNorm(self, finger_ord: int, dof_ord: int, norm: float) -> None:
        if not (0 <= finger_ord < len(self._build.fingers)) or self._driver is None:
            return
        f = self._build.fingers[finger_ord]
        ft = self._ftypes.get(f.id)
        if ft is None or not (0 <= dof_ord < len(ft.dofs)):
            return
        d = ft.dofs[dof_ord]
        lo, hi = self._eff_rad.get((f.id, d.key), d.range)   # effective range (user/port override)
        val = lo + max(0.0, min(1.0, norm)) * (hi - lo)
        servo = self._servo_map.by_finger_dof(f.id, d.key)
        if servo is not None:
            self._driver.set_goal_position(servo.id, rad_to_ticks(val))



    @Property(str, notify=structureChanged)
    def driverName(self) -> str:
        labels = {"mujoco": "MuJoCo sim", "mock": "mock", "dynamixel": "Dynamixel"}
        return labels.get(self._driver.driver_type if self._driver else "", "—")

    # ---- agent control plane (Agent tab + the MCP control server share this) ----
    @Property(bool, notify=armedChanged)
    def armed(self) -> bool:
        return self._armed

    @Property(int, constant=True)
    def agentPort(self) -> int:
        """The local control-server port (0 = agent API disabled). app.py sets `_agent_port`
        before engine.load(Main.qml), so it never changes for the session → constant is safe."""
        return self._agent_port

    def qt_invoke(self, fn, timeout: float = 5.0):
        """Run `fn` on the Qt thread and return its result — the control server's HTTP worker
        threads reach the Bridge/driver through here so serial IO stays single-threaded."""
        return self._invoker.invoke(fn, timeout)

    # ---- webcam (Agent-tab live view + snapshot source) ----
    @Property(str, notify=cameraDevicesChanged)
    def cameraDevices(self) -> str:                 # JSON [{id,name,default}]
        return json.dumps(self.camera.device_list() if self.camera else [])

    @Property(int, notify=cameraFrame)
    def camFrame(self) -> int:
        return self.camera.frame_no if self.camera else 0

    @Property(bool, notify=cameraStateChanged)
    def cameraActive(self) -> bool:
        return bool(self.camera and self.camera.active)

    @Property(str, notify=cameraStateChanged)
    def snapshotSource(self) -> str:                # "twin" | "cam"
        return self._snapshot_source

    @Property(int, notify=cameraDetectionsChanged)
    def markerCount(self) -> int:                   # ArUco markers currently visible (Agent-tab badge)
        return len(self.camera.latest_detections()) if self.camera else 0

    # ---- demo recorder (visuomotor-IL episodes) ----
    @Property(bool, notify=demoRecordingChanged)
    def demoRecording(self) -> bool:
        return self._demo.active

    @Property(int, notify=demoRecordingChanged)
    def demoEpisodeCount(self) -> int:              # episodes saved for the current task
        return self._demo.episode_count

    @Property(str, notify=demoRecordingChanged)
    def lastEpisodePath(self) -> str:
        return str(self._demo.last_dir) if self._demo.last_dir else ""

    # ---- WiLoR hand-tracking teleop ----
    @Property(str, notify=cameraStateChanged)
    def cameraDeviceId(self) -> str:                # the robot cam's RUNNING device ("" until started)
        return self.camera.device_id if self.camera else ""

    @Property(bool, notify=teleopStateChanged)
    def teleopEnabled(self) -> bool:
        return self._teleop_on

    @Property(bool, notify=teleopStreamChanged)
    def teleopStreamLive(self) -> bool:
        """Is the external WiLoR tracker actually delivering? Teleop can be ON with nothing arriving
        (tracker not started, wrong port, hand out of frame) — the demo recorder shows this as a
        readiness check, because recording that state yields episodes where the hand never moves."""
        return self._teleop_stream_live

    @Property(bool, notify=twinRenderChanged)
    def twinPaused(self) -> bool:
        return self._twin_paused

    @Slot(bool)
    def setEstopped(self, on: bool) -> None:
        """QML mirrors its eStopped flag here (FIRST call in toggleStop, both directions) — the
        direct teleop write path checks it before touching the driver. Layer 1 of the stop chain;
        disableTeleop + torqueOffAll follow as layers 2/3."""
        self._estopped = bool(on)

    @Slot(bool)
    def setTwinPaused(self, on: bool) -> None:
        """Freeze/resume the offscreen twin render — a perf escape hatch when the twin render + the
        teleop loop compete for CPU. Teleop + the driver keep running while paused (the twin is only
        a preview); the last frame stays on screen."""
        self._twin_paused = bool(on)
        self.twinRenderChanged.emit()

    @Slot(bool)
    def setWindowVisible(self, on: bool) -> None:
        """QML mirrors the window's visibility here → the render tick skips the offscreen GL render
        while the window is minimized/hidden (no behavior change while visible; free CPU/GPU when
        occluded)."""
        self._visible = bool(on)

    @Slot(str)
    def startCamera(self, device_id: str = "") -> None:
        """Start (or switch to) a camera device and make it the snapshot source. First call raises
        the macOS camera-permission prompt; snapshot degrades to the twin until a frame arrives."""
        if self.camera and self.camera.start(device_id):
            self._snapshot_source = "cam"
            self.cameraStateChanged.emit()

    @Slot()
    def stopCamera(self) -> None:
        if self.camera:
            self.camera.stop()
        self._snapshot_source = "twin"              # never leave snapshot pointed at a dead cam
        self.cameraStateChanged.emit()

    @Slot(str)
    def setSnapshotSource(self, src: str) -> None:
        """Explicit twin<->cam toggle, independent of the camera on/off (e.g. keep the live feed
        running but snapshot the twin)."""
        self._snapshot_source = "cam" if src == "cam" else "twin"
        self.cameraStateChanged.emit()

    # ---- servo telemetry (10 Hz) -> QML ----
    def _tele_tick(self) -> None:
        if self._driver is None:
            return
        with PERF.lap("tele_tick"):
            self._tele_tick_body()

    def _tele_tick_body(self) -> None:
        try:
            with PERF.lap("tele_tick.read_states"):
                states = self._driver.read_states()
        except Exception:
            return
        # while real hardware is connected, the digital twin MIRRORS the real hand
        if self._twin is not None and getattr(self._driver, "driver_type", "") == "dynamixel":
            for s in self._servo_map.servos:
                st = states.get(s.id)
                if st is not None and s.id in self._online:
                    try:
                        self._twin.set_ctrl(s.actuator, ticks_to_rad(st.present_position))
                    except Exception:
                        pass
        if self._recording:                 # sample the measured pose on the telemetry beat
            import time
            self._rec_buf.append((time.monotonic() - self._rec_t0, self._grouped_dofs(states)))
        if self._demo.active:               # demo recorder: image+action+proprio+object on the beat
            try:                            # NEVER let the recorder break the telemetry/twin/safety beat
                self._demo_step(states)
            except Exception:
                pass
        out = {}
        for sid, st in states.items():
            out[str(sid)] = {
                "pos": st.present_position, "torque": st.torque_enabled,
                "moving": st.moving, "temp": st.present_temperature,
                "volt": st.present_voltage, "online": sid in self._online,
                "err": st.hw_error, "cur": st.present_current,
            }
        self._last_tele = out               # cache for the agent read path (agentState)
        self.telemetryChanged.emit(json.dumps(out))

    # ---- poses & recordings (per build; QML drives the actual DOFs via playbackFrame) ----



    @Property(str, notify=posesChanged)
    def posesJson(self) -> str:
        return json.dumps([{"name": p["name"], "kind": p.get("kind", "pose"),
                            "frames": len(p.get("frames", [])),
                            "secs": round(p.get("dt", 0.1) * len(p.get("frames", [])), 1)}
                           for p in self._poses])

    @Property(bool, notify=posesChanged)
    def isRecording(self) -> bool:
        return self._recording

    @Property(bool, notify=posesChanged)
    def isPlaying(self) -> bool:
        return self._play_timer.isActive()





    # ---- recording ----




    # ---- playback (Bridge = clock; QML applies each frame through its normal DOF path) ----
    # ---- phased safe-home: open all curls first (pulls the fingertips apart), then flex,
    # ---- then spread — collision-aware return home across 2-3 fingers ----
    # ---- camera (from the viewport mouse) ----
    # ---- sim playground (physics/grasp mode) ----
    @Property(bool, notify=structureChanged)
    def physicsOn(self) -> bool:
        return self._physics

    # ---- render tick ----
    def _tick(self) -> None:
        if self._twin is None or self._twin_paused or not self._visible:
            return                                          # paused / window hidden: skip the render
        with PERF.lap("twin.render"):
            try:
                arr = self._twin.render()
            except Exception:
                return
            if arr is None:
                return
            arr = np.ascontiguousarray(arr)
            h, w, _ = arr.shape
            img = QImage(arr.data, w, h, 3 * w, QImage.Format.Format_RGB888).copy()
        self.image_provider.set_frame(img)
        self._frame_no += 1
        self.frameTick.emit(self._frame_no)
