"""Twin — the MuJoCo physics twin: real-time stepping thread + offscreen renderer.

Carries over the hard-won lessons from craft-control:
  - physics runs on its OWN real-time thread (decoupled from reads/render), else motion stalls,
  - position-servo tuning lives in the composed scene (damping 0.005 / armature 1e-6 / kp 1 / kv .03),
  - offscreen rendering on macOS needs MUJOCO_GL=cgl + mujoco.Renderer.

`mujoco` is imported lazily so the package imports without it.
"""

from __future__ import annotations

import threading
import time


class Twin:
    def __init__(self, scene_path: str, width: int = 1100, height: int = 1100):
        self._scene_path = scene_path
        self._w, self._h = width, height
        self._mj = None
        self._model = None
        self._data = None
        self._renderer = None
        self._cam = None
        self._opt = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    # ---- lifecycle ----
    def start(self) -> None:
        import os
        os.environ.setdefault("MUJOCO_GL", "cgl")     # macOS offscreen GL
        import mujoco  # lazy
        self._mj = mujoco
        self._model = mujoco.MjModel.from_xml_path(self._scene_path)
        self._data = mujoco.MjData(self._model)
        mujoco.mj_forward(self._model, self._data)
        self._cam = mujoco.MjvCamera()
        mujoco.mjv_defaultFreeCamera(self._model, self._cam)   # frames the model nicely
        # render options: draw the WORLD coordinate frame (XYZ triad at the origin) so the
        # scene has a real orientation reference that turns WITH the camera (a static corner
        # legend can't). framelength/width scale it up from the tiny default to readable.
        self._opt = mujoco.MjvOption()
        mujoco.mjv_defaultOption(self._opt)
        self._opt.frame = mujoco.mjtFrame.mjFRAME_WORLD
        self._model.vis.scale.framelength = 0.35
        self._model.vis.scale.framewidth = 0.05
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, name="handlab-sim", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        with self._lock:
            self._renderer = None
            self._model = self._data = None

    def reload(self, scene_path: str) -> None:
        """Swap in a newly composed scene (after the build changed)."""
        self.stop()
        self._scene_path = scene_path
        self.start()

    def _loop(self) -> None:
        # Advance sim time to track the WALL CLOCK: each tick, step as many times as the
        # elapsed real time needs (capped, so a hiccup can't spiral). One step-per-iter
        # with sleep made motion judder under lock/GIL contention once physics/dynamics
        # were on — irregular step spacing shows up as stutter on a dynamic scene.
        ts = float(self._model.opt.timestep)
        sim_t = time.monotonic()
        while not self._stop.is_set():
            now = time.monotonic()
            n = int((now - sim_t) / ts)
            if n <= 0:
                time.sleep(0.0005)
                continue
            n = min(n, 25)                      # cap ~50ms of catch-up
            with self._lock:
                if self._data is None:
                    break
                for _ in range(n):
                    self._mj.mj_step(self._model, self._data)
            sim_t += n * ts
            if (now - sim_t) > 0.2:             # fell far behind -> resync, don't accumulate
                sim_t = now

    # ---- control / read ----
    def set_ctrl(self, actuator: str, value: float) -> None:
        with self._lock:
            if self._model is None:
                return
            self._data.ctrl[self._model.actuator(actuator).id] = value

    def qpos(self, joint: str) -> float:
        with self._lock:
            adr = self._model.jnt_qposadr[self._model.joint(joint).id]
            return float(self._data.qpos[adr])

    def qpos_many(self, joints) -> dict:
        """Read several joint positions under a SINGLE lock hold (vs. one lock per `qpos`). The 10 Hz
        telemetry read calls this once per beat instead of once per servo, cutting contention with the
        physics-step thread. Missing joints are omitted (caller falls back)."""
        out = {}
        with self._lock:
            if self._model is None:
                return out
            for j in joints:
                try:
                    adr = self._model.jnt_qposadr[self._model.joint(j).id]
                    out[j] = float(self._data.qpos[adr])
                except Exception:
                    pass
        return out

    def set_free_joint(self, joint: str, pos, quat=(1.0, 0.0, 0.0, 0.0)) -> None:
        """Teleport a free-joint body (the playground object) and zero its velocity —
        re-drops the object without disturbing the hand's pose."""
        with self._lock:
            if self._model is None:
                return
            try:
                jid = self._model.joint(joint).id
            except Exception:
                return
            qadr = self._model.jnt_qposadr[jid]
            vadr = self._model.jnt_dofadr[jid]
            self._data.qpos[qadr:qadr + 3] = pos
            self._data.qpos[qadr + 3:qadr + 7] = quat
            self._data.qvel[vadr:vadr + 6] = 0.0
            self._mj.mj_forward(self._model, self._data)

    # ---- offscreen render -> RGB frame for QML ----
    def render(self):
        """Return an HxWx3 uint8 numpy array of the current scene (or None until started)."""
        import mujoco  # lazy
        with self._lock:
            if self._model is None:
                return None
            if self._renderer is None:
                self._renderer = mujoco.Renderer(self._model, height=self._h, width=self._w)
            self._renderer.update_scene(self._data, self._cam, self._opt)
            return self._renderer.render()

    # ---- camera control (orbit / zoom from the viewport) ----
    def zoom(self, factor: float) -> None:
        with self._lock:
            if self._cam is not None:
                self._cam.distance = max(0.03, min(8.0, self._cam.distance * factor))

    def orbit(self, d_azimuth: float, d_elevation: float) -> None:
        with self._lock:
            if self._cam is not None:
                self._cam.azimuth = (self._cam.azimuth + d_azimuth) % 360.0
                self._cam.elevation = max(-89.0, min(89.0, self._cam.elevation + d_elevation))

    def pan(self, dx: float, dy: float) -> None:
        """Translate the camera target in the view plane (dx right, dy up; in fractions
        of the view — scaled by distance so pan speed feels constant at any zoom)."""
        import math
        with self._lock:
            if self._cam is None:
                return
            az = math.radians(self._cam.azimuth)
            el = math.radians(self._cam.elevation)
            # camera right + up vectors from azimuth/elevation (MuJoCo free camera)
            right = (-math.sin(az), math.cos(az), 0.0)
            up = (-math.cos(az) * math.sin(el), -math.sin(az) * math.sin(el), math.cos(el))
            s = self._cam.distance
            for i in range(3):
                self._cam.lookat[i] += (dx * right[i] + dy * up[i]) * s

    def pan_x(self, delta: float) -> None:
        """Translate the camera target along the WORLD X axis only (fraction of the view,
        distance-scaled like pan). Backs the ⌥/Option axis-locked pan."""
        with self._lock:
            if self._cam is not None:
                self._cam.lookat[0] += delta * self._cam.distance

    def reset_view(self) -> None:
        """Re-frame the model with the default free camera (undo orbit/zoom/pan)."""
        with self._lock:
            if self._cam is not None and self._model is not None:
                self._mj.mjv_defaultFreeCamera(self._model, self._cam)

    def camera_orientation(self) -> tuple[float, float]:
        """(azimuth, elevation) in degrees — feeds the live corner orientation gizmo."""
        with self._lock:
            if self._cam is None:
                return (0.0, 0.0)
            return (float(self._cam.azimuth), float(self._cam.elevation))
