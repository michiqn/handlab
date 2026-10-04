"""Demo Recorder — synchronized (robot-cam image, action, proprioception, object-pose) episodes for
end-to-end visuomotor imitation learning (ACT / Diffusion Policy).

**Qt-free, no threads, ZERO motion authority.** It only buffers data *pushed* into `on_step` and
writes it out; it never reads the driver, never issues a command, never touches the control/safety
path. The clock is the caller's (handlab samples on the 10 Hz telemetry beat — see bridge_record).

One episode = one folder, lossless + trivially loadable:
    demos/<build>/<task>/episode_<NNN>/
      frames/000000.png …     # robot-cam observation, one PNG per timestep
      episode.npz             # aligned arrays (length T), schema below
      meta.yaml               # self-describing (built by the caller, augmented here)

`episode.npz` arrays (all length T, aligned by index; the per-servo vectors have ONE COLUMN PER
SERVO of the running build, in `ServoMap.servos` order — 9 on claw3f, 12 on claw4f; `meta.yaml`
records the build, and a mid-episode build change is dropped rather than raggedly appended):
  t(f64) frame_idx(i64) action_goal(T,9 i64, logical ticks 2048=zero) present_pos(T,9 i64)
  current(T,9 i64 mA) dof_deg(T,9 f64) aruco_pose(T,7 f64, palm-base [xyz,quat], NaN if unseen)
  aruco_pose_cam(T,7 f64, camera-frame, NaN if uncalibrated/unseen) object_id(T i64, -1=none)
  online(T,9 i64, 1/0 per servo — 0 = dropped off the bus that beat → present_pos/current are the
         read_states sentinel, NOT real; mask these rows/columns when training)
  cam_frame_no(T i64, camera frame id — a REPEATED value across steps = the observation was held,
               i.e. the 10 Hz telemetry beat outran new-frame arrival; the staleness flag)
"""
from __future__ import annotations

import math
import re
import shutil
import time
from pathlib import Path

import numpy as np

_NAN7 = [float("nan")] * 7
_ARRAYS = ("t", "frame_idx", "action_goal", "present_pos", "current", "dof_deg",
           "aruco_pose", "aruco_pose_cam", "object_id", "online", "cam_frame_no")


def _rvec_to_quat(rvec) -> list[float]:
    """Axis-angle rotation vector -> quaternion [qx, qy, qz, qw]."""
    r = np.asarray(rvec, dtype=float)
    ang = float(np.linalg.norm(r))
    if ang < 1e-9:
        return [0.0, 0.0, 0.0, 1.0]
    axis = r / ang
    s = math.sin(ang / 2.0)
    return [float(axis[0] * s), float(axis[1] * s), float(axis[2] * s), float(math.cos(ang / 2.0))]


def aruco_from_dets(dets, object_ids) -> tuple[int, list[float], list[float]]:
    """Pick the primary visible object marker (first id in `object_ids` seen WITH a pose) and return
    (object_id, pose_in_base_7, pose_cam_7). Each pose = [x,y,z, qx,qy,qz,qw] or all-NaN when
    unavailable (cam needs calibration; in_base needs the reference marker co-visible). -1 if none."""
    by_id = {d.get("id"): d for d in (dets or [])}
    for oid in object_ids:
        d = by_id.get(oid)
        pose = d.get("pose") if d else None
        if not pose:
            continue
        cam7 = [float(v) for v in pose["tvec"]] + _rvec_to_quat(pose["rvec"])
        ib = pose.get("in_base")
        base7 = ([float(v) for v in ib["pos"]] + _rvec_to_quat(ib["rvec"])) if ib else list(_NAN7)
        return int(oid), base7, cam7
    return -1, list(_NAN7), list(_NAN7)


def _slug(s: str) -> str:
    s = re.sub(r"[^\w\-]+", "_", (s or "task").strip()).strip("_")
    return s or "task"


class DemoRecorder:
    def __init__(self, demos_root):
        self._root = Path(demos_root)
        self._active = False
        self._task_dir: Path | None = None
        self._dir: Path | None = None
        self._last_dir: Path | None = None

    # ---- state (read by the bridge for QML properties) ----
    @property
    def active(self) -> bool:
        return self._active

    @property
    def last_dir(self) -> Path | None:
        return self._last_dir

    @property
    def episode_count(self) -> int:
        d = self._task_dir
        return len([p for p in d.glob("episode_*") if p.is_dir()]) if d and d.exists() else 0

    # ---- lifecycle ----
    def start(self, build: str, task: str, meta: dict,
              object_ids=(10, 11, 12, 13, 14, 15), capture_wh=None) -> Path:
        """Begin an episode under demos/<build>/<task>/episode_<NNN>/. `meta` is the caller-built
        self-describing dict; it is augmented + written at stop. `capture_wh` (w,h) downscales the
        logged frame (default None = native)."""
        self._task_dir = self._root / _slug(build) / _slug(task)
        self._task_dir.mkdir(parents=True, exist_ok=True)
        n = 1 + max((_ep_index(p) for p in self._task_dir.glob("episode_*")), default=-1)
        self._dir = self._task_dir / f"episode_{n:03d}"
        (self._dir / "frames").mkdir(parents=True)
        self._object_ids = list(object_ids)
        self._capture_wh = tuple(capture_wh) if capture_wh else None
        self._meta = dict(meta or {})
        self._t0: float | None = None
        self._ncols: int | None = None
        self._buf: dict[str, list] = {k: [] for k in _ARRAYS}
        self._active = True
        return self._dir

    def on_step(self, frame_bgr, frame_ts: float, goal_ticks, present, current, dof_deg,
                online, dets, cam_frame_no: int) -> None:
        """Append ONE timestep. `frame_bgr` = numpy HxWx3 robot-cam frame (skipped if None — no
        image, no step). The per-servo vectors are in `ServoMap.servos` order (`online` = 1/0 per
        servo); their width is fixed by the first step of the episode. `dets`
        = latest ArUco detections (plain dicts); the object pose is extracted here. `cam_frame_no` =
        camera frame id (staleness flag). Never issues a command."""
        if not self._active or frame_bgr is None:
            return
        n = len(goal_ticks)                          # a mid-episode build change must not ragged the arrays
        if self._ncols is None:
            self._ncols = n
        elif n != self._ncols:
            return
        import cv2
        idx = len(self._buf["frame_idx"])
        img = cv2.resize(frame_bgr, self._capture_wh) if self._capture_wh else frame_bgr
        if not cv2.imwrite(str(self._dir / "frames" / f"{idx:06d}.png"), img):
            return                                   # write failed → no orphan npz row without a PNG
        if self._t0 is None:
            self._t0 = frame_ts
        oid, base7, cam7 = aruco_from_dets(dets, self._object_ids)
        b = self._buf
        b["t"].append(float(frame_ts - self._t0))
        b["frame_idx"].append(idx)
        b["action_goal"].append([int(x) for x in goal_ticks])
        b["present_pos"].append([int(x) for x in present])
        b["current"].append([int(x) for x in current])
        b["dof_deg"].append([float(x) for x in dof_deg])
        b["aruco_pose"].append(base7)
        b["aruco_pose_cam"].append(cam7)
        b["object_id"].append(int(oid))
        b["online"].append([int(x) for x in online])
        b["cam_frame_no"].append(int(cam_frame_no))

    def stop(self, outcome: str = "success", notes: str = "") -> Path | None:
        """Finalize: write episode.npz + meta.yaml (with `outcome` ∈ success/fail + `notes`). An empty
        episode (T==0) is discarded. Returns the episode path (or None)."""
        if not self._active:
            return None
        self._active = False
        T = len(self._buf["frame_idx"])
        if T == 0:
            shutil.rmtree(self._dir, ignore_errors=True)
            return None
        b = self._buf
        np.savez(
            self._dir / "episode.npz",
            t=np.asarray(b["t"], np.float64),
            frame_idx=np.asarray(b["frame_idx"], np.int64),
            action_goal=np.asarray(b["action_goal"], np.int64),
            present_pos=np.asarray(b["present_pos"], np.int64),
            current=np.asarray(b["current"], np.int64),
            dof_deg=np.asarray(b["dof_deg"], np.float64),
            aruco_pose=np.asarray(b["aruco_pose"], np.float64),
            aruco_pose_cam=np.asarray(b["aruco_pose_cam"], np.float64),
            object_id=np.asarray(b["object_id"], np.int64),
            online=np.asarray(b["online"], np.int64),
            cam_frame_no=np.asarray(b["cam_frame_no"], np.int64),
        )
        meta = dict(self._meta)
        meta["episode_len_T"] = int(T)
        meta["outcome"] = str(outcome)
        meta["notes"] = str(notes)
        meta["stop_wallclock"] = time.strftime("%Y-%m-%dT%H:%M:%S")
        import yaml
        with open(self._dir / "meta.yaml", "w") as f:
            yaml.safe_dump(meta, f, sort_keys=False, allow_unicode=True)
        self._last_dir = self._dir
        return self._dir

    def discard_last(self) -> bool:
        """Remove the in-progress episode if recording (abort the current take), else the most
        recent saved one. Prioritising the active dir avoids nuking a previous good episode when a
        fresh recording is running."""
        d = (self._dir if self._active else None) or self._last_dir
        if d and Path(d).exists():
            shutil.rmtree(d, ignore_errors=True)
            if d == self._last_dir:
                self._last_dir = None
            if self._active and d == self._dir:
                self._active = False
            return True
        return False


def _ep_index(p: Path) -> int:
    m = re.search(r"episode_(\d+)", p.name)
    return int(m.group(1)) if m else -1
