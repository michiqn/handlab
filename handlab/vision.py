"""vision.py — optional ArUco marker detection for the webcam feed (agent vision-in-the-loop).

Guarded exactly like camera.py: OpenCV is an OPTIONAL dep (the `[vision]` extra). If `cv2` is
missing, `AVAILABLE` is False and the app runs unchanged (no detection, no overlay). The heavy
`MarkerDetector.detect()` is pure numpy + cv2 (no Qt), so it runs OFF the Qt thread (a QThreadPool
worker in camera.py) and is unit-testable without a camera or a QGuiApplication. Only the two
QImage helpers touch Qt — and Qt is always present (PySide6 is a core dep).

Coordinate/pose note: `detect()` returns 2D pixel data always, and adds a 6-DoF `pose` per marker
ONLY when the detector was given that marker's edge length + camera intrinsics (see
calibrate_intrinsics.py and MarkerDetector below). pose rvec/tvec are CAMERA-frame; when a
reference marker (vision.yaml `reference:`, default id 0 = palm base) is co-visible with an
unambiguous pose, every other pose additionally carries `in_base` = {pos [m], rvec} in the
BASE/hand frame (annotate_in_base). The reference itself is flagged `is_reference`; a
near-fronto-parallel pose may be flagged `ambiguous` (planar-pose flip risk — don't trust its
ORIENTATION; position/distance stay usable).
"""
from __future__ import annotations

from pathlib import Path

import numpy as np

try:
    import cv2
    AVAILABLE = True
except Exception:                     # opencv not installed → detection disabled, app unaffected
    cv2 = None
    AVAILABLE = False

from PySide6.QtCore import QPointF
from PySide6.QtGui import QColor, QImage, QPainter, QPen, QPolygonF

DEFAULT_DICT = "DICT_4X4_50"          # small + robust; print with cv2.aruco.generateImageMarker

# User-state vision config (same ~/.handlab home the agent discovery uses, so it works from the
# .app too). TWO files by design — the review showed a shared read-modify-write file either
# crashes on hand-edits, wipes data on parse errors, or strips the human's comments:
#   vision.yaml     HAND-maintained, code only READS it. markers: {id: edge length METERS}
#                   (measured on the mounted markers, set in ~/.handlab/vision.yaml)
#   intrinsics.yaml MACHINE-owned, written by tools/calibrate_intrinsics.py. Flat mapping
#                   {device_id: {width, height, camera_matrix, dist_coeffs, rms}}.
VISION_CONFIG = Path.home() / ".handlab" / "vision.yaml"
INTRINSICS_FILE = Path.home() / ".handlab" / "intrinsics.yaml"


def _read_yaml_dict(path: Path) -> dict:
    """A yaml file as a dict — {} when missing, unreadable, or not a mapping (tolerant read path)."""
    import yaml
    try:
        d = yaml.safe_load(path.read_text())
        return d if isinstance(d, dict) else {}
    except Exception:
        return {}


def load_vision_config() -> dict:
    return _read_yaml_dict(VISION_CONFIG)


def save_intrinsics(device_id: str, width: int, height: int,
                    camera_matrix, dist_coeffs, rms: float) -> None:
    """Merge one camera's intrinsics into the MACHINE-owned intrinsics.yaml. The hand-maintained
    vision.yaml is never touched. An existing-but-unparsable file is backed up to .bak instead of
    being silently overwritten (never destroy another camera's calibration on a parse error)."""
    import yaml
    cur: dict = {}
    if INTRINSICS_FILE.exists():
        try:
            loaded = yaml.safe_load(INTRINSICS_FILE.read_text())
            if isinstance(loaded, dict):
                cur = loaded
            elif loaded is not None:
                raise ValueError(f"not a mapping: {type(loaded).__name__}")
        except Exception as e:
            bak = INTRINSICS_FILE.with_suffix(".yaml.bak")
            INTRINSICS_FILE.rename(bak)
            print(f"[handlab] {INTRINSICS_FILE} unlesbar ({e}) — gesichert nach {bak}, schreibe neu.")
    cur[str(device_id)] = {
        "width": int(width), "height": int(height),
        "camera_matrix": [[float(v) for v in row] for row in np.asarray(camera_matrix)],
        "dist_coeffs": [float(v) for v in np.asarray(dist_coeffs).flatten()],
        "rms": float(rms),
    }
    INTRINSICS_FILE.parent.mkdir(parents=True, exist_ok=True)
    INTRINSICS_FILE.write_text("# machine-owned — written by tools/calibrate_intrinsics.py\n"
                               + yaml.safe_dump(cur, sort_keys=False))


def reference_id(cfg: dict | None = None) -> int:
    """The marker id that DEFINES the hand/base frame (vision.yaml `reference:`, default 0 — the
    palm-base marker). Every other marker's pose gets expressed relative to it when co-visible."""
    cfg = load_vision_config() if cfg is None else cfg
    try:
        return int(cfg.get("reference", 0))
    except (TypeError, ValueError):
        return 0


def _pose_to_T(rvec, tvec) -> np.ndarray:
    """(rvec, tvec) -> 4x4 rigid transform T_cam_marker (marker frame -> camera frame)."""
    T = np.eye(4)
    T[:3, :3], _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64))
    T[:3, 3] = np.asarray(tvec, dtype=np.float64).flatten()
    return T


def _inv_T(T: np.ndarray) -> np.ndarray:
    """Rigid-transform inverse (R^T, -R^T t) — cheaper + numerically cleaner than np.linalg.inv."""
    out = np.eye(4)
    R = T[:3, :3]
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ T[:3, 3]
    return out


def annotate_in_base(dets: list[dict], ref_id: int) -> None:
    """Hand-eye without a separate calibration: the REFERENCE marker (palm base, rigidly mounted)
    defines the base frame in every frame it is visible. For each other marker with a pose, add
        pose["in_base"] = {"pos": [x,y,z] m, "rvec": [rx,ry,rz]}
    = its position/orientation in the BASE frame: T_base_m = inv(T_cam_base) @ T_cam_m. This is
    what makes measurements camera-placement-independent — moving the laptop moves both markers
    together, the relative transform stays put. No-op when the reference isn't co-visible with a
    pose (consumers must treat in_base as optional). Mutates dets in place."""
    refs = [d for d in dets if d["id"] == ref_id and "pose" in d]
    if len(refs) != 1:
        return       # no ref, or DUPLICATE refs (false positive/reflection/second print) — an
        #              arbitrary pick would silently poison every relative pose. No base frame.
    ref = refs[0]
    if ref["pose"].get("ambiguous"):
        return       # a flipped reference orientation would poison EVERY relative pose — skip
    T_base_cam = _inv_T(_pose_to_T(ref["pose"]["rvec"], ref["pose"]["tvec"]))
    ref["pose"]["is_reference"] = True
    for d in dets:
        if d is ref or "pose" not in d:
            continue
        T_base_m = T_base_cam @ _pose_to_T(d["pose"]["rvec"], d["pose"]["tvec"])
        rvec_rel, _ = cv2.Rodrigues(T_base_m[:3, :3])
        d["pose"]["in_base"] = {"pos": [float(v) for v in T_base_m[:3, 3]],
                                "rvec": [float(v) for v in rvec_rel.flatten()]}


def marker_lengths(cfg: dict | None = None) -> dict[int, float]:
    """{marker id -> edge length m} from vision.yaml's markers section. Tolerates str keys; only
    positive finite lengths count (a negative/zero length would produce a garbage pose)."""
    import math
    cfg = load_vision_config() if cfg is None else cfg
    out = {}
    for k, v in (cfg.get("markers") or {}).items():
        try:
            v = float(v)
            if v > 0 and math.isfinite(v):
                out[int(k)] = v
        except (TypeError, ValueError):
            continue
    return out


def intrinsics_for(device_id: str, width: int, height: int):
    """(camera_matrix, dist_coeffs) for this camera at THIS frame size, or None if uncalibrated.
    fx/fy/cx/cy scale linearly with resolution, so a calibration at another size is rescaled —
    but only when the aspect ratio matches (a different aspect means a crop, not a scale)."""
    entry = _read_yaml_dict(INTRINSICS_FILE).get(str(device_id))
    if not isinstance(entry, dict):
        return None
    try:
        k = np.array(entry["camera_matrix"], dtype=np.float64)
        dist = np.array(entry["dist_coeffs"], dtype=np.float64)
        cw, ch = int(entry["width"]), int(entry["height"])
    except (KeyError, TypeError, ValueError):
        return None
    if cw <= 0 or ch <= 0 or width <= 0 or height <= 0:
        return None
    if abs((cw / ch) - (width / height)) > 0.02 * (cw / ch):
        return None                              # aspect mismatch -> rescale would be wrong
    s = width / cw
    if abs(s - 1.0) > 1e-9:
        k = k.copy()
        k[0, 0] *= s; k[1, 1] *= s; k[0, 2] *= s; k[1, 2] *= s
    return k, dist


def make_detector_for(device_id: str, width: int, height: int) -> "MarkerDetector":
    """The camera's MarkerDetector: calibrated (per-id marker lengths -> 6-DoF poses, plus
    base-frame poses relative to the reference marker) when intrinsics.yaml knows this device,
    else a plain 2D detector."""
    cfg = load_vision_config()
    lengths = marker_lengths(cfg)
    intr = intrinsics_for(device_id, width, height)
    if intr is None:
        return MarkerDetector(marker_lengths=lengths)
    k, dist = intr
    return MarkerDetector(marker_lengths=lengths, camera_matrix=k, dist_coeffs=dist,
                          reference_id=reference_id(cfg))


def qimage_to_bgr(img: QImage) -> np.ndarray:
    """QImage → contiguous HxWx3 uint8 BGR (cv2's channel order), honoring the row-stride padding
    (RGB888 rows are padded to a 4-byte boundary, so bytesPerLine ≥ width*3)."""
    img = img.convertToFormat(QImage.Format.Format_RGB888)
    w, h, bpl = img.width(), img.height(), img.bytesPerLine()
    buf = np.frombuffer(img.constBits(), dtype=np.uint8)[: bpl * h].reshape(h, bpl)
    rgb = buf[:, : w * 3].reshape(h, w, 3)
    return np.ascontiguousarray(rgb[:, :, ::-1])          # RGB → BGR, detached


_DICTS: dict = {}


def _dictionary(name: str):
    d = _DICTS.get(name)
    if d is None:
        d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))
        _DICTS[name] = d
    return d


class MarkerDetector:
    """Wraps a cv2 ArucoDetector. Pose is per marker: detect() returns each marker's 6-DoF pose
    (camera frame) when `camera_matrix` is set AND the marker's edge length is known — either from
    `marker_lengths` ({id: meters}, the mounted-marker registry) or the `marker_length_m` fallback
    for uniform setups/tests. `estimatePoseSingleMarkers` was removed in OpenCV 4.7, so we solve
    the square directly (SOLVEPNP_IPPE_SQUARE)."""

    def __init__(self, dictionary: str = DEFAULT_DICT, marker_length_m: float | None = None,
                 camera_matrix: np.ndarray | None = None, dist_coeffs: np.ndarray | None = None,
                 marker_lengths: dict[int, float] | None = None,
                 reference_id: int | None = None):
        if not AVAILABLE:
            raise RuntimeError("opencv not installed — install the [vision] extra")
        params = cv2.aruco.DetectorParameters()
        # subpixel corners: sharper squares -> markedly less planar-pose (IPPE) ambiguity
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self._detector = cv2.aruco.ArucoDetector(_dictionary(dictionary), params)
        self.marker_length_m = marker_length_m
        self.marker_lengths = dict(marker_lengths or {})
        self.camera_matrix = camera_matrix
        self.dist_coeffs = (dist_coeffs if dist_coeffs is not None
                            else (np.zeros(5) if camera_matrix is not None else None))
        self.reference_id = reference_id      # base-frame marker -> detect() adds in_base poses

    @property
    def has_pose(self) -> bool:
        return self.camera_matrix is not None and (
            bool(self.marker_lengths) or self.marker_length_m is not None)

    def _length_for(self, marker_id: int) -> float | None:
        return self.marker_lengths.get(int(marker_id), self.marker_length_m)

    def detect(self, bgr: np.ndarray) -> list[dict]:
        """Runs on a worker thread. Returns a list of
        {id, corners[4][xy], center[xy],
         pose?{rvec, tvec, distance_m, ambiguous?, is_reference?, in_base?{pos, rvec}}}."""
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detector.detectMarkers(gray)
        if ids is None:
            return []
        out = []
        for c, mid in zip(corners, ids.flatten()):
            pts = c.reshape(-1, 2)                        # 4×2 in TL,TR,BR,BL order
            det = {"id": int(mid),
                   "corners": [[float(x), float(y)] for x, y in pts],
                   "center": [float(pts[:, 0].mean()), float(pts[:, 1].mean())]}
            length = self._length_for(mid)
            if self.camera_matrix is not None and length:
                pose = self._pose(pts, length)
                if pose is not None:
                    det["pose"] = pose
            out.append(det)
        if self.reference_id is not None and self.camera_matrix is not None:
            annotate_in_base(out, self.reference_id)   # base-frame poses when the ref is co-visible
        return out

    def _pose(self, pts: np.ndarray, length_m: float) -> dict | None:
        """Planar poses are AMBIGUOUS near fronto-parallel (IPPE returns two ~equally good
        solutions; noise picks the branch -> the orientation can flip frame to frame). We take
        the better solution and FLAG the pose when the runner-up is nearly as good — consumers
        (incl. annotate_in_base) must not trust an ambiguous ORIENTATION; distance/position stay
        usable."""
        s = length_m / 2.0
        obj = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], np.float32)  # matches corner order
        n, rvecs, tvecs, errs = cv2.solvePnPGeneric(
            obj, pts.astype(np.float32), self.camera_matrix, self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not n:
            return None
        t = tvecs[0].flatten()
        pose = {"rvec": [float(v) for v in rvecs[0].flatten()],
                "tvec": [float(v) for v in t],
                "distance_m": float(np.linalg.norm(t))}
        if n >= 2:
            e0, e1 = float(np.asarray(errs).flatten()[0]), float(np.asarray(errs).flatten()[1])
            if e0 > 0.6 * max(e1, 1e-9):          # runner-up nearly as good -> orientation unsafe
                pose["ambiguous"] = True
        return pose


def draw_overlay(img: QImage, dets: list[dict]) -> QImage:
    """Qt-only (no cv2): a NEW QImage with each marker outlined + its id labeled. The raw frame is
    left untouched so the model's snapshot stays clean — only the human preview gets annotated."""
    if not dets:
        return img
    out = img.convertToFormat(QImage.Format.Format_RGB888).copy()
    p = QPainter(out)
    p.setRenderHint(QPainter.RenderHint.Antialiasing, True)
    pen = QPen(QColor("#38F5B0"))
    pen.setWidth(max(2, out.width() // 400))
    p.setPen(pen)
    for d in dets:
        p.drawPolygon(QPolygonF([QPointF(x, y) for x, y in d["corners"]]))
        cx, cy = d["center"]
        p.drawText(QPointF(cx + 5, cy - 5), str(d["id"]))
    p.end()
    return out
