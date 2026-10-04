#!/usr/bin/env python3
"""
Live ArUco pose tracking for a 2-marker finger rig (DICT_4X4_50).

Standalone terminal tool (a live-overlay window, no .app needed). It reuses handlab's CANONICAL
config so the numbers match the app:
  - camera intrinsics  ← ~/.handlab/intrinsics.yaml   (real calibration, per device)
  - marker edge sizes  ← ~/.handlab/vision.yaml        (your MEASURED mm)

Default rig (override sizes in vision.yaml):
  ID 1  : finger base      ID 2  : finger segment

For each marker it draws the 3D axes and, when both are visible, shows marker 2 relative to marker 1
(distance + rotation). It also tracks the SPREAD/joint angle by the inter-marker VECTOR direction —
which is immune to the near-flat square-marker AMBIGUITY that makes the orientation flip ~180°. When
a pose is ambiguous the overlay says so and the rotation readout is not to be trusted (use the vector).

macOS note: a plain cv2 camera grab is attributed to your TERMINAL app — grant Terminal/iTerm camera
access in System Settings → Privacy → Camera. Quit the handlab .app first (one process owns the cam).

Keys:  q/ESC quit · s save frame · a toggle axes · r reset the spread reference (zero the angle)

Requires: opencv-python >= 4.7 with GUI support (not the headless build from handlab's `vision`
extra — this tool opens a window), numpy, pyyaml
"""

import argparse
import os
import time

import cv2
import numpy as np

try:
    import yaml
except Exception:
    yaml = None

# ---------------------------------------------------------------- config ----

# Fallback edge lengths (m) — only used if vision.yaml has no entry for the id.
FALLBACK_SIZES_M = {1: 0.0195, 2: 0.0145}
DEFAULT_MARKER_SIZE_M = 0.0145

MARKER_LABELS = {1: "base (ID1)", 2: "finger (ID2)"}
MARKER_COLORS = {1: (0, 220, 255), 2: (255, 180, 0)}  # BGR

HANDLAB = os.path.expanduser("~/.handlab")
AMBIGUOUS_RATIO = 0.60   # 2nd solvePnP solution within 60% of the best reproj error → ambiguous


# ------------------------------------------------------------ aruco setup ---

def build_detector(dict_id=cv2.aruco.DICT_4X4_50):
    """detect(gray) -> (corners, ids, rejected); handles OpenCV <4.7 and >=4.7."""
    if hasattr(cv2.aruco, "getPredefinedDictionary"):
        dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
    else:
        dictionary = cv2.aruco.Dictionary_get(dict_id)

    if hasattr(cv2.aruco, "ArucoDetector"):
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        detector = cv2.aruco.ArucoDetector(dictionary, params)
        return detector.detectMarkers

    params = cv2.aruco.DetectorParameters_create()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return lambda gray: cv2.aruco.detectMarkers(gray, dictionary, parameters=params)


def marker_object_points(size_m):
    """Corners in the marker frame, matching detectMarkers order (TL, TR, BR, BL); Z out of the tag."""
    h = size_m / 2.0
    return np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]], dtype=np.float32)


def estimate_pose(corners, size_m, K, dist):
    """Single-marker pose. Returns (rvec, tvec, ambiguous). Uses solvePnPGeneric so we get BOTH
    IPPE-square solutions and can flag ambiguity (the near-flat 180° flip) instead of silently
    picking one — the fix for the app's `ambiguous:true` case."""
    obj = marker_object_points(size_m)
    img = np.asarray(corners, dtype=np.float32).reshape(4, 2)
    flags = getattr(cv2, "SOLVEPNP_IPPE_SQUARE", cv2.SOLVEPNP_ITERATIVE)
    try:
        n, rvecs, tvecs, err = cv2.solvePnPGeneric(obj, img, K, dist, flags=flags)
    except cv2.error:
        return None, None, False
    if n < 1:
        return None, None, False
    e = np.asarray(err).ravel()
    order = np.argsort(e)                       # best (lowest reproj error) first
    best = order[0]
    ambiguous = n >= 2 and (e[order[0]] / max(e[order[1]], 1e-9)) > AMBIGUOUS_RATIO
    return rvecs[best].reshape(3, 1), tvecs[best].reshape(3, 1), bool(ambiguous)


# ----------------------------------------------------------- calibration ----

def default_intrinsics(width, height, hfov_deg=60.0):
    f = (width / 2.0) / np.tan(np.deg2rad(hfov_deg) / 2.0)
    K = np.array([[f, 0, width / 2.0], [0, f, height / 2.0], [0, 0, 1.0]], dtype=np.float64)
    return K, np.zeros((5, 1), dtype=np.float64)


def load_handlab_intrinsics(path, cam_key, width, height):
    """~/.handlab/intrinsics.yaml is keyed by device id -> {width,height,camera_matrix,dist_coeffs}.
    Pick by --cam-key substring, else by matching WxH, else the single/first entry. Returns
    (K, dist, note) or None if the file/entry isn't usable."""
    if yaml is None or not os.path.exists(path):
        return None
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    entries = {k: v for k, v in data.items() if isinstance(v, dict) and "camera_matrix" in v}
    if not entries:
        return None
    pick = None
    if cam_key:
        pick = next((k for k in entries if cam_key in str(k)), None)
    if pick is None:
        pick = next((k for k, v in entries.items()
                     if v.get("width") == width and v.get("height") == height), None)
    if pick is None:
        pick = next(iter(entries))
    e = entries[pick]
    K = np.asarray(e["camera_matrix"], dtype=np.float64)
    dist = np.asarray(e.get("dist_coeffs", np.zeros(5)), dtype=np.float64).reshape(-1, 1)
    return K, dist, "handlab intrinsics [%s], rms=%.2f" % (str(pick)[:16], e.get("rms", float("nan")))


def load_marker_sizes(path):
    """~/.handlab/vision.yaml markers: {id: edge length in metres}. Returns {} on any problem."""
    if yaml is None or not os.path.exists(path):
        return {}
    try:
        with open(path) as f:
            data = yaml.safe_load(f) or {}
        return {int(k): float(v) for k, v in (data.get("markers") or {}).items()}
    except Exception:
        return {}


def load_calibration(path):
    """Explicit --calib: .npz (camera_matrix/K/mtx + dist_coeffs/dist), or FileStorage .yml/.xml."""
    ext = os.path.splitext(path)[1].lower()
    if ext == ".npz":
        data = np.load(path)
        K = next((data[k] for k in ("camera_matrix", "K", "mtx") if k in data), None)
        if K is None:
            raise KeyError("no camera_matrix/K/mtx in %s" % path)
        dist = next((data[k] for k in ("dist_coeffs", "dist", "distortion") if k in data),
                    np.zeros((5, 1)))
        return np.asarray(K, np.float64), np.asarray(dist, np.float64)
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise IOError("cannot open %s" % path)
    K = fs.getNode("camera_matrix").mat()
    dist = fs.getNode("dist_coeffs").mat()
    fs.release()
    if K is None:
        raise KeyError("no camera_matrix node in %s" % path)
    return K.astype(np.float64), (dist.astype(np.float64) if dist is not None else np.zeros((5, 1)))


# ------------------------------------------------------------- pose math ----

def to_matrix(rvec, tvec):
    T = np.eye(4)
    T[:3, :3] = cv2.Rodrigues(rvec)[0]
    T[:3, 3] = tvec.ravel()
    return T


def relative_pose(rvec_a, tvec_a, rvec_b, tvec_b):
    """Pose of B in A's frame."""
    return np.linalg.inv(to_matrix(rvec_a, tvec_a)) @ to_matrix(rvec_b, tvec_b)


def rotation_angle_deg(R):
    return np.degrees(np.arccos(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)))


def angle_between(u, v):
    nu, nv = np.linalg.norm(u), np.linalg.norm(v)
    if nu < 1e-9 or nv < 1e-9:
        return 0.0
    return float(np.degrees(np.arccos(np.clip(np.dot(u, v) / (nu * nv), -1.0, 1.0))))


# ------------------------------------------------------------------ main ----

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--camera", type=int, default=0, help="cv2 camera index (default 0)")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--calib", type=str, default=None, help="explicit .npz/.yml calibration file")
    ap.add_argument("--cam-key", type=str, default=None,
                    help="device-id substring to pick from ~/.handlab/intrinsics.yaml")
    ap.add_argument("--hfov", type=float, default=60.0, help="assumed HFOV when nothing calibrated")
    ap.add_argument("--print-hz", type=float, default=5.0, help="console print rate (0 = off)")
    args = ap.parse_args()

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        raise SystemExit("Could not open camera %d (quit the handlab .app; grant Terminal camera "
                         "access in System Settings → Privacy → Camera)." % args.camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    ok, frame = cap.read()
    if not ok:
        raise SystemExit("Camera opened but returned no frame.")
    h, w = frame.shape[:2]

    # calibration: explicit --calib > handlab intrinsics.yaml > rough guess
    if args.calib:
        K, dist = load_calibration(args.calib)
        calib_note = "calib: %s" % os.path.basename(args.calib)
    else:
        hl = load_handlab_intrinsics(os.path.join(HANDLAB, "intrinsics.yaml"), args.cam_key, w, h)
        if hl is not None:
            K, dist, calib_note = hl
        else:
            K, dist = default_intrinsics(w, h, args.hfov)
            calib_note = "UNCALIBRATED (~%.0f deg HFOV) — distances approximate" % args.hfov

    sizes = {**FALLBACK_SIZES_M, **load_marker_sizes(os.path.join(HANDLAB, "vision.yaml"))}
    detect = build_detector()
    print("Streaming %dx%d | %s | sizes(mm)=%s"
          % (w, h, calib_note, {k: round(v * 1000, 1) for k, v in sizes.items()}))
    print("q/ESC quit · s save · a axes · r reset spread reference")

    draw_axes = True
    last_print = 0.0
    t_prev = time.time()
    fps = 0.0
    ref_vec = None          # inter-marker vector at the last 'r' (spread zero); its angle = 0
    swept_max = 0.0

    while True:
        ok, frame = cap.read()
        if not ok:
            print("Frame grab failed.")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detect(gray)

        poses = {}
        amb = {}
        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            for c, i in zip(corners, ids.flatten()):
                mid = int(i)
                size = sizes.get(mid, DEFAULT_MARKER_SIZE_M)
                rvec, tvec, ambiguous = estimate_pose(c, size, K, dist)
                if rvec is None:
                    continue
                poses[mid] = (rvec, tvec)
                amb[mid] = ambiguous
                if draw_axes:
                    cv2.drawFrameAxes(frame, K, dist, rvec, tvec, size * 0.75, 2)
                px, py = c.reshape(4, 2).mean(axis=0).astype(int)
                color = MARKER_COLORS.get(mid, (0, 255, 0))
                x, y, z = tvec.ravel() * 1000.0
                tag = MARKER_LABELS.get(mid, "ID%d" % mid) + (" !AMB" if ambiguous else "")
                cv2.putText(frame, tag, (px - 40, py - 14), cv2.FONT_HERSHEY_SIMPLEX,
                            0.55, (0, 0, 255) if ambiguous else color, 2, cv2.LINE_AA)
                cv2.putText(frame, "x%+.0f y%+.0f z%.0f mm" % (x, y, z), (px - 60, py + 26),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)

        y0 = 30
        if 1 in poses and 2 in poses:
            T = relative_pose(*poses[1], *poses[2])
            t_mm = T[:3, 3] * 1000.0
            dist_mm = float(np.linalg.norm(t_mm))
            ang = rotation_angle_deg(T[:3, :3])
            ambiguous = amb.get(1) or amb.get(2)

            # SPREAD signal: angle of the inter-marker vector vs the 'r' reference — POSITION-based,
            # so it survives the orientation ambiguity. Move to one extreme, press r, move to the
            # other: `swept` reads the joint's angular travel.
            vec = np.array(t_mm)
            swept = angle_between(vec, ref_vec) if ref_vec is not None else 0.0
            swept_max = max(swept_max, swept)

            lines = [
                "ID2 rel ID1:  d=%.1f mm   rot=%.1f deg %s" % (dist_mm, ang, "(AMBIG-untrusted)" if ambiguous else ""),
                "  t = [%+.1f, %+.1f, %+.1f] mm" % (t_mm[0], t_mm[1], t_mm[2]),
                "  SPREAD (vector): %.1f deg   max swept: %.1f deg %s" % (
                    swept, swept_max, "" if ref_vec is not None else "(press r to zero)"),
            ]
            for k, ln in enumerate(lines):
                col = (0, 0, 255) if (k == 0 and ambiguous) else (0, 255, 120)
                cv2.putText(frame, ln, (12, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2, cv2.LINE_AA)
                y0 += 26

            now = time.time()
            if args.print_hz > 0 and now - last_print > 1.0 / args.print_hz:
                print("d=%6.1f mm  rot=%5.1f%s  spread=%5.1f (max %5.1f)"
                      % (dist_mm, ang, " AMB" if ambiguous else "", swept, swept_max))
                last_print = now
        else:
            missing = [MARKER_LABELS[m] for m in (1, 2) if m not in poses]
            cv2.putText(frame, "waiting for: " + ", ".join(missing), (12, y0),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2, cv2.LINE_AA)

        t_now = time.time()
        fps = 0.9 * fps + 0.1 / max(t_now - t_prev, 1e-6)
        t_prev = t_now
        cv2.putText(frame, "%.1f fps | %s" % (fps, calib_note), (12, frame.shape[0] - 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)

        cv2.imshow("ArUco finger tracker (DICT_4X4_50)", frame)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        if key == ord("a"):
            draw_axes = not draw_axes
        if key == ord("r"):
            if 1 in poses and 2 in poses:
                ref_vec = np.array(relative_pose(*poses[1], *poses[2])[:3, 3] * 1000.0)
                swept_max = 0.0
                print("spread reference zeroed")
        if key == ord("s"):
            fn = "capture_%d.png" % int(time.time())
            cv2.imwrite(fn, frame)
            print("saved", fn)

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
