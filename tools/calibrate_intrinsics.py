#!/usr/bin/env python
"""handlab camera-intrinsics calibration — turns marker pixels into real 6-DoF poses in metres.

Runs against the RUNNING app (the GUI keeps the camera — one process per device): frames come over
the token-gated HTTP API (`/snapshot.jpg`, full resolution), not from a capture of its own.
So: no macOS camera-permission dance, no device conflict, and it works for WHICHEVER camera is
selected in the Agent tab (run it once per camera).

STEPS (usable without looking at the screen):
  1. once:  python tools/calibrate_intrinsics.py --make-board
     → print docs/markers/charuco_5x5.png at 100 % (300 dpi embedded), glue it to cardboard.
  2. Start the app → Agent tab → camera Start.
  3. python tools/calibrate_intrinsics.py --square-mm <MEASURED>
     (measure one chessboard square of the PRINTOUT with a ruler — mandatory, dpi often lies!)
  4. Move the board slowly in front of the camera: near/far, left/right/up/down, slightly tilted.
     The tool collects enough distinct views on its own and then computes.

Result → ~/.handlab/intrinsics.yaml (per camera device id; the app loads it automatically on the
next camera start → `calibrated: true` in /detections, poses + distance_m per marker).

Board: ChArUco 7×5, DICT_5X5_100 — deliberately a DIFFERENT dictionary than the 4X4 rig markers,
so the mounted IDs 0/1 never interfere during calibration.
"""
from __future__ import annotations

import argparse
import json
import struct
import sys
import time
import urllib.request
import zlib
from pathlib import Path

import numpy as np

try:
    import cv2
except ImportError:
    raise SystemExit("opencv missing — install with:  pip install -e '.[vision]'")

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from handlab import vision  # noqa: E402  (save_intrinsics / VISION_CONFIG)

DISCOVERY = Path.home() / ".handlab" / "agent.json"
BOARD_PNG = Path(__file__).resolve().parent.parent / "docs" / "markers" / "charuco_5x5.png"
SQUARES_X, SQUARES_Y = 7, 5          # chessboard squares (columns x rows)
MARKER_RATIO = 0.74                  # marker edge = 0.74 * square edge
DICT_NAME = "DICT_5X5_100"


def _board(square_m: float):
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, DICT_NAME))
    return cv2.aruco.CharucoBoard((SQUARES_X, SQUARES_Y), square_m, square_m * MARKER_RATIO, d)


# ─────────────────────────── board printing ───────────────────────────
def _embed_dpi(path: Path, dpi: int) -> None:
    """Embed a PNG pHYs chunk, otherwise macOS prints at 72 dpi, ~4x too large (learned 13.07)."""
    ppm = round(dpi / 0.0254)
    data = struct.pack(">IIB", ppm, ppm, 1)
    chunk = struct.pack(">I", len(data)) + b"pHYs" + data
    chunk += struct.pack(">I", zlib.crc32(b"pHYs" + data) & 0xFFFFFFFF)
    raw = path.read_bytes()
    ihdr_end = raw.index(b"IHDR") + 4 + 13 + 4
    path.write_bytes(raw[:ihdr_end] + chunk + raw[ihdr_end:])


def make_board(square_mm: float = 25.0, dpi: int = 300) -> None:
    mm = dpi / 25.4
    w, h = round(SQUARES_X * square_mm * mm), round(SQUARES_Y * square_mm * mm)
    margin = round(10 * mm)
    img = _board(square_mm / 1000.0).generateImage((w, h), marginSize=0)
    page = np.full((h + 2 * margin + round(8 * mm), w + 2 * margin), 255, np.uint8)
    page[margin: margin + h, margin: margin + w] = img
    cv2.putText(page, f"handlab ChArUco {SQUARES_X}x{SQUARES_Y} {DICT_NAME} - nominal "
                      f"{square_mm:.0f} mm/square - print at 100%, then MEASURE",
                (margin, page.shape[0] - round(3 * mm)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 0, 1,
                cv2.LINE_AA)
    BOARD_PNG.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(BOARD_PNG), page)
    _embed_dpi(BOARD_PNG, dpi)
    # self-check: the rendered board must detect itself
    det = cv2.aruco.CharucoDetector(_board(square_mm / 1000.0))
    corners, ids, _, _ = det.detectBoard(page)
    n = 0 if ids is None else len(ids)
    print(f"✓ {BOARD_PNG}  ({page.shape[1]}x{page.shape[0]} px, {dpi} dpi)")
    print(f"  self-check: {n} ChArUco corners detected "
          f"({'PASS' if n >= 12 else 'FAIL — board unusable!'})")


# ─────────────────────────── app HTTP client ───────────────────────────
def _discovery() -> tuple[str, int, str]:
    try:
        d = json.loads(DISCOVERY.read_text())
        return d["host"], int(d["port"]), d["token"]
    except Exception as e:
        raise SystemExit(f"App not running? ~/.handlab/agent.json unreadable ({e}). "
                         f"Start handlab first.")


def _get(path: str) -> bytes:
    host, port, token = _discovery()
    req = urllib.request.Request(f"http://{host}:{port}{path}",
                                 headers={"X-Handlab-Token": token})
    with urllib.request.urlopen(req, timeout=5) as r:
        return r.read()


def fetch_frame() -> np.ndarray | None:
    """Current RAW camera frame at full resolution (max=4096 ⇒ no downscale at 720p)."""
    raw = _get("/snapshot.jpg?max=4096")
    if raw[:3] != b"\xff\xd8\xff":
        return None
    return cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)


def camera_state() -> dict:
    return json.loads(_get("/detections"))


# ─────────────────────────── capture + calibrate ───────────────────────────
def _bucket(corners: np.ndarray, w: int, h: int) -> tuple:
    """Diversity bucket of a view: 3x3 position of the centroid x 3 size classes.
    Max. 2 views per bucket ⇒ forces near/far + all image regions instead of 15x the same pose."""
    pts = corners.reshape(-1, 2)
    cx, cy = pts[:, 0].mean() / w, pts[:, 1].mean() / h
    span = (pts[:, 0].max() - pts[:, 0].min()) / w
    size = 0 if span < 0.25 else (1 if span < 0.5 else 2)
    return (min(int(cx * 3), 2), min(int(cy * 3), 2), size)


def accept_view(buckets: dict, corners, w: int, h: int, min_corners: int = 12,
                per_bucket: int = 2) -> bool:
    """Pure acceptance logic (unit-testable): enough corners + bucket not yet full."""
    if corners is None or len(corners) < min_corners:
        return False
    b = _bucket(corners, w, h)
    if buckets.get(b, 0) >= per_bucket:
        return False
    buckets[b] = buckets.get(b, 0) + 1
    return True


def _assert_camera_serving(st: dict) -> None:
    """Hard-stop the two ways /snapshot.jpg does NOT serve the camera — otherwise you accidentally
    calibrate the 900x720 TWIN render (review finding, reproduced for real)."""
    if not st.get("active"):
        raise SystemExit("Camera is OFF — app → Agent tab → Start, then retry.")
    if st.get("snapshot_source") == "twin":
        raise SystemExit("Snapshot source is TWIN — Agent tab → snapshot: choose 'Camera', "
                         "then retry (otherwise the sim render would be calibrated).")


def run_calibration(square_mm: float, views_target: int, timeout_s: float) -> None:
    st = camera_state()
    _assert_camera_serving(st)
    device = st.get("device", "")
    board = _board(square_mm / 1000.0)
    detector = cv2.aruco.CharucoDetector(board)

    print(f"Calibrating device: {device or '<unknown>'}  (square = {square_mm} mm measured)")
    print(f"→ Move the board slowly: near/far, all image corners, slight tilt. Target: "
          f"{views_target} distinct views.\n")
    buckets: dict = {}
    all_corners, all_ids = [], []
    size = None                          # fixed by the FIRST ACCEPTED view
    t0 = time.monotonic()
    last_report = 0
    while len(all_corners) < views_target:
        if time.monotonic() - t0 > timeout_s:
            if len(all_corners) >= 8:
                print(f"\n⚠ Timeout — computing with {len(all_corners)} views (better than nothing).")
                break
            raise SystemExit(f"\n✗ Timeout after {timeout_s:.0f}s with only {len(all_corners)} "
                             f"views. Board in frame? Lighting ok? Square size printed correctly?")
        try:                             # ONE network hiccup must not throw the session away
            if len(all_corners) % 3 == 0:            # the camera could stop/switch mid-run
                _assert_camera_serving(camera_state())
            frame = fetch_frame()
        except SystemExit:
            raise
        except Exception as e:           # urllib.error.*, TimeoutError, OSError, ...
            print(f"  … HTTP hiccup ({e}) — continuing")
            time.sleep(0.5)
            continue
        if frame is None:
            time.sleep(0.3)
            continue
        fsize = (frame.shape[1], frame.shape[0])
        if size is not None and fsize != size:
            print(f"  ⚠ frame size changed {size}→{fsize} (camera switched?) — frame dropped")
            time.sleep(0.3)
            continue
        corners, ids, _, _ = detector.detectBoard(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        if ids is not None and accept_view(buckets, corners, fsize[0], fsize[1]):
            size = fsize                 # size counts ONLY from accepted camera views
            all_corners.append(corners)
            all_ids.append(ids)
            print(f"  ✓ view {len(all_corners)}/{views_target} "
                  f"({len(ids)} corners, buckets: {len(buckets)})")
        elif time.monotonic() - last_report > 5:
            last_report = time.monotonic()
            n = 0 if ids is None else len(ids)
            print(f"  … waiting ({n} corners visible; view too similar or too few)")
        time.sleep(0.35)

    # views → object/image points (modern path: Board.matchImagePoints + calibrateCamera)
    obj_all, img_all = [], []
    for corners, ids in zip(all_corners, all_ids):
        obj, img = board.matchImagePoints(corners, ids)
        if obj is not None and len(obj) >= 6:
            obj_all.append(obj)
            img_all.append(img)
    if len(obj_all) < 8:
        raise SystemExit(f"✗ only {len(obj_all)} usable views — retry with more variation.")
    print(f"\nComputing calibration from {len(obj_all)} views …")
    rms, k, dist, _, _ = cv2.calibrateCamera(obj_all, img_all, size, None, None)
    vision.save_intrinsics(device, size[0], size[1], k, dist, rms)
    print(f"✓ RMS reprojection error: {rms:.3f} px  "
          f"({'very good' if rms < 0.7 else 'ok' if rms < 1.5 else '⚠ high — board flat? re-measure?'})")
    print(f"✓ fx={k[0][0]:.1f} fy={k[1][1]:.1f} cx={k[0][2]:.1f} cy={k[1][2]:.1f} @ {size[0]}x{size[1]}")
    print(f"✓ saved: {vision.INTRINSICS_FILE}  [{device}]")
    print("\n→ In the app: camera Stop + Start (loads the intrinsics) → /detections shows "
          "calibrated:true + poses in metres.")


def main() -> None:
    ap = argparse.ArgumentParser(description="handlab camera intrinsics (ChArUco)")
    ap.add_argument("--make-board", action="store_true", help="write the printable board and exit")
    ap.add_argument("--square-mm", type=float, default=None,
                    help="MEASURED edge length of one chessboard square on the printout (mm)")
    ap.add_argument("--views", type=int, default=15, help="target number of distinct views")
    ap.add_argument("--timeout", type=float, default=180.0, help="capture timeout (s)")
    args = ap.parse_args()
    if args.make_board:
        make_board()
        return
    if args.square_mm is None:
        raise SystemExit("--square-mm missing: measure one square of the PRINTOUT with a ruler "
                         "(mandatory — printers like to scale). Board not printed yet? "
                         "Run --make-board first.")
    run_calibration(args.square_mm, args.views, args.timeout)


if __name__ == "__main__":
    main()
