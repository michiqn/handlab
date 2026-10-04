"""ArUco detection (the [vision] extra): the QImage→BGR conversion, MarkerDetector.detect on a
synthetic marker, optional pose, and the preview overlay. The whole module skips when opencv isn't
installed. Pure cv2/numpy where possible; the QImage helpers run under a QCoreApplication like
test_camera (the real app always has a QGuiApplication, so overlay text renders there)."""
import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")            # skip until `pip install -e '.[vision]'`

from PySide6.QtCore import QCoreApplication
from PySide6.QtGui import QImage

from handlab import vision


def _qapp():
    return QCoreApplication.instance() or QCoreApplication([])


def _marker_image(marker_id=7, dict_name="DICT_4X4_50", px=240, border=60):
    """A white canvas with one ArUco marker centered — what the detector should find. Returns BGR."""
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
    m = cv2.aruco.generateImageMarker(d, marker_id, px)         # px×px 0/255 grayscale
    canvas = np.full((px + 2 * border, px + 2 * border), 255, np.uint8)
    canvas[border:border + px, border:border + px] = m
    return cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)


def test_available_for_this_run():
    assert vision.AVAILABLE is True                            # opencv installed → detection on


def test_detect_finds_the_marker():
    det = vision.MarkerDetector(dictionary="DICT_4X4_50")
    img = _marker_image(marker_id=7)
    dets = det.detect(img)
    assert len(dets) == 1
    d = dets[0]
    assert d["id"] == 7
    assert len(d["corners"]) == 4 and all(len(c) == 2 for c in d["corners"])
    assert "pose" not in d                                     # no intrinsics → 2D only
    h, w = img.shape[:2]
    assert abs(d["center"][0] - w / 2) < 20 and abs(d["center"][1] - h / 2) < 20


def test_detect_empty_scene_returns_nothing():
    det = vision.MarkerDetector()
    assert det.detect(np.full((120, 160, 3), 127, np.uint8)) == []


def test_pose_when_intrinsics_given():
    K = np.array([[400.0, 0, 180], [0, 400.0, 180], [0, 0, 1]])
    det = vision.MarkerDetector(marker_length_m=0.05, camera_matrix=K, dist_coeffs=np.zeros(5))
    dets = det.detect(_marker_image(marker_id=3))
    assert len(dets) == 1 and "pose" in dets[0]
    p = dets[0]["pose"]
    assert len(p["tvec"]) == 3 and len(p["rvec"]) == 3 and p["distance_m"] > 0


def test_per_id_marker_lengths_gate_the_pose():
    """Pose only for markers whose edge length is known — and the distance must match the pinhole
    prediction (z = fx * L / pixel_size) within tolerance, proving the length is actually used."""
    K = np.array([[400.0, 0, 180], [0, 400.0, 180], [0, 0, 1]])
    det = vision.MarkerDetector(marker_lengths={3: 0.05}, camera_matrix=K)
    with_len = det.detect(_marker_image(marker_id=3))
    without = det.detect(_marker_image(marker_id=9))
    assert "pose" in with_len[0]
    assert "pose" not in without[0]                       # id 9 has no length -> 2D only
    expected_z = 400.0 * 0.05 / 240                       # fx * L / marker_px
    assert abs(with_len[0]["pose"]["distance_m"] - expected_z) < 0.2 * expected_z


def test_intrinsics_config_roundtrip_and_rescale(tmp_path, monkeypatch):
    monkeypatch.setattr(vision, "VISION_CONFIG", tmp_path / "vision.yaml")
    monkeypatch.setattr(vision, "INTRINSICS_FILE", tmp_path / "intrinsics.yaml")
    K = [[800.0, 0.0, 640.0], [0.0, 800.0, 360.0], [0.0, 0.0, 1.0]]
    vision.save_intrinsics("CAM-A", 1280, 720, np.array(K), np.zeros(5), rms=0.5)
    # same resolution -> unchanged
    k1, d1 = vision.intrinsics_for("CAM-A", 1280, 720)
    assert abs(k1[0, 0] - 800.0) < 1e-9 and d1.shape[0] == 5
    # half resolution (same aspect) -> fx/cx scale linearly
    k2, _ = vision.intrinsics_for("CAM-A", 640, 360)
    assert abs(k2[0, 0] - 400.0) < 1e-9 and abs(k2[0, 2] - 320.0) < 1e-9
    # aspect mismatch (crop, not scale) or unknown device -> None
    assert vision.intrinsics_for("CAM-A", 1280, 1024) is None
    assert vision.intrinsics_for("CAM-B", 1280, 720) is None
    # a second save merges (CAM-A survives); the HAND-maintained vision.yaml is never touched
    (tmp_path / "vision.yaml").write_text("# hand-written comment\nmarkers:\n  0: 0.012\n  '1': 0.008\n")
    vision.save_intrinsics("CAM-B", 640, 480, np.eye(3), np.zeros(5), rms=1.0)
    assert vision.marker_lengths() == {0: 0.012, 1: 0.008}     # str keys tolerated
    assert vision.intrinsics_for("CAM-A", 1280, 720) is not None
    assert vision.intrinsics_for("CAM-B", 640, 480) is not None
    assert "# hand-written comment" in (tmp_path / "vision.yaml").read_text()


def test_save_intrinsics_survives_corrupt_state(tmp_path, monkeypatch):
    """The review's data-loss findings: an unparsable intrinsics file is backed up (never silently
    wiped), a None/list-shaped file doesn't crash the save, and negative marker lengths are ignored."""
    monkeypatch.setattr(vision, "VISION_CONFIG", tmp_path / "vision.yaml")
    monkeypatch.setattr(vision, "INTRINSICS_FILE", tmp_path / "intrinsics.yaml")
    (tmp_path / "intrinsics.yaml").write_text("{{{ not yaml")
    vision.save_intrinsics("CAM-A", 1280, 720, np.eye(3), np.zeros(5), rms=0.5)
    assert (tmp_path / "intrinsics.yaml.bak").exists()          # corrupt file preserved, not wiped
    assert vision.intrinsics_for("CAM-A", 1280, 720) is not None
    # a yaml file that parses to None (empty) must not crash the merge
    (tmp_path / "intrinsics.yaml").write_text("\n")
    vision.save_intrinsics("CAM-C", 640, 480, np.eye(3), np.zeros(5), rms=0.9)
    assert vision.intrinsics_for("CAM-C", 640, 480) is not None
    # negative/zero/NaN marker lengths never reach the detector (garbage-pose guard)
    (tmp_path / "vision.yaml").write_text("markers:\n  0: -0.012\n  1: 0\n  2: .nan\n  3: 0.008\n")
    assert vision.marker_lengths() == {3: 0.008}


def test_make_detector_for_calibrated_vs_plain(tmp_path, monkeypatch):
    monkeypatch.setattr(vision, "VISION_CONFIG", tmp_path / "vision.yaml")
    monkeypatch.setattr(vision, "INTRINSICS_FILE", tmp_path / "intrinsics.yaml")
    plain = vision.make_detector_for("CAM-X", 1280, 720)       # no config at all
    assert plain.has_pose is False
    vision.save_intrinsics("CAM-X", 1280, 720, np.array([[800.0, 0, 640], [0, 800.0, 360], [0, 0, 1]]),
                           np.zeros(5), rms=0.4)
    (tmp_path / "vision.yaml").write_text("markers:\n  0: 0.012\n")
    cal = vision.make_detector_for("CAM-X", 1280, 720)
    assert cal.has_pose is True and cal.marker_lengths == {0: 0.012}


def test_calibration_tool_view_diversity():
    """accept_view (pure logic in tools/calibrate_intrinsics.py): rejects too-few corners and
    caps near-identical views, so the calibration can't be fed 15 copies of the same pose."""
    import importlib.util
    from pathlib import Path
    p = Path(vision.__file__).resolve().parent.parent / "tools" / "calibrate_intrinsics.py"
    spec = importlib.util.spec_from_file_location("calibrate_intrinsics", p)
    tool = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(tool)
    center = np.array([[[600.0 + i, 340.0 + i]] for i in range(14)])   # 14 corners, mid-frame
    buckets = {}
    assert tool.accept_view(buckets, center, 1280, 720) is True
    assert tool.accept_view(buckets, center, 1280, 720) is True        # 2nd of same pose ok
    assert tool.accept_view(buckets, center, 1280, 720) is False       # 3rd identical -> rejected
    few = center[:5]
    assert tool.accept_view(buckets, few, 1280, 720) is False          # too few corners
    corner_view = center + np.array([[-500.0, -250.0]])                # other image region -> new bucket
    assert tool.accept_view(buckets, corner_view, 1280, 720) is True


# ─────────────────────────── hand-eye: base-frame poses ───────────────────────────
def _pose(rot=None, t=(0, 0, 0.5)):
    R = np.eye(3) if rot is None else rot
    rvec, _ = cv2.Rodrigues(R)
    return {"rvec": [float(v) for v in rvec.flatten()], "tvec": [float(v) for v in t],
            "distance_m": float(np.linalg.norm(t))}


def _rz(deg):
    a = np.deg2rad(deg)
    return np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1.0]])


def test_annotate_in_base_known_geometry():
    # identity-rotated reference: base axes == camera axes -> pure translation delta
    dets = [{"id": 0, "pose": _pose(t=(0, 0, 0.5))},
            {"id": 1, "pose": _pose(t=(0.1, 0, 0.5))},
            {"id": 7, "center": [5, 5]}]                      # no pose -> untouched
    vision.annotate_in_base(dets, 0)
    assert dets[0]["pose"].get("is_reference") is True
    assert np.allclose(dets[1]["pose"]["in_base"]["pos"], [0.1, 0, 0], atol=1e-9)
    assert np.allclose(dets[1]["pose"]["in_base"]["rvec"], [0, 0, 0], atol=1e-9)
    assert "in_base" not in dets[2].get("pose", {}) and "pose" not in dets[2]
    # reference rotated 90° about z -> the same world offset reads as -y in the base frame
    dets = [{"id": 0, "pose": _pose(rot=_rz(90), t=(0, 0, 0.5))},
            {"id": 1, "pose": _pose(t=(0.1, 0, 0.5))}]
    vision.annotate_in_base(dets, 0)
    assert np.allclose(dets[1]["pose"]["in_base"]["pos"], [0, -0.1, 0], atol=1e-9)


def test_in_base_is_camera_placement_invariant():
    """THE hand-eye property: move the camera (one rigid transform applied to BOTH markers) and
    the base-frame pose must not change — that's what makes measurements rig-stable."""
    ref, mk = _pose(rot=_rz(30), t=(0.02, -0.01, 0.4)), _pose(rot=_rz(-45), t=(0.1, 0.05, 0.55))
    a = [{"id": 0, "pose": dict(ref)}, {"id": 1, "pose": dict(mk)}]
    vision.annotate_in_base(a, 0)
    # a camera move = premultiplying both T_cam_marker by the same rigid transform
    T_shift = np.eye(4)
    T_shift[:3, :3] = _rz(70) @ np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0.0]])
    T_shift[:3, 3] = [0.3, -0.2, 0.1]

    def shifted(p):
        T = vision._pose_to_T(p["rvec"], p["tvec"])
        T2 = T_shift @ T
        rvec, _ = cv2.Rodrigues(T2[:3, :3])
        return {"rvec": [float(v) for v in rvec.flatten()], "tvec": [float(v) for v in T2[:3, 3]]}

    b = [{"id": 0, "pose": shifted(ref)}, {"id": 1, "pose": shifted(mk)}]
    vision.annotate_in_base(b, 0)
    assert np.allclose(a[1]["pose"]["in_base"]["pos"], b[1]["pose"]["in_base"]["pos"], atol=1e-9)
    assert np.allclose(a[1]["pose"]["in_base"]["rvec"], b[1]["pose"]["in_base"]["rvec"], atol=1e-9)


def test_annotate_in_base_noop_without_reference():
    dets = [{"id": 1, "pose": _pose()}, {"id": 2, "center": [1, 1]}]
    vision.annotate_in_base(dets, 0)                          # ref id 0 not present
    assert "in_base" not in dets[0]["pose"]


def test_duplicate_reference_yields_no_base_frame():
    """Two detections claiming the reference id (false positive, reflection, second print): an
    arbitrary pick would silently poison every relative pose — so nobody gets an in_base."""
    dets = [{"id": 0, "pose": _pose(t=(0, 0, 0.5))},
            {"id": 0, "pose": _pose(t=(0.2, 0, 0.6))},
            {"id": 1, "pose": _pose(t=(0.1, 0, 0.5))}]
    vision.annotate_in_base(dets, 0)
    assert all("in_base" not in d["pose"] for d in dets)
    assert all("is_reference" not in d["pose"] for d in dets)


def test_ambiguous_reference_blocks_in_base():
    """The IPPE planar-pose ambiguity guard: an ambiguous REFERENCE orientation must never be used
    as the base frame (a flipped ref had leaked 6 cm of lateral offset into z before this guard) —
    but ambiguity on a NON-reference marker still allows its (position-stable) in_base."""
    dets = [{"id": 0, "pose": {**_pose(t=(0, 0, 0.5)), "ambiguous": True}},
            {"id": 1, "pose": _pose(t=(0.1, 0, 0.5))}]
    vision.annotate_in_base(dets, 0)
    assert "in_base" not in dets[1]["pose"] and "is_reference" not in dets[0]["pose"]
    dets = [{"id": 0, "pose": _pose(t=(0, 0, 0.5))},
            {"id": 1, "pose": {**_pose(t=(0.1, 0, 0.5)), "ambiguous": True}}]
    vision.annotate_in_base(dets, 0)
    assert np.allclose(dets[1]["pose"]["in_base"]["pos"], [0.1, 0, 0], atol=1e-9)


def test_detect_two_frontal_markers_end_to_end_in_base():
    """Rendered fronto-parallel scene (the historically nasty case): with subpixel corners + the
    best-of-both-IPPE-solutions pose, id 3 must sit at the exact metric lateral offset."""
    px, l_m, fx = 240, 0.05, 800.0
    canvas = np.full((400, 900), 255, np.uint8)
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    canvas[80:80 + px, 60:60 + px] = cv2.aruco.generateImageMarker(d, 0, px)
    canvas[80:80 + px, 560:560 + px] = cv2.aruco.generateImageMarker(d, 3, px)
    K = np.array([[fx, 0, 450.0], [0, fx, 200.0], [0, 0, 1.0]])
    det = vision.MarkerDetector(marker_lengths={0: l_m, 3: l_m}, camera_matrix=K, reference_id=0)
    dets = {x["id"]: x for x in det.detect(cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR))}
    assert dets[0]["pose"].get("is_reference") is True
    z = fx * l_m / px
    pos = dets[3]["pose"]["in_base"]["pos"]
    assert abs(pos[0] - 500 * z / fx) < 0.003                  # 500 px apart -> 104.2 mm ± 3 mm
    assert abs(pos[1]) < 0.003 and abs(pos[2]) < 0.01
    assert np.linalg.norm(dets[3]["pose"]["in_base"]["rvec"]) < 0.1


# detector marker-frame convention: bitmap corners map BL,BR,TR,TL onto the IPPE square, i.e. the
# detected frame is the "natural" one flipped 180° about x. Constant per marker -> cancels out of
# every RELATIVE measurement; ground truths below are expressed via F = diag(1,-1,-1).
_F = np.diag([1.0, -1.0, -1.0])


def _render_marker(canvas, K, marker_id, px, length_m, rvec, tvec):
    """Perspective-render a marker at a KNOWN 6-DoF pose into the canvas (ground truth for E2E)."""
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    b = 40
    m = np.full((px + 2 * b, px + 2 * b), 255, np.uint8)
    m[b:b + px, b:b + px] = cv2.aruco.generateImageMarker(d, marker_id, px)
    s = length_m / 2.0
    obj = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], np.float32)
    img_pts, _ = cv2.projectPoints(obj, np.asarray(rvec, float), np.asarray(tvec, float), K, None)
    TL, TR, BR, BL = [b, b], [b + px, b], [b + px, b + px], [b, b + px]
    src = np.array([BL, BR, TR, TL], np.float32)               # detector convention (see _F note)
    H = cv2.getPerspectiveTransform(src, img_pts.reshape(4, 2).astype(np.float32))
    warped = cv2.warpPerspective(m, H, (canvas.shape[1], canvas.shape[0]),
                                 flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT,
                                 borderValue=255)
    np.minimum(canvas, warped, out=canvas)                     # black marker wins over white


def test_detect_two_tilted_markers_end_to_end_in_base():
    """Rendered scene with KNOWN ground-truth poses (both tilted 20-25° -> unambiguous): the
    detector must recover marker 3's base-frame pose to millimeter accuracy."""
    K = np.array([[800.0, 0, 450.0], [0, 800.0, 220.0], [0, 0, 1.0]])
    l_m = 0.05

    def _ry(deg):
        a = np.deg2rad(deg)
        return np.array([[np.cos(a), 0, np.sin(a)], [0, 1, 0], [-np.sin(a), 0, np.cos(a)]])

    R0, t0 = _ry(25), np.array([-0.05, 0.0, 0.40])
    R3, t3 = _ry(-20), np.array([0.07, 0.02, 0.45])
    rv0, _ = cv2.Rodrigues(R0)
    rv3, _ = cv2.Rodrigues(R3)
    canvas = np.full((440, 900), 255, np.uint8)
    _render_marker(canvas, K, 0, 240, l_m, rv0, t0)
    _render_marker(canvas, K, 3, 240, l_m, rv3, t3)
    det = vision.MarkerDetector(marker_lengths={0: l_m, 3: l_m}, camera_matrix=K, reference_id=0)
    dets = {x["id"]: x for x in det.detect(cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR))}
    assert dets[0]["pose"].get("is_reference") is True
    # ground truth in the detector's frame convention (see _F above)
    expected = _F @ (R0.T @ (t3 - t0))
    pos = np.array(dets[3]["pose"]["in_base"]["pos"])
    assert np.allclose(pos, expected, atol=0.004), (pos, expected)     # sub-4-mm
    rv_expected, _ = cv2.Rodrigues(_F @ (R0.T @ R3) @ _F)
    rv_got = np.array(dets[3]["pose"]["in_base"]["rvec"])
    assert np.linalg.norm(rv_got - rv_expected.flatten()) < 0.08       # orientation matches too


def test_qimage_to_bgr_roundtrips_a_known_color():
    _qapp()
    im = QImage(8, 6, QImage.Format.Format_RGB888)
    im.fill(0xFF8040)                                          # R=0xFF, G=0x80, B=0x40
    bgr = vision.qimage_to_bgr(im)
    assert bgr.shape == (6, 8, 3)
    assert list(bgr[0, 0]) == [0x40, 0x80, 0xFF]              # cv2 BGR order


def test_draw_overlay_returns_new_annotated_image():
    _qapp()
    im = QImage(80, 60, QImage.Format.Format_RGB888)
    im.fill(0x000000)
    dets = [{"id": 5, "corners": [[10, 10], [40, 10], [40, 40], [10, 40]], "center": [25, 25]}]
    out = vision.draw_overlay(im, dets)
    assert out.width() == 80 and out.height() == 60
    assert out is not im                                       # NEW image → the raw frame is untouched
    assert int(vision.qimage_to_bgr(out).sum()) > 0            # the outline actually got drawn
    assert vision.draw_overlay(im, []) is im                   # empty dets → early return, same object
