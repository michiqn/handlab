"""Phase-2 webcam feed — the frame path + the twin/cam snapshot source swap.

Headless, no real camera (opening a device needs a QGuiApplication + TCC and would prompt): we use
the QVideoFrame(QImage) seam for the frame path and fakes for the source toggle. The QCamera/
QMediaCaptureSession lifecycle + real device enumeration are covered by the live smoke
(scratchpad verify_cam.py), not here. Runs under a QCoreApplication like test_control_server."""

from PySide6.QtCore import QCoreApplication
from PySide6.QtGui import QImage

from handlab import camera as cammod
from handlab.bridge_agent import BridgeAgentMixin


def _qapp():
    return QCoreApplication.instance() or QCoreApplication([])


def _img(fill=0x224466, w=64, h=48):
    im = QImage(w, h, QImage.Format.Format_RGB888)
    im.fill(fill)
    return im


# ─────────────────────────── frame path ───────────────────────────
def test_frame_to_qimage_guards_and_converts():
    _qapp()
    from PySide6.QtMultimedia import QVideoFrame
    assert cammod._frame_to_qimage(QVideoFrame()) is None          # invalid frame -> skip this tick
    out = cammod._frame_to_qimage(QVideoFrame(_img()))
    assert out is not None and not out.isNull() and out.width() == 64


def test_provider_none_until_first_frame_serves_then_clears():
    _qapp()
    p = cammod.CamImageProvider()
    assert p.current() is None                                     # -> agent snapshot 503, not a fake
    assert not p.requestImage("f0", None, None).isNull()           # QML still gets the placeholder
    im = _img()
    p.set_frame(im)
    assert p.current() is im
    assert p.requestImage("f1", None, None) is im
    p.set_frame(None)                                              # teardown (stop/error) clears it
    assert p.current() is None                                     # no stale frame survives
    assert not p.requestImage("f2", None, None).isNull()           # back to the placeholder


# ─────────────────────────── snapshot source swap ───────────────────────────
class _Src:
    def __init__(self, img, active=True):
        self._img = img
        self.active = active

    def current(self):
        return self._img


class _FakeBridge(BridgeAgentMixin):
    def __init__(self, source, cam_img, twin_img, cam_active=True):
        self._snapshot_source = source
        self.camera = _Src(cam_img, cam_active)
        self.image_provider = _Src(twin_img)


def test_snapshot_returns_selected_source():
    _qapp()
    cam, twin = _img(0x00ff00), _img(0x0000ff)
    # cam selected, live + has a frame -> JPEG of the cam
    assert _FakeBridge("cam", cam, twin).agent_snapshot_jpeg()[:3] == b"\xff\xd8\xff"
    # twin selected -> JPEG of the twin
    assert _FakeBridge("twin", cam, twin).agent_snapshot_jpeg()[:3] == b"\xff\xd8\xff"
    # cam selected but no frame yet (starting / TCC pending) -> falls back to twin, still a frame
    assert _FakeBridge("cam", None, twin).agent_snapshot_jpeg()[:3] == b"\xff\xd8\xff"
    # cam selected, no cam AND no twin frame -> empty (control server turns this into a 503)
    assert _FakeBridge("cam", None, None).agent_snapshot_jpeg() == b""


def test_inactive_camera_never_serves_a_stale_frame():
    """Regression for the review finding: a stopped/unplugged camera keeps its last frame, but must
    NOT be served as if live — snapshot falls back to the twin (or 503), never the frozen frame."""
    _qapp()
    cam, twin = _img(0x00ff00), _img(0x0000ff)     # green cam vs blue twin
    # cam selected + HAS a retained frame, but the camera is no longer active -> serve the twin
    stale = _FakeBridge("cam", cam, twin, cam_active=False).agent_snapshot_jpeg()
    assert stale == _FakeBridge("twin", cam, twin).agent_snapshot_jpeg()   # == twin, not the cam frame
    # inactive cam with a frame + no twin frame -> 503, not the stale frame
    assert _FakeBridge("cam", cam, None, cam_active=False).agent_snapshot_jpeg() == b""


def test_swap_actually_picks_a_different_source():
    _qapp()
    cam, twin = _img(0x00ff00), _img(0x0000ff)     # green vs blue -> different JPEG bytes
    as_cam = _FakeBridge("cam", cam, twin).agent_snapshot_jpeg()
    as_twin = _FakeBridge("twin", cam, twin).agent_snapshot_jpeg()
    assert as_cam != as_twin                        # the toggle really selects a different image
