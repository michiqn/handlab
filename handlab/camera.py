"""camera.py — a real webcam feed for the agent snapshot (Phase 2 / vision-in-the-loop).

The Bridge owns ONE headless QVideoSink. Its frames become QImages published through a
CamImageProvider (mirroring TwinImageProvider), so BOTH the Agent-tab live view and
`agent_snapshot_jpeg` read the same latest frame — one device consumer, one conversion. No QML
`VideoOutput`: a QMediaCaptureSession drives a single sink, and snapshot must work even when the
Agent tab isn't the visible page. All camera lifecycle stays on the Qt thread (built inside
Bridge.__init__, which only runs after QGuiApplication exists — QtMultimedia requires it).

Device selection: QMediaDevices.videoInputs() enumerates every camera (built-in FaceTime,
Continuity iPhone, and a USB UVC like the EMEET C960) WITHOUT triggering the macOS camera
permission prompt; the prompt fires on the first QCamera.start(). So the built-in cam works out of
the box and a USB cam is just another picker entry — no code change.
"""

from __future__ import annotations

from PySide6.QtCore import QObject, QRunnable, QThreadPool, Signal
from PySide6.QtGui import QImage
from PySide6.QtMultimedia import (QCamera, QCameraDevice, QMediaCaptureSession,
                                  QMediaDevices, QVideoFrame, QVideoSink)

from .bridge import FrameImageProvider

# same latest-frame provider the twin uses (shared class in bridge.py — QtMultimedia-free, so the
# Bridge can build its provider even when this module fails to import); alias kept for callers/tests
CamImageProvider = FrameImageProvider

# optional ArUco detection (the [vision] extra). Guarded exactly like the camera itself: no opencv
# -> no detection, no overlay, the feed behaves exactly as before.
try:
    from . import vision
    _VISION_OK = vision.AVAILABLE
except Exception:
    vision = None
    _VISION_OK = False


def _frame_to_qimage(frame: QVideoFrame) -> QImage | None:
    """A delivered QVideoFrame -> a detached QImage, or None if it isn't usable this tick
    (invalid/empty frame). Kept module-level so the frame path is unit-testable via
    QVideoFrame(QImage) without opening a real device."""
    if not frame.isValid():
        return None
    img = frame.toImage()
    if img.isNull():
        return None
    return img.copy()                 # detach from the frame's buffer (matches the twin's .copy())


class _DetectSignals(QObject):
    """Carries a worker-thread detection result back to the Qt thread (queued AutoConnection)."""
    done = Signal(int, object)                     # (epoch, list[dict]) — epoch guards staleness


class _DetectJob(QRunnable):
    """Runs MarkerDetector.detect(bgr) on a QThreadPool worker so the ArUco pass never stalls the
    Qt thread (which renders the twin at 30 fps + serves 10 Hz telemetry). Carries the detection
    EPOCH it was started under — a result arriving after stop()/device switch must be dropped, or
    the old camera's detections get republished under the new camera's identity."""

    def __init__(self, detector, bgr, signals: "_DetectSignals", epoch: int):
        super().__init__()
        self._detector = detector
        self._bgr = bgr
        self._signals = signals
        self._epoch = epoch

    def run(self) -> None:
        try:
            dets = self._detector.detect(self._bgr)
        except Exception:
            dets = []
        self._signals.done.emit(self._epoch, dets)


class CameraController(QObject):
    """Owns the single webcam device (QCamera -> QMediaCaptureSession -> one headless QVideoSink)
    and publishes each frame as a QImage via `provider`. `current()` is the snapshot seam; the live
    view polls `provider` through image://cam. Touch only on the Qt thread (its home thread)."""

    frameReady     = Signal(int)     # per delivered frame -> QML Image poll
    devicesChanged = Signal()        # videoInputs hotplug (USB cam plug/unplug) -> refresh picker
    stateChanged   = Signal()        # active / selected device changed
    errorText      = Signal(str)     # e.g. macOS camera permission denied
    detectionsChanged = Signal()     # new ArUco detections available -> Agent-tab count + overlay

    _MAX_W = 1280                    # cap capture to ~720p -> keeps per-frame toImage() cheap on
    #                                  the Qt thread (which also renders the twin at 30 fps)
    _DET_EVERY = 3                   # detect every Nth frame (~10 Hz at 30 fps) to bound CPU

    def __init__(self, parent: QObject | None = None):
        super().__init__(parent)
        self.provider = CamImageProvider()
        self._session = QMediaCaptureSession(self)
        self._sink = QVideoSink(self)
        self._session.setVideoSink(self._sink)
        self._sink.videoFrameChanged.connect(self._on_frame)   # AutoConnection -> Qt thread
        self._devices = QMediaDevices(self)                    # kept alive for the hotplug signal
        self._devices.videoInputsChanged.connect(self.devicesChanged)
        self._camera: QCamera | None = None
        self._img: QImage | None = None
        self._frame_no = 0
        self._active = False
        self._want_device_id = ""        # remembered across the async permission request
        # ArUco detection (optional): runs OFF the Qt thread; results annotate the preview + feed the
        # agent. current() stays the RAW frame so the model snapshot is never annotated. The detector
        # is built lazily on the FIRST delivered frame — only then are the device AND the actual
        # frame size known, which decides whether stored intrinsics apply (calibrated poses).
        self._detector = None
        self._det_stale = True           # (re)build the detector on the next frame
        self._det_key = None             # (device_id, w, h) the current detector was built for
        self._det_epoch = 0              # bumps on stop/switch -> stale worker results get dropped
        self._device_id = ""             # selected device (keys the per-camera intrinsics)
        self._dets: list[dict] = []
        self._det_busy = False
        self._det_skip = 0
        self._det_signals = _DetectSignals(self)
        self._det_signals.done.connect(self._on_dets)

    # ---- introspection for the QML picker ----
    def device_list(self) -> list[dict]:
        return [{"id": bytes(d.id()).decode("latin1"),
                 "name": d.description(), "default": d.isDefault()}
                for d in QMediaDevices.videoInputs()]

    def current(self) -> QImage | None:      # the snapshot seam (read on the Qt thread)
        return self._img

    @property
    def active(self) -> bool:
        return self._active

    @property
    def frame_no(self) -> int:
        return self._frame_no

    @property
    def device_id(self) -> str:
        return self._device_id

    @property
    def calibrated(self) -> bool:
        """True when the CURRENT session's detector delivers 6-DoF poses. Gated on _det_stale so a
        device switch never reports the previous camera's calibration state for the new device."""
        return (not self._det_stale and self._detector is not None
                and self._detector.has_pose)

    # ---- lifecycle (Qt thread only) ----
    def _pick(self, device_id: str) -> QCameraDevice | None:
        cams = QMediaDevices.videoInputs()
        if device_id:
            for d in cams:
                if bytes(d.id()).decode("latin1") == device_id:
                    return d
        dflt = QMediaDevices.defaultVideoInput()
        if dflt is not None and not dflt.isNull():
            return dflt
        return cams[0] if cams else None

    def _apply_format(self, dev: QCameraDevice) -> None:
        """Cap resolution to ~720p (largest <= _MAX_W, else the smallest) so toImage() stays cheap
        and the C960 doesn't default to 1080p/60."""
        if self._camera is None:
            return
        fmts = dev.videoFormats()
        if not fmts:
            return
        under = [f for f in fmts if f.resolution().width() <= self._MAX_W]
        best = (max(under, key=lambda f: f.resolution().width()) if under
                else min(fmts, key=lambda f: f.resolution().width()))
        try:
            self._camera.setCameraFormat(best)
        except Exception:
            pass

    def start(self, device_id: str = "") -> bool:
        """Ensure macOS camera permission, then open the device. On first-ever use the permission is
        Undetermined -> we EXPLICITLY request it (shows the system dialog) and open the device from
        the grant callback; Qt's implicit request-on-start does not fire reliably (the camera just
        stays inactive with no prompt and no error). Denied -> a clear message the Agent tab shows."""
        if self._pick(device_id) is None:
            self.errorText.emit("no camera device")
            return False
        self._want_device_id = device_id
        from PySide6.QtCore import QCameraPermission, Qt
        from PySide6.QtGui import QGuiApplication
        appobj = QGuiApplication.instance()
        if appobj is not None:
            status = appobj.checkPermission(QCameraPermission())
            if status == Qt.PermissionStatus.Denied:
                self.errorText.emit("camera access denied — enable this app in System Settings › "
                                    "Privacy & Security › Camera, then relaunch")
                return False
            if status == Qt.PermissionStatus.Undetermined:
                # PySide6 wants the 3-arg form: (permission, context QObject, callable)
                appobj.requestPermission(QCameraPermission(), self, self._on_permission)
                return True                  # system dialog shown; _open() runs from the callback
        return self._open(device_id)

    def _on_permission(self, perm) -> None:
        from PySide6.QtCore import Qt
        if perm.status() == Qt.PermissionStatus.Granted:
            self._open(self._want_device_id)
        else:
            self.errorText.emit("camera access denied — enable this app in System Settings › "
                                "Privacy & Security › Camera")

    def _open(self, device_id: str) -> bool:
        dev = self._pick(device_id)
        if dev is None or dev.isNull():
            self.errorText.emit("no camera device")
            return False
        self.stop()
        self._camera = QCamera(dev, self)
        self._camera.errorOccurred.connect(self._on_cam_error)
        self._camera.activeChanged.connect(self._set_active)
        self._session.setCamera(self._camera)
        self._apply_format(dev)
        self._device_id = bytes(dev.id()).decode("latin1")
        self._det_stale = True                       # device (maybe) changed -> rebuild on next frame
        self._camera.start()
        self._set_active(self._camera.isActive())    # activeChanged confirms it once frames flow
        self.stateChanged.emit()
        return True

    def stop(self) -> None:
        if self._camera is not None:
            try:
                self._camera.stop()
                self._session.setCamera(None)
                self._camera.deleteLater()
            except Exception:
                pass
            self._camera = None
        self._clear_frame()
        self._set_active(False)

    def _clear_frame(self) -> None:
        """Drop the retained frame when the device goes down, so current() returns None again — a
        stopped/errored camera must never serve a stale frame to the snapshot, nor flash the old
        frame in the live view on restart (frame_no back to 0 keeps the QML gate off until a real
        new frame arrives)."""
        self._img = None
        self.provider.set_frame(None)
        self._frame_no = 0
        self._dets = []
        self._det_busy = False
        self._det_skip = 0
        self._det_stale = True           # next session rebuilds (device/intrinsics may differ)
        self._det_key = None
        self._detector = None            # a stopped camera keeps no stale detector/calibration
        self._device_id = ""
        self._det_epoch += 1             # in-flight worker results become stale -> _on_dets drops them

    # ---- frame path (Qt thread via the AutoConnection queue) ----
    def _on_frame(self, frame: QVideoFrame) -> None:
        from .perf import PERF                     # opt-in probes (HANDLAB_PERF=1), zero-cost when off
        PERF.mark("robocam.frame")
        with PERF.lap("robocam.handler"):
            self._on_frame_body(frame)

    def _on_frame_body(self, frame: QVideoFrame) -> None:
        if self._camera is None:                   # queued frame delivered after stop() -> drop it
            return                                 # (else a dead camera repopulates _img/preview)
        img = _frame_to_qimage(frame)
        if img is None:
            return
        self._img = img                            # RAW frame — current()/snapshot stays unannotated
        # (Re)build the detector when the session is stale OR the frame geometry changed — a queued
        # frame from the PREVIOUS camera could otherwise consume the rebuild with the wrong size and
        # lock a mismatched calibration in for the whole session. Keying on (device, w, h) self-heals
        # on the new camera's first real frame.
        key = (self._device_id, img.width(), img.height())
        if _VISION_OK and (self._det_stale or key != self._det_key):
            try:
                self._detector = vision.make_detector_for(*key)
            except Exception:
                self._detector = None
            self._det_key = key
            self._det_stale = False
            self.stateChanged.emit()                # calibrated-state may have changed
        preview = img
        if self._detector is not None:
            self._det_skip += 1
            if not self._det_busy and self._det_skip >= self._DET_EVERY:
                self._det_skip = 0
                self._det_busy = True
                bgr = vision.qimage_to_bgr(img)    # cheap memcpy here; the ArUco pass runs off-thread
                QThreadPool.globalInstance().start(
                    _DetectJob(self._detector, bgr, self._det_signals, self._det_epoch))
            if self._dets:
                preview = vision.draw_overlay(img, self._dets)
        self.provider.set_frame(preview)           # ANNOTATED frame drives the live preview
        self._frame_no += 1
        self.frameReady.emit(self._frame_no)

    def _on_dets(self, epoch: int, dets: list) -> None:   # Qt thread (queued from the worker)
        if epoch != self._det_epoch:               # result from a stopped/switched session -> drop
            return                                 # (and leave _det_busy to the CURRENT session)
        self._dets = dets or []
        self._det_busy = False
        self.detectionsChanged.emit()

    def latest_detections(self) -> list[dict]:
        return list(self._dets)

    def _set_active(self, active: bool) -> None:
        active = bool(active)
        if active != self._active:
            self._active = active
            self.stateChanged.emit()

    def _on_cam_error(self, err, msg: str) -> None:
        self.errorText.emit(msg or "camera error")
        self._clear_frame()                  # a dead camera keeps no stale frame around
        self._set_active(False)
