"""The /detections read path: bridge.agent_detections() shape + the ControlApi route. Read-only
perception (no arm), so it works disarmed. FakeBridge with a synchronous invoker — no Qt, no camera,
no opencv needed (the detections themselves are produced in camera.py, unit-tested in test_vision)."""
import json

from handlab.control_server import ControlApi
from handlab.bridge_agent import BridgeAgentMixin


class _FakeCam:
    def __init__(self, active, dets, calibrated=False, device_id=""):
        self.active = active
        self.calibrated = calibrated
        self.device_id = device_id
        self._dets = dets

    def latest_detections(self):
        return list(self._dets)


class _Bridge:
    """Only the surface agent_detections() reads — reusing the REAL mixin method under test."""
    agent_detections = BridgeAgentMixin.agent_detections

    def __init__(self, cam):
        self.camera = cam


def _api(bridge):
    return ControlApi(bridge, invoke=lambda fn: fn())          # synchronous — no Qt


def test_detections_empty_when_camera_off():
    st, ct, body = _api(_Bridge(_FakeCam(active=False, dets=[{"id": 1}]))).detections()
    d = json.loads(body)
    assert st == 200 and ct == "application/json"
    assert d["ok"] is True and d["active"] is False and d["count"] == 0 and d["markers"] == []


def test_detections_passthrough_when_active():
    markers = [{"id": 7, "corners": [[0, 0], [1, 0], [1, 1], [0, 1]], "center": [0.5, 0.5]}]
    st, _ct, body = _api(_Bridge(_FakeCam(active=True, dets=markers))).detections()
    d = json.loads(body)
    assert st == 200 and d["active"] is True and d["count"] == 1
    assert d["markers"][0]["id"] == 7 and d["calibrated"] is False


def test_detections_when_no_camera_at_all():
    class _Bare:
        camera = None
        agent_detections = BridgeAgentMixin.agent_detections
    st, _ct, body = _api(_Bare()).detections()
    assert json.loads(body)["active"] is False


def test_detections_report_calibrated_and_device():
    """Once intrinsics are loaded the payload says so (the agent decides pixel- vs meter-reasoning
    from this flag) and names the device the calibration belongs to."""
    cam = _FakeCam(active=True, dets=[{"id": 0, "pose": {"distance_m": 0.4}}],
                   calibrated=True, device_id="CAM-42")
    d = json.loads(_api(_Bridge(cam)).detections()[2])
    assert d["calibrated"] is True and d["device"] == "CAM-42"
    assert d["markers"][0]["pose"]["distance_m"] == 0.4
    # camera off -> calibrated is False and device empty regardless of the detector state
    d_off = json.loads(_api(_Bridge(_FakeCam(False, [], calibrated=True, device_id="CAM-42"))).detections()[2])
    assert d_off["calibrated"] is False and d_off["device"] == ""
