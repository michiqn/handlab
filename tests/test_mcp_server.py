"""The handlab MCP server: each tool reaches the live control API, carries the token, and fails
CLEANLY (a readable JSON error, never a crash) when the GUI isn't running.

Headless: a real ControlServer over a FakeBridge with a synchronous invoker (no Qt), and the MCP
tools are plain callables (FastMCP's @tool returns the function unchanged), so we call them
directly and point their discovery global at the test server.
"""
import json

import pytest

pytest.importorskip("mcp")   # skip the whole module until `pip install -e '.[agent]'`

from mcp.server.fastmcp import Image

from handlab.control_server import ControlServer
from handlab import mcp_server


class _Fake:
    """Just the agent-slot surface the MCP tools reach (mirrors test_control_server.FakeBridge)."""

    def __init__(self, armed=False):
        self.armed = armed
        self.calls = []

    def agentState(self):
        return json.dumps({"driver": "mock", "armed": self.armed, "online": [],
                           "servos": {}, "dofs": [], "poses": [{"name": "open", "kind": "pose"}]})

    def agentSetDof(self, finger, dof, deg):
        if not self.armed:
            return json.dumps({"ok": False, "error": "disarmed"})
        self.calls.append(("dof", finger, dof, deg))
        return json.dumps({"ok": True, "finger": finger, "dof": dof, "deg": deg})

    def agentEstop(self):
        self.armed = False
        self.calls.append(("estop",))
        return json.dumps({"ok": True, "estop": True, "armed": False})

    def setArmed(self, on):
        self.armed = bool(on)

    def agent_snapshot_jpeg(self, max_px=768):
        return b"\xff\xd8\xff" + b"jpegbytes"

    def agent_detections(self):
        return {"ok": True, "active": True, "calibrated": False, "count": 1,
                "markers": [{"id": 7, "corners": [[0, 0]] * 4, "center": [1, 2]}]}


def _serve(bridge, token="tok"):
    srv = ControlServer(bridge, invoke=lambda fn: fn(), token=token, port=0)
    srv.start()
    return srv


def _point(monkeypatch, tmp_path, port, token="tok"):
    f = tmp_path / "agent.json"
    f.write_text(json.dumps({"host": "127.0.0.1", "port": port, "token": token}))
    monkeypatch.setattr(mcp_server, "DISCOVERY", f)


def test_get_state_reaches_server(monkeypatch, tmp_path):
    b = _Fake()
    srv = _serve(b)
    try:
        _point(monkeypatch, tmp_path, srv.port)
        st = json.loads(mcp_server.get_state())
        assert st["driver"] == "mock" and st["armed"] is False
    finally:
        srv.stop()


def test_set_dof_gated_then_armed(monkeypatch, tmp_path):
    b = _Fake()
    srv = _serve(b)
    try:
        _point(monkeypatch, tmp_path, srv.port)
        assert json.loads(mcp_server.set_dof("index", "flex", 40))["ok"] is False   # disarmed
        assert b.calls == []
        b.setArmed(True)
        assert json.loads(mcp_server.set_dof("index", "flex", 40))["ok"] is True
        assert ("dof", "index", "flex", 40.0) in b.calls
    finally:
        srv.stop()


def test_wrong_token_rejected_not_crashed(monkeypatch, tmp_path):
    b = _Fake(armed=True)
    srv = _serve(b, token="right")
    try:
        _point(monkeypatch, tmp_path, srv.port, token="WRONG")
        r = json.loads(mcp_server.set_dof("index", "flex", 40))
        assert r["ok"] is False and "token" in r["error"].lower()
        assert b.calls == []                          # a 401 never reached the bridge
    finally:
        srv.stop()


def test_snapshot_returns_image(monkeypatch, tmp_path):
    b = _Fake()
    srv = _serve(b)
    try:
        _point(monkeypatch, tmp_path, srv.port)
        img = mcp_server.snapshot()
        assert isinstance(img, Image)                 # JPEG magic → an MCP image content block
    finally:
        srv.stop()


def test_detect_markers_route(monkeypatch, tmp_path):
    b = _Fake()
    srv = _serve(b)
    try:
        _point(monkeypatch, tmp_path, srv.port)
        d = json.loads(mcp_server.detect_markers())
        assert d["active"] is True and d["markers"][0]["id"] == 7    # read-only, works disarmed
    finally:
        srv.stop()


def test_estop_route(monkeypatch, tmp_path):
    b = _Fake(armed=True)
    srv = _serve(b)
    try:
        _point(monkeypatch, tmp_path, srv.port)
        r = json.loads(mcp_server.estop())
        assert r.get("ok") is True and ("estop",) in b.calls and b.armed is False
    finally:
        srv.stop()


def test_server_down_is_clean_error(monkeypatch, tmp_path):
    _point(monkeypatch, tmp_path, 9)                  # nothing listening → connection refused
    r = json.loads(mcp_server.get_state())
    assert r["ok"] is False and "reachable" in r["error"].lower()   # readable, no exception


def test_missing_discovery_is_clean_error(monkeypatch, tmp_path):
    monkeypatch.setattr(mcp_server, "DISCOVERY", tmp_path / "nope.json")
    r = json.loads(mcp_server.get_state())
    assert r["ok"] is False and "not running" in r["error"].lower()
