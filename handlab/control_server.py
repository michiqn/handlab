"""control_server.py — a token-gated 127.0.0.1 HTTP surface onto the LIVE Bridge.

This is how the handlab MCP server (→ Claude) drives the SAME hand a human drives, without a
second process fighting for the serial port: the GUI keeps the port + the camera, and every
command comes in here and routes to the Bridge's *agent* slots (which enforce the ARM gate and
reuse the effective-range clamp, the force/velocity caps and the phased safe-home).

Thread-safety is the crux: the 10 Hz servo telemetry runs on the Qt thread, but HTTP requests
arrive on worker threads. `QtInvoker` marshals every Bridge call onto the Qt thread (a queued
signal carries the call and the worker blocks for the result) — the driver is NEVER touched off
the Qt thread. `ControlApi` holds the routing + the remote-arm policy and takes the invoker as a
parameter, so it is fully testable without Qt (inject `invoke=lambda fn: fn()`).
"""
from __future__ import annotations

import hmac
import json
import os
import threading
from urllib.parse import parse_qs, urlsplit
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Callable

from PySide6.QtCore import QObject, Signal, Slot


# ─────────────────────────────── Qt-thread invoker ───────────────────────────────
class _Call:
    """A pending Qt-thread call with a claim/cancel handshake so a caller that TIMED OUT can
    guarantee the command never actuates the hand later. Without this, a call queued behind a
    stalled Qt thread (a servo reconnect, a twin reload) would still run set_goal_position after
    the HTTP worker already returned an error — a phantom, unattributed robot move."""
    __slots__ = ("fn", "done", "result", "error", "_lock", "_state")

    def __init__(self, fn: Callable[[], Any]):
        self.fn = fn
        self.done = threading.Event()
        self.result: Any = None
        self.error: BaseException | None = None
        self._lock = threading.Lock()
        self._state = "pending"         # pending → running (Qt thread) | cancelled (caller timeout)

    def claim_run(self) -> bool:
        """Qt thread, before executing fn: False if the caller already gave up (→ don't actuate)."""
        with self._lock:
            if self._state == "cancelled":
                return False
            self._state = "running"
            return True

    def cancel(self) -> bool:
        """Caller thread, on timeout: True only if fn had NOT started yet (safe to drop it)."""
        with self._lock:
            if self._state == "pending":
                self._state = "cancelled"
                return True
            return False


class QtInvoker(QObject):
    """Run a callable on the thread this object lives on (the Qt/main thread). Create it on the
    main thread; a worker thread then calls `invoke(fn)` and blocks until `fn` has run on the Qt
    thread. The connection is AutoConnection, so a cross-thread emit is queued (marshalled) and a
    same-thread emit runs inline (no deadlock when called from the Qt thread itself)."""

    _submit = Signal(object)

    def __init__(self, parent: QObject | None = None):
        super().__init__(parent)
        self._submit.connect(self._run)

    @Slot(object)
    def _run(self, call: _Call) -> None:
        if not call.claim_run():        # caller already timed out + cancelled → never actuate
            return
        try:
            call.result = call.fn()
        except BaseException as e:      # noqa: BLE001 — carried back to the calling thread
            call.error = e
        finally:
            call.done.set()

    def invoke(self, fn: Callable[[], Any], timeout: float = 5.0) -> Any:
        call = _Call(fn)
        self._submit.emit(call)
        if not call.done.wait(timeout):
            if call.cancel():           # not started yet → drop it, it can never actuate the hand
                raise TimeoutError("handlab control: Qt thread did not respond in time")
            call.done.wait()            # already running → let it finish + return its real result
        if call.error is not None:
            raise call.error
        return call.result


# ─────────────────────────────── policy + routing ───────────────────────────────
def _bytes(obj: Any) -> bytes:
    return json.dumps(obj).encode()


class ControlApi:
    """Maps requests to Bridge *agent* slots. The agent slots already enforce the ARM gate and
    return a JSON string with an `ok` flag; we pass it through and pick the HTTP status from `ok`.
    The one policy that lives HERE (the remote boundary) is: the remote side may DISARM but never
    ARM — arming is a physical action in the Agent panel."""

    def __init__(self, bridge: Any, invoke: Callable[[Callable[[], Any]], Any],
                 snapshot: Callable[[int], bytes] | None = None,
                 log: Callable[[dict], None] | None = None):
        self._bridge = bridge
        self._invoke = invoke
        self._snapshot = snapshot
        self._log = log                     # one line per MUTATING command → the Agent-tab log

    def _act(self, tool: str, detail: str, fn: Callable[[], Any]):
        """Run a mutating agent call AND log it — both inside the one `_invoke` hop, so the log
        emit runs on the Qt thread and even a REFUSED command (disarmed, bad name) is recorded."""
        def run():
            raw = fn()
            if self._log is not None:
                try:
                    ok = bool(json.loads(raw).get("ok", True))
                except (TypeError, ValueError):
                    ok = True
                self._log({"tool": tool, "detail": detail, "ok": ok})
            return raw
        return self._passthrough(self._invoke(run))

    # each handler returns (status:int, content_type:str, body:bytes)
    def state(self):
        return self._passthrough(self._invoke(lambda: self._bridge.agentState()))

    def poses(self):
        st = json.loads(self._invoke(lambda: self._bridge.agentState()))
        return 200, "application/json", _bytes(st.get("poses", []))

    def snapshot(self, max_px: int = 768):
        grab = self._snapshot or (lambda px: self._invoke(lambda: self._bridge.agent_snapshot_jpeg(px)))
        data = grab(max_px)
        if not data:
            return 503, "application/json", _bytes({"ok": False, "error": "no frame yet"})
        return 200, "image/jpeg", data

    def detections(self):
        return 200, "application/json", _bytes(self._invoke(lambda: self._bridge.agent_detections()))

    def set_dof(self, body: dict):
        finger = str(body.get("finger", ""))
        dof = str(body.get("dof", ""))
        try:
            deg = float(body.get("deg"))
        except (TypeError, ValueError):
            return 400, "application/json", _bytes({"ok": False, "error": "deg must be a number"})
        return self._act(f"set_dof", f"{finger}/{dof}→{deg:g}°",
                         lambda: self._bridge.agentSetDof(finger, dof, deg))

    def apply_pose(self, body: dict):
        name = str(body.get("name", ""))
        return self._act("apply_pose", name, lambda: self._bridge.agentApplyPose(name))

    def play_clip(self, body: dict):
        name = str(body.get("name", ""))
        loop = bool(body.get("loop", False))
        return self._act("play_clip", name + (" ↻" if loop else ""),
                         lambda: self._bridge.agentPlayClip(name, loop))

    def home(self, body: dict):
        finger = str(body.get("finger", ""))
        return self._act("safe_home", finger or "all", lambda: self._bridge.agentSafeHome(finger))

    def torque(self, body: dict):
        on = bool(body.get("on", False))
        return self._act("torque", "on" if on else "off", lambda: self._bridge.agentTorque(on))

    def estop(self):
        return self._act("estop", "", lambda: self._bridge.agentEstop())

    def get_arm(self):
        armed = bool(self._invoke(lambda: self._bridge.armed))
        return 200, "application/json", _bytes({"armed": armed})

    def set_arm(self, body: dict):
        if bool(body.get("on", False)):
            return 403, "application/json", _bytes(
                {"ok": False, "error": "arming is a human action — use the ARM toggle in the Agent panel"})
        def run():
            self._bridge.setArmed(False)
            if self._log is not None:
                self._log({"tool": "disarm", "detail": "", "ok": True})
        self._invoke(run)
        return 200, "application/json", _bytes({"ok": True, "armed": False})

    def _passthrough(self, raw: Any):
        try:
            obj = json.loads(raw)
        except (TypeError, ValueError):
            obj = {"ok": True, "result": raw}
        status = 200 if obj.get("ok", True) else 409
        return status, "application/json", _bytes(obj)


# ─────────────────────────────── HTTP wrapper ───────────────────────────────
class _Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *args):        # keep the app's stdout clean
        pass

    def _auth(self) -> bool:
        tok = self.headers.get("X-Handlab-Token", "")
        if not self.server.token or not hmac.compare_digest(tok, self.server.token):
            self._send(401, "application/json", _bytes({"ok": False, "error": "bad or missing token"}))
            return False
        return True

    def _send(self, status: int, ctype: str, body: bytes) -> None:
        self.send_response(status)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _body_json(self) -> dict:
        n = int(self.headers.get("Content-Length", 0) or 0)
        if n <= 0:
            return {}
        try:
            return json.loads(self.rfile.read(n) or b"{}")
        except Exception:
            return {}

    def do_GET(self):
        if not self._auth():
            return
        api = self.server.api
        parts = urlsplit(self.path)
        path = parts.path
        try:
            if path == "/snapshot.jpg":
                try:                                     # ?max= sizes the frame (else default 768)
                    max_px = int(parse_qs(parts.query).get("max", ["768"])[0])
                except (TypeError, ValueError):
                    max_px = 768
                self._send(*api.snapshot(max(64, min(4096, max_px))))
            elif path == "/state":
                self._send(*api.state())
            elif path == "/poses":
                self._send(*api.poses())
            elif path == "/detections":
                self._send(*api.detections())
            elif path == "/arm":
                self._send(*api.get_arm())
            else:
                self._send(404, "application/json", _bytes({"ok": False, "error": "not found"}))
        except Exception as e:      # never let one bad request kill the worker
            self._send(500, "application/json", _bytes({"ok": False, "error": str(e)}))

    def do_POST(self):
        body = self._body_json()          # always drain the body (even on 401) → keep-alive stays synced
        if not self._auth():
            return
        api = self.server.api
        path = self.path.split("?")[0]
        try:
            if path == "/dof":
                self._send(*api.set_dof(body))
            elif path == "/pose":
                self._send(*api.apply_pose(body))
            elif path == "/clip":
                self._send(*api.play_clip(body))
            elif path == "/home":
                self._send(*api.home(body))
            elif path == "/torque":
                self._send(*api.torque(body))
            elif path == "/estop":
                self._send(*api.estop())
            elif path == "/arm":
                self._send(*api.set_arm(body))
            else:
                self._send(404, "application/json", _bytes({"ok": False, "error": "not found"}))
        except Exception as e:
            self._send(500, "application/json", _bytes({"ok": False, "error": str(e)}))


class ControlServer:
    """Serves the ControlApi on 127.0.0.1 in a daemon thread. `invoke` marshals onto the Qt
    thread (pass `bridge.qt_invoke` in the app; a synchronous stub in tests)."""

    def __init__(self, bridge: Any, invoke: Callable[[Callable[[], Any]], Any], token: str,
                 host: str = "127.0.0.1", port: int = 8765,
                 snapshot: Callable[[int], bytes] | None = None,
                 log: Callable[[dict], None] | None = None):
        self._httpd = ThreadingHTTPServer((host, port), _Handler)
        self._httpd.api = ControlApi(bridge, invoke, snapshot=snapshot, log=log)
        self._httpd.token = token
        self._thread: threading.Thread | None = None

    @property
    def port(self) -> int:
        return self._httpd.server_address[1]

    def start(self) -> None:
        self._thread = threading.Thread(target=self._httpd.serve_forever,
                                        name="handlab-control", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        try:
            self._httpd.shutdown()
        except Exception:
            pass
        self._httpd.server_close()


def write_discovery(port: int, token: str) -> Path:
    """Write host/port/token where the MCP server (a separate process) can find them. The token
    grants control of the hand, so create the dir + file 0o600 FROM THE START (os.open with the
    mode) instead of writing world-readable then tightening — no exposure window on a shared host."""
    d = Path.home() / ".handlab"
    d.mkdir(parents=True, exist_ok=True, mode=0o700)
    try:
        os.chmod(d, 0o700)              # tighten a pre-existing loose dir
    except OSError:
        pass
    f = d / "agent.json"
    payload = json.dumps({"host": "127.0.0.1", "port": port, "token": token}).encode()
    fd = os.open(f, os.O_CREAT | os.O_WRONLY | os.O_TRUNC, 0o600)
    try:
        os.write(fd, payload)
    finally:
        os.close(fd)
    try:
        os.chmod(f, 0o600)             # repair a pre-existing 0o644 file (os.open won't chmod it)
    except OSError:
        pass
    return f
