"""UDP receiver for an EXTERNAL hand-tracking process (the decoupled WiLoR tracker in `tracking/`).

WiLoR is torch-heavy and can't live in the lean `.app`, so it runs as its own process and STREAMS
per-DOF normalized targets here over a local UDP socket. The teleop loop polls `latest()` exactly the
way the in-process MediaPipe path polls `cam.latest_landmarks()` — so downstream (OneEuro smoothing,
`_teleop_dof_deg`, `_teleop_goals` → `set_goal_positions`, the `_estopped` gate) is reused unchanged.

Wire format — one JSON object per UDP datagram (latest-wins; UDP drops are harmless for a pose stream):
    {"t": <sender_monotonic_s>, "fingers": {"index": {"flex": 0.5, "curl": 0.3, "spread": -0.1},
                                            "mid": {...}, "ring": {...}, "thumb": {...}}}
Values are norms: flex/curl ∈ [0,1] (0 = open/rest, 1 = closed), spread ∈ [-1,1] (bipolar). Finger
keys are ROBOT finger ids (the tracker does the human→robot finger mapping).
"""
from __future__ import annotations

import json
import socket
import threading
import time

DEFAULT_PORT = 8770
_STALE_S = 1e9              # sentinel age when nothing has arrived yet


class TeleopStreamReceiver:
    """Binds a UDP socket and keeps the most recent frame. Thread-safe; poll `latest()`."""

    def __init__(self, port: int = DEFAULT_PORT, host: str = "127.0.0.1"):
        self._addr = (host, int(port))
        self._sock: socket.socket | None = None
        self._thread: threading.Thread | None = None
        self._running = False
        self._lock = threading.Lock()
        self._latest: dict | None = None       # {finger: {dof: float}}
        self._stamp = 0.0                       # time.monotonic() of the last accepted frame

    @property
    def port(self) -> int:
        return self._addr[1]

    def start(self) -> None:
        if self._running:
            return
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(self._addr)
        s.settimeout(0.5)                       # so the loop can notice `_running` going False
        self._sock = s
        self._running = True
        self._thread = threading.Thread(target=self._loop, name="teleop-stream", daemon=True)
        self._thread.start()

    def _loop(self) -> None:
        while self._running:
            sock = self._sock
            if sock is None:
                break
            try:
                data, _ = sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            norms = self._parse(data)
            if norms is not None:
                with self._lock:
                    self._latest = norms
                    self._stamp = time.monotonic()

    @staticmethod
    def _parse(data: bytes) -> dict | None:
        try:
            msg = json.loads(data.decode("utf-8"))
        except Exception:
            return None
        fingers = msg.get("fingers") if isinstance(msg, dict) else None
        if not isinstance(fingers, dict):
            return None
        out: dict[str, dict[str, float]] = {}
        for finger, dofs in fingers.items():
            if not isinstance(dofs, dict):
                continue
            clean = {}
            for dof, val in dofs.items():
                try:
                    clean[str(dof)] = float(val)
                except (TypeError, ValueError):
                    continue
            if clean:
                out[str(finger)] = clean
        return out or None

    def latest(self) -> tuple[dict | None, float]:
        """(norms | None, age_seconds). Age is a huge sentinel until the first frame lands, so the
        caller's staleness guard naturally skips driving on nothing/old data."""
        with self._lock:
            if self._latest is None:
                return None, _STALE_S
            return self._latest, time.monotonic() - self._stamp

    def stop(self) -> None:
        self._running = False
        s, self._sock = self._sock, None
        if s is not None:
            try:
                s.close()
            except Exception:
                pass
        t, self._thread = self._thread, None
        if t is not None and t.is_alive():
            t.join(timeout=1.0)
