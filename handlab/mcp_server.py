"""mcp_server.py — the handlab MCP server: how Claude drives the hand.

A thin, stateless stdio MCP process. It owns NEITHER the serial port NOR the camera — it only
calls the token-gated HTTP control API that the running handlab GUI already exposes on
127.0.0.1 (see `control_server.py`). The GUI stays the one authoritative
process at the port, enforces the ARM gate + the force/velocity caps + the effective-range clamp
+ the phased safe-home, and a human watches the twin/feed and can STOP the whole time.

Discovery: the GUI writes host/port/token to `~/.handlab/agent.json` (0o600) at startup; this
server reads it per request, so it survives the GUI restarting on a new ephemeral port.

Run it (the way Claude Desktop / Claude Code launch it):
    ~/.venvs/handlab/bin/python -m handlab.mcp_server
"""
from __future__ import annotations

import json
import urllib.error
import urllib.request
from pathlib import Path

from mcp.server.fastmcp import FastMCP, Image

# Module global (not a constant) so tests can monkeypatch it at `handlab.mcp_server.DISCOVERY`.
DISCOVERY = Path.home() / ".handlab" / "agent.json"

INSTRUCTIONS = (
    "You operate a real 3-finger tendon-driven robotic hand (or its MuJoCo twin) through handlab. "
    "Work in perceive → act → observe cycles: call `get_state` / `snapshot` to see, then ONE small "
    "`set_dof` or `apply_pose`, then `snapshot` again to check. Addressing: finger ∈ {index, mid, "
    "thumb}, dof ∈ {spread, flex, curl}; `get_state` gives the exact current angle + effective "
    "range (degrees) for each, and the saved pose/clip skill library. Positive flex/curl = the "
    "finger closes. The hand starts DISARMED and you CANNOT arm it — a human arms it in the Agent "
    "tab; until then every move is refused (ok:false). Move gently and in small steps. ALWAYS "
    "finish a session with `safe_home`. If anything looks wrong, call `estop` immediately."
)

mcp = FastMCP("handlab", instructions=INSTRUCTIONS)


# ─────────────────────────────── HTTP plumbing (testable) ───────────────────────────────
class _NotRunning(Exception):
    """handlab GUI unreachable — no discovery file, or the connection was refused."""


def _discovery() -> tuple[str, int, str]:
    try:
        d = json.loads(DISCOVERY.read_text())
        return d["host"], int(d["port"]), d["token"]
    except FileNotFoundError:
        raise _NotRunning(
            "handlab is not running (no ~/.handlab/agent.json). "
            "Start it first:  ~/.venvs/handlab/bin/python -m handlab")
    except Exception as e:  # malformed / partial file
        raise _NotRunning(f"cannot read ~/.handlab/agent.json: {e}")


def _req(method: str, path: str, body: dict | None = None,
         want_bytes: bool = False, timeout: float = 8.0):
    """One HTTP call to the GUI's control API. Returns the response text (or raw bytes). A server
    that ANSWERS with an error status (401/409/503) has its body passed through — only a genuinely
    unreachable app raises _NotRunning."""
    host, port, token = _discovery()
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(
        f"http://{host}:{port}{path}", data=data, method=method,
        headers={"X-Handlab-Token": token, "Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            raw = r.read()
    except urllib.error.HTTPError as e:      # the server replied (bad token, disarmed, no frame) → keep its body
        raw = e.read()
    except urllib.error.URLError as e:       # refused / no route → the app isn't listening
        raise _NotRunning(
            f"handlab not reachable on 127.0.0.1:{port} — is the app running? ({e.reason})")
    return raw if want_bytes else raw.decode()


def _guard(fn):
    """A tool body that turns _NotRunning into a clean JSON error string instead of crashing the
    MCP session — Claude sees a readable message and can tell the human to start the app."""
    try:
        return fn()
    except _NotRunning as e:
        return json.dumps({"ok": False, "error": str(e)})


# ─────────────────────────────── observe (read-only) ───────────────────────────────
@mcp.tool()
def get_state() -> str:
    """Snapshot of the hand as JSON: driver, the ARMED flag, online servos, and every finger/DOF
    with its current angle (deg) + effective range (the addressing AND the limits), plus the saved
    pose/clip skill library. Read-only — call this FIRST to perceive before acting."""
    return _guard(lambda: _req("GET", "/state"))


@mcp.tool()
def list_poses() -> str:
    """The saved skill library as JSON: each entry's name + kind ('pose' = a static hand shape via
    apply_pose, 'clip' = a recorded motion via play_clip)."""
    return _guard(lambda: _req("GET", "/poses"))


@mcp.tool()
def snapshot(max_px: int = 768):
    """The current view as an image — the MuJoCo twin now (a webcam later). OBSERVE with this
    before and after every action to confirm what actually happened."""
    def go():
        raw = _req("GET", f"/snapshot.jpg?max={int(max_px)}", want_bytes=True)
        if raw[:3] == b"\xff\xd8\xff":                     # JPEG magic → real frame
            return Image(data=raw, format="jpeg")
        try:                                               # otherwise it's a JSON error (e.g. 503 no frame yet)
            return raw.decode()
        except UnicodeDecodeError:
            return json.dumps({"ok": False, "error": "no image"})
    return _guard(go)


@mcp.tool()
def detect_markers() -> str:
    """ArUco markers currently visible in the CAMERA frame, as JSON. Contract per marker: `id` +
    pixel `corners`/`center` ALWAYS; `pose` (camera-frame `rvec`/`tvec` [m] + `distance_m`) only
    when the camera is calibrated AND the marker's edge length is registered; `pose.in_base`
    ({pos [m], rvec} in the HAND frame) only when the reference marker (`is_reference`, the palm
    base) is ALSO visible in the same frame with a trustworthy pose — treat in_base as optional
    and re-frame so the reference is in view when it's missing. If `pose.ambiguous` is true, the
    ORIENTATION may be flipped (~180°) — trust position/distance only. Read-only: needs the camera
    Started in the Agent tab (empty when off or opencv isn't installed)."""
    return _guard(lambda: _req("GET", "/detections"))


# ─────────────────────────────── act (need ARMED; a human arms in the Agent tab) ───────────────────────────────
@mcp.tool()
def set_dof(finger: str, dof: str, deg: float) -> str:
    """Command ONE degree-of-freedom to an angle. finger ∈ {index, mid, thumb}, dof ∈ {spread,
    flex, curl} (see get_state for the exact addressing + limits). The angle is CLAMPED to the
    DOF's effective range; positive flex/curl closes the finger. Requires ARMED (a human arms the
    hand in the Agent tab) — a disarmed hand refuses with ok:false."""
    return _guard(lambda: _req("POST", "/dof", {"finger": finger, "dof": dof, "deg": deg}))


@mcp.tool()
def apply_pose(name: str) -> str:
    """Recall a saved POSE (a whole-hand target shape) by name — see list_poses. Goes through the
    same clamped, phased path a human uses. Requires ARMED."""
    return _guard(lambda: _req("POST", "/pose", {"name": name}))


@mcp.tool()
def play_clip(name: str, loop: bool = False) -> str:
    """Play a recorded motion CLIP ('skill') by name; set loop=true to repeat. See list_poses.
    Requires ARMED."""
    return _guard(lambda: _req("POST", "/clip", {"name": name, "loop": bool(loop)}))


@mcp.tool()
def safe_home(finger: str = "") -> str:
    """Collision-aware phased return to the open rest pose (curl → flex → spread, so fingertips
    separate first). '' or 'all' homes the whole hand; else a finger id (index/mid/thumb). ALWAYS
    finish a session with this. Requires ARMED."""
    return _guard(lambda: _req("POST", "/home", {"finger": finger}))


@mcp.tool()
def torque(on: bool) -> str:
    """Enable/disable holding torque on all servos. Enabling requires ARMED (a settle can move the
    hand on re-engage); disabling is always allowed (it makes the hand limp/safe)."""
    return _guard(lambda: _req("POST", "/torque", {"on": bool(on)}))


@mcp.tool()
def estop() -> str:
    """EMERGENCY STOP — abort any motion, drop torque on every servo (elastics stretch the hand
    open), and disarm. Always allowed. Use the instant anything looks wrong; a human re-arms."""
    return _guard(lambda: _req("POST", "/estop"))


@mcp.tool()
def disarm() -> str:
    """Drop the software ARM: no further moves are accepted (torque is NOT cut — use estop for
    that). You cannot re-arm — arming is a deliberate human action in the Agent tab."""
    return _guard(lambda: _req("POST", "/arm", {"on": False}))


def main() -> None:
    mcp.run()   # stdio transport — how Claude Desktop / Claude Code launch this process


if __name__ == "__main__":
    main()
