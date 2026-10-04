"""Phase-1 agent control plane: the arm gate, the deg→clamp, and the token-gated HTTP surface.

Two layers, both headless (no Qt event loop, no hardware):
  * the Bridge *agent* slots (gating + effective-range clamp) exercised on a bare instance;
  * ControlApi / ControlServer routing + policy over a FakeBridge with a synchronous invoker.
"""
import json
import math
import os
import stat
import threading
import urllib.error
import urllib.request
from pathlib import Path

import pytest

from PySide6.QtCore import QCoreApplication

from handlab import compose, library_io
from handlab.hal.base import rad_to_ticks
from handlab.control_server import ControlApi, ControlServer, QtInvoker, write_discovery


def _qapp():
    return QCoreApplication.instance() or QCoreApplication([])


# ─────────────────────────── Bridge agent slots (gating + clamp) ───────────────────────────
def _agent_bridge():
    """A bare Bridge with just the state the agent slots read + a goal-capturing fake driver."""
    from handlab.bridge import Bridge
    b = library_io.load_build("claw3f")
    sm = compose.derive_servo_map(b)
    ft = {f.id: compose._try_load(f.type) for f in b.fingers}
    fake = Bridge.__new__(Bridge)
    fake._build = b
    fake._servo_map = sm
    fake._armed = False
    fake._online = set(s.id for s in sm.servos)
    fake._poses = [{"name": "open", "kind": "pose"}]
    fake._last_tele = {}
    fake._eff_rad = {(f.id, d.key): d.range
                     for f in b.fingers for _sid, d in zip(f.servos, ft[f.id].dofs)}
    goals = []

    class _Driver:
        driver_type = "mock"

        def set_goal_position(self, sid, ticks):
            goals.append((sid, ticks))

        def set_torque(self, sid, en):
            goals.append(("t", sid, en))

    fake._driver = _Driver()
    fake._goals = goals
    fake._emit_armed = lambda: None            # no QObject → stub the signal emit
    return fake, b


def test_agent_setdof_gates_and_clamps():
    from handlab.bridge import Bridge
    fake, _b = _agent_bridge()

    # disarmed: refused, nothing reaches the driver
    r = json.loads(Bridge.agentSetDof(fake, "index", "flex", 40))
    assert r["ok"] is False and fake._goals == []

    fake._armed = True
    # index flex base range −70..120°: 40° is inside → not clamped, servo id 2 (index flex)
    r = json.loads(Bridge.agentSetDof(fake, "index", "flex", 40))
    assert r["ok"] and not r["clamped"]
    assert fake._goals[-1] == (2, rad_to_ticks(math.radians(40)))

    # way past max → clamped to the ceiling (120°)
    r = json.loads(Bridge.agentSetDof(fake, "index", "flex", 999))
    assert r["clamped"] and r["deg"] == 120
    assert fake._goals[-1] == (2, rad_to_ticks(math.radians(120)))

    # unknown dof → error, no goal written
    n = len(fake._goals)
    assert json.loads(Bridge.agentSetDof(fake, "index", "wobble", 10))["ok"] is False
    assert len(fake._goals) == n


def test_agent_torque_gate():
    from handlab.bridge import Bridge
    fake, b = _agent_bridge()
    # disarmed: enabling refused, no torque writes
    assert json.loads(Bridge.agentTorque(fake, True))["ok"] is False
    assert [c for c in fake._goals if c[0] == "t"] == []
    # disabling always allowed → every servo torque-off
    assert json.loads(Bridge.agentTorque(fake, False))["ok"] is True
    offs = [c for c in fake._goals if c[0] == "t"]
    assert len(offs) == b.motor_count and all(c[2] is False for c in offs)


def test_agent_estop_always_disarms_and_drops_torque():
    from handlab.bridge import Bridge
    fake, b = _agent_bridge()
    fake._armed = True
    fake.stopHoming = lambda: None
    r = json.loads(Bridge.agentEstop(fake))
    assert r["ok"] and r["armed"] is False and fake._armed is False
    offs = [c for c in fake._goals if c[0] == "t"]
    assert len(offs) == b.motor_count and all(c[2] is False for c in offs)


def test_agent_pose_and_home_gate_and_resolve():
    from handlab.bridge import Bridge
    fake, _b = _agent_bridge()
    called = []
    fake.applyPose = lambda n: called.append(("pose", n))
    fake.safeHome = lambda o: called.append(("home", o))

    # disarmed: both refused
    assert json.loads(Bridge.agentApplyPose(fake, "open"))["ok"] is False
    assert json.loads(Bridge.agentSafeHome(fake, "index"))["ok"] is False
    assert called == []

    fake._armed = True
    assert json.loads(Bridge.agentApplyPose(fake, "open"))["ok"] is True
    assert ("pose", "open") in called
    assert json.loads(Bridge.agentApplyPose(fake, "ghost"))["ok"] is False   # unknown pose
    assert json.loads(Bridge.agentSafeHome(fake, "index"))["ok"] is True
    assert ("home", 0) in called                                             # index → ordinal 0
    json.loads(Bridge.agentSafeHome(fake, "all"))
    assert ("home", -1) in called                                            # whole hand
    assert json.loads(Bridge.agentSafeHome(fake, "pinky"))["ok"] is False    # unknown finger


def test_agent_state_shape():
    from handlab.bridge import Bridge
    fake, b = _agent_bridge()
    st = json.loads(Bridge.agentState(fake))
    assert st["armed"] is False and st["driver"] == "mock"
    assert len(st["dofs"]) == b.motor_count
    d0 = next(x for x in st["dofs"] if x["finger"] == "index" and x["dof"] == "flex")
    assert d0["servo"] == 2 and d0["range_deg"] == [-70, 120]
    assert st["poses"] == [{"name": "open", "kind": "pose"}]


# ─────────────────────────── ControlApi + HTTP (policy + routing) ───────────────────────────
class FakeBridge:
    """The agent-slot contract in plain Python — lets the control layer be tested without Qt."""

    def __init__(self):
        self.armed = False
        self.calls = []

    def agentState(self):
        return json.dumps({"driver": "mock", "armed": self.armed, "online": [],
                           "servos": {}, "dofs": [], "poses": [{"name": "open", "kind": "pose"}]})

    def agentSetDof(self, finger, dof, deg):
        if not self.armed:
            return json.dumps({"ok": False, "error": "disarmed"})
        self.calls.append(("dof", finger, dof, deg))
        return json.dumps({"ok": True, "finger": finger, "dof": dof, "deg": deg})

    def agentApplyPose(self, name):
        if not self.armed:
            return json.dumps({"ok": False, "error": "disarmed"})
        self.calls.append(("pose", name))
        return json.dumps({"ok": True, "pose": name})

    def agentPlayClip(self, name, loop=False):
        if not self.armed:
            return json.dumps({"ok": False, "error": "disarmed"})
        self.calls.append(("clip", name, loop))
        return json.dumps({"ok": True, "clip": name, "loop": loop})

    def agentSafeHome(self, finger):
        if not self.armed:
            return json.dumps({"ok": False, "error": "disarmed"})
        self.calls.append(("home", finger))
        return json.dumps({"ok": True})

    def agentTorque(self, on):
        if on and not self.armed:
            return json.dumps({"ok": False, "error": "disarmed"})
        self.calls.append(("torque", on))
        return json.dumps({"ok": True, "torque": on})

    def agentEstop(self):
        self.armed = False
        self.calls.append(("estop",))
        return json.dumps({"ok": True, "estop": True, "armed": False})

    def setArmed(self, on):
        self.armed = bool(on)

    def agent_snapshot_jpeg(self, max_px=768):
        return b"\xff\xd8\xff" + b"jpeg"


def _api(bridge):
    return ControlApi(bridge, invoke=lambda fn: fn())   # synchronous invoke — no Qt needed


def test_arm_gate_blocks_moves_when_disarmed():
    b = FakeBridge()
    st, _ct, body = _api(b).set_dof({"finger": "index", "dof": "flex", "deg": 40})
    assert st == 409 and json.loads(body)["ok"] is False
    assert b.calls == []


def test_arm_then_move():
    b = FakeBridge()
    b.setArmed(True)
    st, _ct, body = _api(b).set_dof({"finger": "index", "dof": "flex", "deg": 40})
    assert st == 200 and json.loads(body)["ok"] is True
    assert ("dof", "index", "flex", 40.0) in b.calls


def test_estop_always_allowed_and_disarms():
    b = FakeBridge()
    b.setArmed(True)
    st, _ct, body = _api(b).estop()
    assert st == 200 and json.loads(body)["armed"] is False and b.armed is False


def test_remote_can_disarm_but_never_arm():
    b = FakeBridge()
    b.setArmed(True)
    api = _api(b)
    st, _ct, _body = api.set_arm({"on": True})
    assert st == 403 and b.armed is True                 # remote ARM refused
    st, _ct, _body = api.set_arm({"on": False})
    assert st == 200 and b.armed is False                # remote DISARM allowed


def test_torque_off_allowed_disarmed_but_on_blocked():
    b = FakeBridge()                                     # disarmed
    api = _api(b)
    assert api.torque({"on": True})[0] == 409
    assert api.torque({"on": False})[0] == 200


def test_snapshot_returns_jpeg():
    st, ct, body = _api(FakeBridge()).snapshot()
    assert st == 200 and ct == "image/jpeg" and body[:3] == b"\xff\xd8\xff"


def test_set_dof_rejects_non_numeric_deg():
    b = FakeBridge()
    b.setArmed(True)
    assert _api(b).set_dof({"finger": "index", "dof": "flex", "deg": "lots"})[0] == 400


def test_http_token_and_routing():
    b = FakeBridge()
    b.setArmed(True)
    srv = ControlServer(b, invoke=lambda fn: fn(), token="secret", port=0)
    srv.start()
    try:
        base = f"http://127.0.0.1:{srv.port}"

        # missing token → 401
        with pytest.raises(urllib.error.HTTPError) as ei:
            urllib.request.urlopen(base + "/state")
        assert ei.value.code == 401

        # with token → 200
        req = urllib.request.Request(base + "/state", headers={"X-Handlab-Token": "secret"})
        with urllib.request.urlopen(req) as r:
            assert r.status == 200 and json.loads(r.read())["armed"] is True

        # POST /dof reaches the bridge
        req = urllib.request.Request(
            base + "/dof", method="POST",
            data=json.dumps({"finger": "index", "dof": "flex", "deg": 30}).encode(),
            headers={"X-Handlab-Token": "secret", "Content-Type": "application/json"})
        with urllib.request.urlopen(req) as r:
            assert json.loads(r.read())["ok"] is True
        assert ("dof", "index", "flex", 30.0) in b.calls

        # snapshot bytes come back as image/jpeg
        req = urllib.request.Request(base + "/snapshot.jpg", headers={"X-Handlab-Token": "secret"})
        with urllib.request.urlopen(req) as r:
            assert r.headers["Content-Type"] == "image/jpeg" and r.read()[:3] == b"\xff\xd8\xff"

        # unknown path → 404 (worker survives)
        req = urllib.request.Request(base + "/nope", headers={"X-Handlab-Token": "secret"})
        with pytest.raises(urllib.error.HTTPError) as ei:
            urllib.request.urlopen(req)
        assert ei.value.code == 404
    finally:
        srv.stop()


def test_http_500_when_handler_raises_but_worker_survives():
    def boom(_fn):
        raise RuntimeError("boom")
    srv = ControlServer(FakeBridge(), invoke=boom, token="secret", port=0)
    srv.start()
    try:
        base = f"http://127.0.0.1:{srv.port}"
        req = urllib.request.Request(base + "/state", headers={"X-Handlab-Token": "secret"})
        with pytest.raises(urllib.error.HTTPError) as ei:
            urllib.request.urlopen(req)
        assert ei.value.code == 500 and b"boom" in ei.value.read()
        # a second request still gets served → the worker didn't die on the exception
        req = urllib.request.Request(base + "/state", headers={"X-Handlab-Token": "secret"})
        with pytest.raises(urllib.error.HTTPError) as ei2:
            urllib.request.urlopen(req)
        assert ei2.value.code == 500
    finally:
        srv.stop()


def test_clip_route_reaches_bridge():
    b = FakeBridge()
    b.setArmed(True)
    st, _ct, body = _api(b).play_clip({"name": "wave", "loop": True})
    assert st == 200 and json.loads(body)["ok"] is True
    assert ("clip", "wave", True) in b.calls


def test_snapshot_503_when_no_frame():
    class NoFrame(FakeBridge):
        def agent_snapshot_jpeg(self, max_px=768):
            return b""
    st, _ct, body = _api(NoFrame()).snapshot()
    assert st == 503 and json.loads(body)["ok"] is False


def test_snapshot_max_px_query_flows_to_bridge():
    """The MCP client's ?max= must reach agent_snapshot_jpeg (it was dropped when do_GET stripped
    the query string). Bad/missing values fall back to the 768 default."""
    seen = []

    class RecBridge(FakeBridge):
        def agent_snapshot_jpeg(self, max_px=768):
            seen.append(max_px)
            return b"\xff\xd8\xff" + b"x"

    srv = ControlServer(RecBridge(), invoke=lambda fn: fn(), token="secret", port=0)
    srv.start()
    try:
        base = f"http://127.0.0.1:{srv.port}"
        h = {"X-Handlab-Token": "secret"}
        for q, want in (("?max=256", 256), ("?max=foo", 768), ("", 768), ("?max=99999", 4096)):
            urllib.request.urlopen(
                urllib.request.Request(base + "/snapshot.jpg" + q, headers=h)).read()
            assert seen[-1] == want, (q, seen[-1])
    finally:
        srv.stop()


def test_mutating_routes_log_including_refusals():
    """The Agent-tab log gets one line per MUTATING command — even a refused one — but reads don't."""
    b = FakeBridge()
    b.setArmed(True)
    cap = []
    api = ControlApi(b, invoke=lambda fn: fn(), log=cap.append)

    api.set_dof({"finger": "index", "dof": "flex", "deg": 40})
    assert cap[-1] == {"tool": "set_dof", "detail": "index/flex→40°", "ok": True}

    # a refused (disarmed) command is STILL logged, with ok False
    b.setArmed(False)
    api.set_dof({"finger": "index", "dof": "flex", "deg": 40})
    assert cap[-1] == {"tool": "set_dof", "detail": "index/flex→40°", "ok": False}

    # remote disarm is logged
    api.set_arm({"on": False})
    assert cap[-1] == {"tool": "disarm", "detail": "", "ok": True}

    # read routes are NOT logged (no spam)
    n = len(cap)
    api.state()
    api.poses()
    assert len(cap) == n


def test_log_is_optional_default_off():
    """Without a log callback (the test default), nothing breaks — existing behavior is unchanged."""
    b = FakeBridge()
    b.setArmed(True)
    st, _ct, body = _api(b).set_dof({"finger": "index", "dof": "flex", "deg": 10})
    assert st == 200 and json.loads(body)["ok"] is True


# ─────────────────────────── QtInvoker (the thread-safety crux) ───────────────────────────
def test_qtinvoker_same_thread_inline_and_reraise():
    _qapp()
    inv = QtInvoker()
    # same-thread emit → AutoConnection runs inline, no event loop needed
    assert inv.invoke(lambda: 7) == 7
    # the callable's exception is carried back and re-raised on the caller
    with pytest.raises(ZeroDivisionError):
        inv.invoke(lambda: 1 / 0)


def test_qtinvoker_timeout_cancels_and_never_actuates():
    app = _qapp()
    inv = QtInvoker()
    ran = []
    outcome = {}

    def worker():
        try:
            # emitted from a WORKER thread → queued to the (main-thread) invoker, which is not
            # running an event loop, so it is never delivered → invoke must time out + cancel.
            inv.invoke(lambda: ran.append(1), timeout=0.3)
        except TimeoutError:
            outcome["timeout"] = True
        except BaseException as e:                     # noqa: BLE001
            outcome["other"] = repr(e)

    t = threading.Thread(target=worker)
    t.start()
    t.join(3.0)
    assert outcome.get("timeout") is True, outcome
    assert ran == []                                    # timed-out call never ran
    app.processEvents()                                 # deliver the queued (now-cancelled) call
    assert ran == []                                    # …and it STILL must not actuate


# ─────────────────────────── agent slots: clips vs poses, offline telemetry ───────────────────────────
def test_agent_pose_rejects_clip_and_clip_plays():
    from handlab.bridge import Bridge
    fake, _b = _agent_bridge()
    fake._armed = True
    fake._poses = [{"name": "open", "kind": "pose"}, {"name": "wave", "kind": "clip"}]
    played = []
    fake.applyPose = lambda n: played.append(("pose", n))
    fake.playClip = lambda n, loop: played.append(("clip", n, loop))

    assert json.loads(Bridge.agentApplyPose(fake, "wave"))["ok"] is False   # pose slot refuses a clip
    assert json.loads(Bridge.agentPlayClip(fake, "open", False))["ok"] is False  # clip slot refuses a pose
    assert json.loads(Bridge.agentPlayClip(fake, "wave", True))["ok"] is True
    assert played == [("clip", "wave", True)]           # only the valid call ran


def test_agent_state_offline_servo_deg_is_null():
    from handlab.bridge import Bridge
    fake, b = _agent_bridge()
    sid_on = b.fingers[0].servos[1]     # index flex
    sid_off = b.fingers[0].servos[0]    # index spread
    fake._last_tele = {
        str(sid_on): {"pos": rad_to_ticks(math.radians(30)), "online": True},
        str(sid_off): {"pos": 0, "online": False},
    }
    st = json.loads(Bridge.agentState(fake))
    dmap = {(d["finger"], d["dof"]): d for d in st["dofs"]}
    assert abs(dmap[("index", "flex")]["deg"] - 30) < 0.2
    assert dmap[("index", "spread")]["deg"] is None     # offline → null, not a phantom −180°


def test_write_discovery_payload_and_perms(tmp_path, monkeypatch):
    monkeypatch.setattr(Path, "home", lambda: tmp_path)
    f = write_discovery(8765, "tok")
    assert json.loads(f.read_text()) == {"host": "127.0.0.1", "port": 8765, "token": "tok"}
    assert stat.S_IMODE(os.stat(f).st_mode) == 0o600   # token file never world-readable
