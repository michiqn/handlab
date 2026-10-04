"""Teleop shared seam + safety hooks, headless.

The WiLoR stream path (frame builder + step drive/hold) is covered in test_teleop_stream. Here we
test the mode-agnostic pieces: the shared `_teleop_apply` (identity gate + E-STOP write gate) and
that `agentEstop` actually stops the motion loop. A bare `Bridge.__new__(Bridge)` carries only the
state these methods read -- no Qt loop, no camera.
"""
import json
import types

from handlab import compose, library_io
from handlab.bridge import Bridge


def _teleop_bridge():
    b = library_io.load_build("claw3f")
    sm = compose.derive_servo_map(b)
    ft = {f.id: compose._try_load(f.type) for f in b.fingers}
    fake = Bridge.__new__(Bridge)
    fake._build = b
    fake._servo_map = sm
    fake._eff_rad = {(f.id, d.key): d.range
                     for f in b.fingers for _sid, d in zip(f.servos, ft[f.id].dofs)}
    fake._teleop_on = False
    fake._teleop_last_t = 0.0
    fake._teleop_filt = {}
    fake._estopped = False
    fake._teleop_prev_frame = None
    frames, batches = [], []
    fake._driver = types.SimpleNamespace(set_goal_positions=lambda g: batches.append(dict(g)))
    fake.teleopFrame = types.SimpleNamespace(emit=frames.append)
    fake.teleopStateChanged = types.SimpleNamespace(emit=lambda: None)
    fake._teleop_timer = types.SimpleNamespace(start=lambda *a: None, stop=lambda: None,
                                               setInterval=lambda *a: None)
    fake._timer = types.SimpleNamespace(setInterval=lambda *a: None)
    fake._frames, fake._batches = frames, batches
    return fake, b


def _close_frame(build):
    """A concrete {socket:[deg]} frame: +20° on every servo (inside every claw's effective range)."""
    return {f.socket: [20 for _ in f.servos] for f in build.fingers}


def test_apply_still_frame_emits_once_but_writes_each_step():
    """Identity gate: an unchanged frame is not re-emitted to QML, but the driver is still called."""
    fake, b = _teleop_bridge()
    frame = _close_frame(b)
    Bridge._teleop_apply(fake, frame)
    Bridge._teleop_apply(fake, dict(frame))
    Bridge._teleop_apply(fake, dict(frame))
    assert len(fake._frames) == 1                   # same frame -> one display emit
    assert len(fake._batches) == 3                  # driver still called each step


def test_apply_estopped_blocks_writes():
    fake, b = _teleop_bridge()
    fake._estopped = True
    Bridge._teleop_apply(fake, _close_frame(b))
    assert fake._batches == []                       # E-STOP flag gates the direct write path


def test_estop_stops_teleop_loop():
    fake, _b = _teleop_bridge()
    fake._teleop_on = True
    fake._driver = None
    fake._armed = True
    fake._emit_armed = lambda: None
    fake.stopHoming = lambda: None
    stops = []
    fake._teleop_timer = types.SimpleNamespace(start=lambda *a: None, stop=lambda: stops.append(1))
    r = json.loads(Bridge.agentEstop(fake))
    assert r["ok"] and fake._teleop_on is False and stops     # E-STOP killed the loop
