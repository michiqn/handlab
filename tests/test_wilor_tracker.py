"""The human half of the WiLoR teleop (tracking/wilor_teleop.py) — the normalizer only.

The tracker is a separate process + venv (torch/WiLoR is NOT imported here; `normalize` needs only
numpy). What matters for the robot: an operator capture must read NEUTRAL at the open hand, a
degenerate spread capture must not manufacture a bias, and the thumb's 'opp' aperture must span its
two anchors — that scalar is the ONLY thumb channel handlab drives (see bridge_teleop).
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tracking"))
w = pytest.importorskip("wilor_teleop")


def _cal(spread_span, opp_open=0.9, opp_pinch=0.15, peace_span=None):
    """open/fist/spread/pinch (+ optional ✌️) captures, with a tunable mid-spread span."""
    open_ = {"index": {"flex": 0.0, "curl": 10.0, "spread": -40.0},
             "mid":   {"flex": 0.0, "curl": 10.0, "spread": -20.0},
             "ring":  {"flex": 0.0, "curl": 10.0, "spread": 3.0},
             "thumb": {"flex": 0.0, "curl": 10.0, "spread": -47.0, "opp_d": opp_open}}
    fist = {f: dict(v, flex=v["flex"] + 60.0, curl=v["curl"] + 100.0) for f, v in open_.items()}
    spread = {"index": dict(open_["index"], spread=-61.0),          # 21° span — a real range
              "mid":   dict(open_["mid"], spread=-20.0 + spread_span),
              "ring":  dict(open_["ring"], spread=18.0),            # +15°, finger straight
              "thumb": dict(open_["thumb"], spread=-56.0)}
    pinch = {"thumb": dict(open_["thumb"], opp_d=opp_pinch)}
    cal = {"open": open_, "fist": fist, "spread": spread, "pinch": pinch}
    if peace_span is not None:                                      # ✌️: the middle finger swings out
        cal["peace"] = {"index": dict(open_["index"], spread=-54.0),
                        "mid":   dict(open_["mid"], spread=-20.0 + peace_span),
                        # ...while the ring finger is CURLED here and its angle drifts the other way
                        "ring":  dict(open_["ring"], flex=70.0, spread=-15.0),
                        "thumb": dict(open_["thumb"])}
    return cal


def _raw(cal, key="open", **over):
    raw = {f: dict(v) for f, v in cal[key].items()}
    for finger, dofs in over.items():
        raw[finger].update(dofs)
    return raw


def test_open_capture_reads_neutral():
    """Every dof is 0 at the operator's own open hand — the anchor the robot's home maps to."""
    cal = _cal(spread_span=-18.0)
    out = w.normalize(_raw(cal), cal)
    for finger, dofs in out.items():
        for dof in ("flex", "curl", "spread"):
            assert abs(dofs[dof]) < 1e-6, (finger, dof)


def test_degenerate_spread_span_holds_neutral_instead_of_a_biased_default():
    """A spread capture narrower than MIN_SPREAD_SPAN is tracker noise. Falling back to the
    0-centered DEFAULT_RANGES then produced a large CONSTANT offset (the middle finger sat at
    s≈-0.7 dead straight); it must read neutral instead — and a real span must still map."""
    cal = _cal(spread_span=2.5)                                   # middle finger: 2.5° = noise
    assert abs(w.normalize(_raw(cal), cal)["mid"]["spread"]) < 1e-6
    moved = w.normalize(_raw(cal, mid={"spread": 10.0}), cal)     # even well off the capture
    assert moved["mid"]["spread"] == 0.0
    wide = _cal(spread_span=-18.0)                                # a real span still normalizes
    splayed = w.normalize(_raw(wide, "spread"), wide)["mid"]["spread"]
    assert abs(abs(splayed) - 1.0) < 1e-6


def test_curled_finger_spread_fades_to_neutral():
    """`spread` is the in-plane angle of the proximal bone, so a CURLED finger has none left to
    measure — the ring finger read spread≈+1 curled into a ✌️ and the robot splayed it sideways.
    The gate weights the reading by cos(elevation): straight = full, curled = neutral, and a real
    splay (fingers straight) must survive it untouched."""
    cal = _cal(spread_span=-18.0)
    splayed = _raw(cal, "spread")                                  # straight + splayed -> full read
    assert abs(w.normalize(splayed, cal)["index"]["spread"]) == pytest.approx(1.0)
    for finger in ("index", "mid"):                                # same splay, but curled away
        curled = _raw(cal, "spread", **{finger: {"flex": 75.0}})
        assert w.normalize(curled, cal)[finger]["spread"] == pytest.approx(0.0)
    half = _raw(cal, "spread", index={"flex": 45.0})                # partway -> partway
    assert 0.0 < abs(w.normalize(half, cal)["index"]["spread"]) < 1.0
    assert w._spread_gate(0.0) == pytest.approx(1.0)                # straight finger: untouched


def test_widest_spread_anchor_wins():
    """🖐 leaves the middle finger sitting mid-hand (2.5° = noise) while ✌️ drives it out, so each
    finger's span comes from whichever capture moved it furthest — sign included. A finger the ✌️
    capture barely moved must keep its 🖐 span."""
    cal = _cal(spread_span=2.5, peace_span=-18.0)          # mid: dead in 🖐, alive in ✌️
    at_peace = w.normalize(_raw(cal, "peace"), cal)
    assert abs(at_peace["mid"]["spread"]) == pytest.approx(1.0)     # ✌️ defines the mid span
    assert abs(at_peace["index"]["spread"]) < 1.0                   # index keeps its wider 🖐 span
    assert abs(w.normalize(_raw(cal, "spread"), cal)["index"]["spread"]) == pytest.approx(1.0)
    assert w.normalize(_raw(cal), cal)["mid"]["spread"] == pytest.approx(0.0)      # open = neutral
    # both captures degenerate -> still neutral, never a biased default
    dead = _cal(spread_span=2.5, peace_span=1.0)
    assert w.normalize(_raw(dead, "peace"), dead)["mid"]["spread"] == pytest.approx(0.0)


def test_a_curled_capture_cannot_anchor_a_spread():
    """Measured live: ✌️ curls the ring finger and its angle drifts to −18° there, against a real
    +15° from 🖐. Being the LARGER magnitude it won the "widest anchor" rule and flipped the
    finger's direction on the robot. A capture only votes where that finger was straight."""
    cal = _cal(spread_span=2.5, peace_span=-18.0)
    assert w.curled_in(cal["peace"]) == ["ring"]           # reported at capture time, not silently
    splayed = w.normalize(_raw(cal, "spread"), cal)["ring"]["spread"]
    assert splayed == pytest.approx(-1.0)                  # 🖐 anchors it (INVERT flips the sign)
    assert w.normalize(_raw(cal, "peace"), cal)["ring"]["spread"] == pytest.approx(0.0)
    # a ring bent the OTHER way from open must not read as splayed either — direction is preserved
    other = w.normalize(_raw(cal, "open", ring={"spread": -12.0}), cal)["ring"]["spread"]
    assert other == pytest.approx(1.0)                     # opposite sign of the splayed reading


def test_thumb_opp_spans_its_anchors():
    """'opp' = the pinch aperture handlab's thumb synergy rides: 0 at the open capture, 1 when the
    tips touch, monotone between. A degenerate anchor pair falls back to the defaults."""
    cal = _cal(spread_span=-18.0, opp_open=0.94, opp_pinch=0.14)
    assert w.normalize(_raw(cal), cal)["thumb"]["opp"] == pytest.approx(0.0)
    assert w.normalize(_raw(cal, "pinch"), cal)["thumb"]["opp"] == pytest.approx(1.0)
    half = w.normalize(_raw(cal, thumb={"opp_d": 0.54}), cal)["thumb"]["opp"]
    assert 0.4 < half < 0.6
    flat = _cal(spread_span=-18.0, opp_open=0.5, opp_pinch=0.45)   # span < 0.1 -> defaults, no
    assert w.normalize(_raw(flat, thumb={"opp_d": w.THUMB_OPP_OPEN}), flat)["thumb"]["opp"] == 0.0
