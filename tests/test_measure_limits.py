"""Unit coverage for tools/measure_limits.py — the pure suggest_range() helper.

tools/ is not a package, so load the module by path (it has no import-time side effects — it only
connects inside main()). suggest_range turns a measured (min,max)° into an app-ready integer range
that must obey the app's setDofRange rules: lo < hi AND lo <= 0 <= hi (0° = home reachable)."""
import importlib.util
from pathlib import Path

_PATH = Path(__file__).resolve().parent.parent / "tools" / "measure_limits.py"
_spec = importlib.util.spec_from_file_location("measure_limits", _PATH)
measure_limits = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(measure_limits)          # import-safe: no hardware touched on import

sr = measure_limits.suggest_range


def test_two_sided_joint_rounds_and_keeps_bounds():
    r = sr(-70.4, 118.6)
    assert r == (-70, 119)
    assert r[0] <= 0 <= r[1] and r[0] < r[1]      # the app's setDofRange invariant


def test_one_sided_joint_clamps_lo_to_zero():
    # a curl-like joint that only flexes positive from home → lo must snap to 0 (home stays reachable)
    assert sr(1.8, 175.0) == (0, 175)


def test_no_move_returns_none():
    assert sr(0.2, 0.3) is None                   # joint didn't move → flag, don't emit a bogus range


def test_margin_pulls_inward_both_sides():
    assert sr(-70.0, 120.0, margin=3) == (-67, 117)


def test_margin_never_crosses_zero():
    # a small one-sided range with a big margin must not flip past 0 (stays lo<=0<=hi or None)
    r = sr(0.0, 4.0, margin=3)
    assert r is None or (r[0] <= 0 <= r[1] and r[0] < r[1])
