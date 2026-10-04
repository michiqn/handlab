"""Guard for the P0 dropped-read fix in tools/characterize.py.

A dropped/offline servo makes read_position() return literal 0; ticks_to_rad(0) is -180°, which
used to poison the stall/settle logic mid-run. _read_deg must return None (skip the tick) on a
dropped-0 or a comm exception, and a real degree otherwise. tools/ is not a package, so we load the
module by path (it has no import-time side effects — it only connects inside main())."""

import importlib.util
import math
from pathlib import Path

import pytest

_PATH = Path(__file__).resolve().parent.parent / "tools" / "characterize.py"
_spec = importlib.util.spec_from_file_location("characterize", _PATH)
characterize = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(characterize)


class _FakeDrv:
    """Minimal stand-in: _read_deg only calls read_position(sid)."""
    def __init__(self, value):
        self._value = value  # an int to return, or an Exception instance to raise

    def read_position(self, sid):
        if isinstance(self._value, Exception):
            raise self._value
        return self._value


def test_read_deg_none_on_dropped_zero():
    # raw 0 = offline/dropped sentinel -> None, NOT -180°.
    assert characterize._read_deg(_FakeDrv(0), 2) is None


def test_read_deg_none_on_read_exception():
    assert characterize._read_deg(_FakeDrv(RuntimeError("bus")), 2) is None


def test_read_deg_home_is_zero_degrees():
    # a servo at home rest reads logical CENTER (2048) = 0°, and must NOT be mistaken for dropped.
    assert characterize._read_deg(_FakeDrv(2048), 2) == pytest.approx(0.0, abs=1e-6)


@pytest.mark.parametrize("ticks,deg", [(2560, 45.0), (1536, -45.0), (3072, 90.0)])
def test_read_deg_converts_real_positions(ticks, deg):
    assert characterize._read_deg(_FakeDrv(ticks), 2) == pytest.approx(deg, abs=0.1)
