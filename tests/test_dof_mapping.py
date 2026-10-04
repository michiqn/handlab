"""Regression guard for the deg<->tick pipeline (found during the real-hand bring-up).

On the real hand, a commanded 30° flex read back as ~24° in get_state. We proved this is PHYSICAL
(servo stalls short), NOT a software units/scale bug: the command path (rad_to_ticks -> _to_wire) and
the readback path (_to_logical -> ticks_to_rad) are exact inverses with identical constants, so if the
servo actually reaches its goal, get_state returns the commanded angle exactly. These tests nail that
symmetry down so a future change can't silently reintroduce a scale factor and get misread as
'compliance'."""

import math

import pytest

from handlab.hal.base import CENTER, rad_to_ticks, ticks_to_rad
from handlab.hal.dynamixel_driver import DynamixelDriver
from handlab.models import Servo, ServoMap


def _driver(signs=None, offsets=None):
    """A DynamixelDriver with a tiny fixed servo map, never connected — we only exercise the
    pure tick math (_to_wire/_to_logical), which needs no serial port."""
    sm = ServoMap(servos=[Servo(id=i, finger="f", dof="d", actuator=f"a{i}") for i in (1, 2, 3)])
    d = DynamixelDriver(sm)
    for sid, s in (signs or {}).items():
        d.set_sign(sid, s)
    if offsets:
        d.set_calibration({sid: {"offset": o} for sid, o in offsets.items()})
    return d


def test_ticks_roundtrip_is_exact_for_integer_ticks():
    # ticks_to_rad is exact; rad_to_ticks rounds back -> integer ticks survive a full round-trip.
    for t in range(0, 4096, 7):
        assert rad_to_ticks(ticks_to_rad(t)) == t


@pytest.mark.parametrize("deg", [-90, -45, -7.0, 0, 7.0, 15, 30, 70, 110, 180])
def test_degree_roundtrip_within_one_tick(deg):
    back = math.degrees(ticks_to_rad(rad_to_ticks(math.radians(deg))))
    # one tick is 360/4096 ≈ 0.088° — rounding is the only loss, no scale factor.
    assert abs(back - deg) < 360 / 4096


@pytest.mark.parametrize("sign", [1, -1])
@pytest.mark.parametrize("offset", [0, -50, 137])
def test_wire_logical_are_exact_inverses(sign, offset):
    d = _driver(signs={2: sign}, offsets={2: offset})
    for logical in range(0, 4096, 11):
        assert d._to_logical(2, d._to_wire(2, logical)) == logical


@pytest.mark.parametrize("sign", [1, -1])
@pytest.mark.parametrize("offset", [0, -80, 200])
@pytest.mark.parametrize("cmd_deg", [15, 30, 70, 110])
def test_full_command_readback_has_no_scale_factor(sign, offset, cmd_deg):
    """The exact path get_state exercises: commanded degrees -> goal wire ticks, and IF the servo
    reaches that goal (present_wire == goal_wire), the reported degree equals the command. Any 65%-
    style shortfall must therefore be physical (present_wire != goal_wire), never this math."""
    d = _driver(signs={2: sign}, offsets={2: offset})
    goal_wire = d._to_wire(2, rad_to_ticks(math.radians(cmd_deg)))       # command path
    present_logical = d._to_logical(2, goal_wire)                        # read_states() does this
    reported = round(math.degrees(ticks_to_rad(present_logical)), 1)     # agentState does this
    assert abs(reported - cmd_deg) < 360 / 4096
