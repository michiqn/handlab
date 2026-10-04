#!/usr/bin/env python
"""Measure real joint limits by backdriving — torque OFF, you move the fingers by hand.

Connects the Dynamixel bus torque-off, recovers the app's calibrated home frame, then tracks the
LIVE min/max of every servo (in the app's joint degrees, 0° = home) while you sweep each finger
through its full range. Ctrl-C prints the measured min/max + an app-ready range per joint. It writes
NOTHING — type the ones you want into the app's Control tab (tap a joint's min/max label).

Run it from the hardware venv, with the handlab app + Dynamixel Wizard CLOSED (only ONE process may
own the U2D2 serial port):

    ~/.venvs/handlab/bin/python tools/measure_limits.py
    ~/.venvs/handlab/bin/python tools/measure_limits.py --margin 2 --build claw4f

Prereqs: set the hand's HOME and DIRECTION in the app first (this reads that frame). 0° = stretched
rest. Torque stays OFF the whole time — the motors never move; you do.
"""
from __future__ import annotations

import argparse
import glob
import sys
import time

from handlab import compose, library_io
from handlab.hal.base import ticks_to_deg
from handlab.hal.dynamixel_driver import DynamixelDriver


def suggest_range(lo_deg: float, hi_deg: float, margin: int = 0) -> tuple[int, int] | None:
    """Measured (min,max)° → an app-ready integer [lo,hi] that satisfies the app's rules: it always
    includes 0° (home, so `lo<=0<=hi`) and `margin` pulls each bound inward (tendon safety). Returns
    None when the joint effectively didn't move (`lo>=hi`), so the caller can flag "re-sweep"."""
    lo = min(0, round(lo_deg) + margin)          # negative → +margin moves it inward (toward 0)
    hi = max(0, round(hi_deg) - margin)          # positive → -margin moves it inward
    return (lo, hi) if lo < hi else None


def _find_port(explicit: str | None) -> str:
    if explicit:
        return explicit
    ports = sorted(glob.glob("/dev/cu.usbserial*") + glob.glob("/dev/cu.usbmodem*"))
    if not ports:
        raise SystemExit("✗ no U2D2 port found (looked for /dev/cu.usbserial* + cu.usbmodem*). "
                         "Plug in the U2D2, or pass --port.")
    return ports[0]


def connect_driver(port: str, baud: int | None, build_name: str):
    """Headless (no Qt/MuJoCo): build → ServoMap → driver, connect torque-off. Mirrors the app's
    own construction (bridge_hardware._new_dynamixel) so sign/caps match exactly."""
    build = library_io.load_build(build_name)
    sm = compose.derive_servo_map(build)
    hw = build.hardware
    baud = baud or hw.baud
    drv = DynamixelDriver(sm, calibration=library_io.load_calibration(),
                          profile_velocity=hw.profile_velocity, profile_acceleration=hw.profile_acceleration,
                          current_limit=hw.current_limit, goal_current=hw.goal_current,
                          hold_current=hw.hold_current, pos_p=hw.pos_p, pos_i=hw.pos_i, pos_d=hw.pos_d)
    try:
        drv.connect(port, baud)                  # _configure writes TORQUE_ENABLE=0 → stays limp
    except Exception as e:
        raise SystemExit(
            f"\n✗ connect failed: {e}\n"
            "  • 'No such file' → U2D2/USB not present: check the cable (ls /dev/cu.*).\n"
            "  • 'busy' / 'no status packet' → the handlab app or Dynamixel Wizard still owns the\n"
            "    port. Quit them (only ONE process may hold the bus), then retry.\n")
    if not drv._online:                          # port open but nothing pinged → retry
        for attempt in range(1, 4):
            print(f"⚠ no servos answered — re-pinging ({attempt}/3) …")
            time.sleep(1.0)
            try:
                drv.disconnect(); time.sleep(0.5); drv.connect(port, baud)
            except Exception:
                pass
            if drv._online:
                break
    if not drv._online:
        drv.disconnect()
        raise SystemExit(
            "\n✗ no servo answered (online: []). That's power/cable, not the script:\n"
            "  • 5 V power hub on? all daisy-chain connectors seated?\n"
            "  → power-cycle the hub, reseat the connectors, then retry.\n")
    print(f"✓ connected on {port} @ {baud}   online: {sorted(drv._online)}")
    return drv, sm


def sweep(drv, sm, hz: float, margin: int) -> None:
    ids = sorted(drv._online)
    for sid in ids:                              # guarantee free movement (torque already off)
        try:
            drv.set_torque(sid, False)
        except Exception:
            pass
    lo = {sid: 0.0 for sid in ids}               # 0° = home is always inside the range
    hi = {sid: 0.0 for sid in ids}

    def label(sid):
        sv = sm.by_id(sid)
        return (sv.finger, sv.dof) if sv else ("?", "?")

    print("=== RANGE CAPTURE ===  Torque is OFF — the motors will NOT move; YOU move the fingers.")
    print("Sweep each finger through its FULL range (spread · flex · curl), gently to each stop.")
    print("The live min/max below must move with your hand.  Press Ctrl-C when you've done them all.\n")

    hdr = f"{'id':>3}  {'finger':7} {'dof':6} {'cur':>6} {'min':>6} {'max':>6}"
    nlines = 0
    try:
        while True:
            rows = [hdr]
            for sid in ids:
                try:
                    raw = drv.read_position(sid)
                except Exception:
                    raw = 0
                f, d = label(sid)
                if raw == 0:                     # dropped/offline this tick → hold, don't corrupt min/max
                    rows.append(f"{sid:>3}  {f:7} {d:6} {'--':>6} {round(lo[sid]):>+6} {round(hi[sid]):>+6}")
                    continue
                deg = ticks_to_deg(raw)          # app joint degrees (home = 0, sign applied)
                lo[sid] = min(lo[sid], deg)
                hi[sid] = max(hi[sid], deg)
                rows.append(f"{sid:>3}  {f:7} {d:6} {deg:>+6.0f} {round(lo[sid]):>+6} {round(hi[sid]):>+6}")
            out = (f"\x1b[{nlines}A" if nlines else "") + "".join(f"\x1b[2K{r}\n" for r in rows)
            sys.stdout.write(out)
            sys.stdout.flush()
            nlines = len(rows)
            time.sleep(1.0 / max(hz, 1.0))
    except KeyboardInterrupt:
        pass

    print("\n=== RESULT — measured mechanical range per joint (app frame, 0° = home) ===")
    print(f"{'finger':7} {'dof':6} {'#id':>4}   {'measured':>13}    app range_deg")
    for sid in ids:
        f, d = label(sid)
        rng = suggest_range(lo[sid], hi[sid], margin)
        meas = f"{round(lo[sid]):+d} / {round(hi[sid]):+d}°"
        tail = "(did not move — re-sweep)" if rng is None else f"min {rng[0]:+d}   max {rng[1]:+d}"
        print(f"{f:7} {d:6} {sid:>4}   {meas:>13}    {tail}")
    print("\nType the ranges you want into the app's Control tab (tap a joint's min/max label).")
    print("Tip: leave a few ° inside the mechanical max to spare the tendons (or re-run with --margin).")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="Backdrive each joint to its limits and read them (torque off, print only).")
    ap.add_argument("--port", default=None, help="U2D2 port (default: auto-detect cu.usbserial*/cu.usbmodem*)")
    ap.add_argument("--baud", type=int, default=None, help="baud (default: the build's)")
    ap.add_argument("--build", default="claw4f", help="build name (default: claw4f)")
    ap.add_argument("--hz", type=float, default=14.0, help="live readout rate (default: 14)")
    ap.add_argument("--margin", type=int, default=0, help="degrees to pull each limit inward (tendon safety)")
    args = ap.parse_args(argv)

    port = _find_port(args.port)
    drv, sm = connect_driver(port, args.baud, args.build)
    try:
        input("\n→ Hold the hand at the stretched REST pose (fingers straight), then press Enter to home … ")
        drv.home_at_connect()                    # recover stored absolute home; delta-home the rest. Reads only.
        print("✓ homed (0° = rest).\n")
        sweep(drv, sm, args.hz, args.margin)
    finally:
        try:
            drv.disconnect()                     # torque off + release the port
        except Exception:
            pass
        print("\n✓ disconnected (hand limp, port released).")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
