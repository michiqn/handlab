#!/usr/bin/env python
"""
handlab motor characterization — find real joint min/max + measure friction/force.

Run with the hardware venv AFTER you have closed the Dynamixel Wizard AND the handlab app
(only ONE program may own the serial port):

    python tools/characterize.py range
    python tools/characterize.py force --finger mid

MODES
  range  Torque OFF, you backdrive each finger by hand, the encoder records min/max. Fails
         for curl (one-sided tendon — moving the finger does not turn the motor shaft).
  auto   MOTORIZED limit search (recommended): the motor drives each DOF with a force cap
         (compliant) into its end stop, records angle + stall current, and classifies hard stop /
         FREE-RUN (slack tendon) / soft (spring balance). Flags: --dof, --hold "flex=30", --cap.
  map    2D envelope: vary --outer, measure the --inner max at each step (--step/--outer-from).
         Shows the coupling as a curve (e.g. curl_max over flex).
  force  slow out-and-back sweep, logs current in both directions to CSV (friction vs. spring).
  diag   raw torque state + encoder ticks (troubleshooting).

COUPLING (important): the DOFs form a chain spread→flex→curl; a distal DOF measured at home=0
is the WORST case (the proximal DOF gates it). For the real max of each DOF, measure at the
proximal angle that maximizes it (auto --dof --hold / map). curl-min = 0 (one-sided).
Limit policy = max reachable + the force cap as a safety net.

Only ONE process on the port — close the Dynamixel Wizard AND the handlab app first:
    python tools/characterize.py auto --finger thumb
Convention: 0° = stretched rest pose (after zeroing). Tick 2048 = 0°, 4096/rev.
Targets the 3-finger claw3f build.
"""
from __future__ import annotations

import argparse
import glob
import math
import time

from handlab import compose, library_io
from handlab.hal.base import rad_to_ticks, ticks_to_rad
from handlab.hal.dynamixel_driver import DynamixelDriver

BAUD_DEFAULT = 1000000


# ----------------------------------------------------------------------------- setup
def find_port(explicit: str | None) -> str:
    """The U2D2 serial port: --port if given, else the first /dev/cu.usbserial* / cu.usbmodem*
    (the build YAML's port is only a placeholder)."""
    if explicit:
        return explicit
    ports = sorted(glob.glob("/dev/cu.usbserial*") + glob.glob("/dev/cu.usbmodem*"))
    if not ports:
        raise SystemExit("no U2D2 port found (looked for /dev/cu.usbserial* + cu.usbmodem*). "
                         "Plug in the U2D2, or pass --port.")
    return ports[0]


def build_driver(port: str, baud: int, profile_velocity: int, goal_current: int,
                 current_limit: int, zero: bool = True) -> tuple[DynamixelDriver, "object"]:
    """Headless (no Qt/MuJoCo): build -> ServoMap -> driver, connect, (zero)."""
    build = library_io.load_build("claw3f")
    servo_map = compose.derive_servo_map(build)
    cal = library_io.load_calibration()   # sign etc. — important, otherwise the direction is wrong
    # current_limit MUST be >= goal_current, otherwise the driver clamps goal_current down.
    # Take the ceiling from the build YAML (1500), not the old hardcoded 600 -> --cap now works.
    current_limit = max(int(current_limit), int(goal_current), int(build.hardware.current_limit))

    drv = DynamixelDriver(
        servo_map, calibration=cal,
        profile_velocity=profile_velocity,          # slow = small
        current_limit=current_limit, goal_current=goal_current,
    )
    try:
        drv.connect(port, baud)
    except Exception as e:
        raise SystemExit(
            f"\n✗ connect failed: {e}\n"
            f"  • 'No such file' → U2D2/USB missing: check/replug the USB cable (ls /dev/cu.*).\n"
            f"  • 'no status packet' / busy → handlab or the Wizard still open (⌘Q!), OR the servos\n"
            f"    browned out (too much current): power-cycle the hub. Only ONE program on the port.\n"
        )

    if not drv._online:                       # port open but no servo pings back -> reconnect
        for attempt in range(1, 4):
            print(f"⚠ no servos found — pinging again (attempt {attempt}/3) …")
            time.sleep(1.0)
            try:
                drv.disconnect()
                time.sleep(0.5)
                drv.connect(port, baud)
            except Exception:
                pass
            if drv._online:
                break
    print(f"✓ connected on {port} @ {baud}   online: {sorted(drv._online)}")
    if not drv._online:
        raise SystemExit(
            "\n✗ No servo answers (online: []). This is HARDWARE/bus, not the script:\n"
            "  • Power:  power hub / 5 V supply on? Hub LED lit?\n"
            "  • Cables: all daisy-chain plugs seated? U2D2 ↔ hub ↔ first servo?\n"
            "  • Loose:  did a plug come loose while bending a finger? (very common!)\n"
            "  → power-cycle the hub, reseat the plugs, then start again.\n"
        )

    if zero:
        input("\n→ Let the hand hang loosely in the stretched rest pose, then [Enter] to zero … ")
        drv.capture_zero()   # sets 0° = current rest pose (torque is off)
        print("✓ zeroed (0° = rest pose)\n")
    return drv, servo_map


# ----------------------------------------------------------------------------- DIAG
def run_diag(drv: DynamixelDriver, servo_map, hz: float = 7.0) -> None:
    """Raw low-level diagnosis: torque state from the wire, force torque OFF, confirm, then show
    RAW encoder ticks (reg 132, unconverted) live. If the user bends a finger and the raw number
    MOVES -> encoder/backdrive OK (the problem is in the logic layer). If it does NOT move ->
    the motor shaft isn't turning (torque on OR tendon slip)."""
    ids = sorted(drv._online)
    print("=== DIAGNOSIS — torque state + raw encoder ===")
    for sid in ids:
        try:
            t0 = drv._read(sid, 64, 1)            # torque-enable register BEFORE switching off
            drv._write1(sid, 64, 0)               # force torque OFF
            t1 = drv._read(sid, 64, 1)            # ... and read back
            pos = drv._read(sid, 132, 4, signed=True)   # raw encoder ticks
            print(f"  #{sid:>2}  torque {t0} -> {t1}   raw_ticks={pos}")
        except Exception as e:
            print(f"  #{sid:>2}  READ ERROR: {e}")
    print("\nTorque is now OFF (see above, should be 0 everywhere).")
    print("→ NOW move ONE finger back and forth by hand. The raw number of its servo")
    print("  MUST change. If nothing happens -> the motor shaft is not turning with it.")
    print("  [Ctrl-C] to stop.\n")
    try:
        while True:
            cells = []
            for sid in ids:
                try:
                    pos = drv._read(sid, 132, 4, signed=True)
                except Exception:
                    pos = -1
                cells.append(f"{sid}:{pos}")
            print("\r raw  " + "  ".join(cells) + "   ", end="", flush=True)
            time.sleep(1.0 / hz)
    except KeyboardInterrupt:
        print("\n")


# ----------------------------------------------------------------------------- RANGE
def run_range(drv: DynamixelDriver, servo_map, hz: float = 14.0) -> None:
    """Torque OFF. Record min/max per joint while the user moves it by hand.
    Uses SINGLE position reads (read_position) — the same one zeroing uses, which is reliable
    (the bulk read/GroupSyncRead can misbehave on macOS -> returned 0)."""
    ids = sorted(drv._online)
    for sid in ids:                        # GUARANTEED free to move (torque off)
        try:
            drv.set_torque(sid, False)
        except Exception:
            pass
    lo = {sid: 0.0 for sid in ids}         # 0° = rest pose is always inside the range
    hi = {sid: 0.0 for sid in ids}

    print("=== RANGE CAPTURE ===   torque is OFF → the MOTORS do NOT move.")
    print("YOU move each finger BY HAND through its full travel (spread · flex · curl),")
    print("up to the soft end stop. The live° numbers below MUST follow.")
    print("[Ctrl-C] when you're through all of them.\n")
    try:
        while True:
            live = []
            for sid in ids:
                try:
                    raw = drv.read_position(sid)
                except Exception:
                    raw = 0
                if raw == 0:                       # read error/dropped this tick
                    live.append(f"{sid}: --")
                    continue
                deg = math.degrees(ticks_to_rad(raw))
                lo[sid] = min(lo[sid], deg)
                hi[sid] = max(hi[sid], deg)
                live.append(f"{sid}:{deg:+4.0f}")
            print("\r live°  " + "  ".join(live) + "  ", end="", flush=True)
            time.sleep(1.0 / hz)
    except KeyboardInterrupt:
        pass

    print("\n\n=== RESULT — real mechanical range per joint ===")
    print(f"{'id':>3}  {'finger':7} {'dof':7} {'measured min/max':>16}   suggested limit")
    for sid in ids:
        sv = servo_map.by_id(sid)
        mn, mx = round(lo[sid]), round(hi[sid])
        smn = mn + 2 if mn < -1 else mn        # operating limit a few degrees inside
        smx = mx - 2 if mx > 1 else mx
        fname = sv.finger if sv else "?"
        dname = sv.dof if sv else "?"
        print(f"{sid:>3}  {fname:7} {dname:7} {mn:+6.0f} / {mx:+4.0f}°     [{smn:+4d}, {smx:+4d}]")
    moved = any(abs(lo[sid]) > 2 or abs(hi[sid]) > 2 for sid in ids)
    if not moved:
        print("\n⚠ NO movement detected (everything ~0°). Two possibilities:")
        print("  • fingers not moved by hand yet → again, grab the fingers & bend them through, or")
        print("  • the hand is stiff (could it be moved by hand at all?).")
    else:
        print("\n0° = stretched rest pose. Set these values as the per-DOF limits (tap the min/max")
        print("  label on the slider) → slider, twin joint and servo travel read the same range.")


# ----------------------------------------------------------------------------- FORCE
def run_force(drv: DynamixelDriver, servo_map, finger: str, amp_deg: float,
              cap_ma: int, hz: float = 12.0, dwell: float = 2.5) -> None:
    """Motorized, gentle: out-and-back per DOF, logging current in both directions."""
    ids = [s.id for s in servo_map.servos if s.finger == finger and s.id in drv._online]
    if not ids:
        raise SystemExit(f"No online servo for finger '{finger}'. Online: {sorted(drv._online)}")

    # set the force cap for the measurement (RAM), then torque on (re-asserts the cap):
    drv._goal_current = int(cap_ma)
    for sid in ids:
        drv.set_torque(sid, True)   # goal := present (no jerk), torque ON
    print(f"=== FORCE SWEEP  finger={finger}  ±{amp_deg:.0f}°  cap={cap_ma} mA ===")
    print("Keep the hand FREE (fingers will move). [Ctrl-C] = go limp immediately.\n")

    stamp = time.strftime("%Y%m%d_%H%M%S")
    csv_path = f"characterize_{finger}_{stamp}.csv"
    rows = []
    try:
        for sid in ids:
            sv = servo_map.by_id(sid)
            dof = sv.dof if sv else "?"
            # 0 -> +amp -> 0 -> -amp -> 0  (out/in in both directions)
            for tgt in (amp_deg, 0.0, -amp_deg, 0.0):
                drv.set_goal_position(sid, rad_to_ticks(math.radians(tgt)))
                t0 = time.monotonic()
                while time.monotonic() - t0 < dwell:
                    try:                                            # single reads (robust)
                        raw = drv.read_position(sid)
                        cur = drv._read(sid, 126, 2, signed=True)   # Present Current mA
                        pwm = drv._read(sid, 124, 2, signed=True)   # Present PWM
                        temp = drv._read(sid, 146, 1)               # Present Temperature °C
                    except Exception:
                        continue
                    if raw == 0:                                    # dropped read -> skip (no -180°)
                        continue
                    pos = math.degrees(ticks_to_rad(raw))
                    rows.append((f"{time.monotonic():.3f}", sid, dof, f"{tgt:.1f}",
                                 f"{pos:.1f}", cur, pwm, temp))
                    print(f"\r  #{sid} {dof:6} target={tgt:+5.0f}° actual={pos:+6.1f}° "
                          f"current={cur:+5d}mA pwm={pwm:+5d} T={temp}°C  ", end="", flush=True)
                    if temp >= 55:
                        print("\n⚠ 55 °C reached — aborting this DOF.")
                        break
                    time.sleep(1.0 / hz)
            print()
    except KeyboardInterrupt:
        print("\n[Ctrl-C] — shutting down.")
    finally:
        for sid in ids:
            try:
                drv.set_torque(sid, False)
            except Exception:
                pass
        with open(csv_path, "w") as f:
            f.write("t,id,dof,target_deg,pos_deg,current_ma,pwm,temp_c\n")
            for r in rows:
                f.write(",".join(str(x) for x in r) + "\n")
        print(f"\n✓ {len(rows)} samples → {csv_path}")
        print("  friction ≈ (current_out − current_in)/2 at the same position; "
              "spring ≈ mean of both directions.")


# ----------------------------------------------------------------------------- AUTO
def run_auto(drv: DynamixelDriver, servo_map, finger: str, cap_ma: int,
             dof_filter: str | None = None, hold: dict | None = None,
             far_deg: float = 200.0, settle_deg: float = 1.0, settle_time: float = 0.8,
             timeout: float = 11.0, hz: float = 10.0) -> None:
    """Motorized limit search: the MOTOR drives each DOF gently (force cap) into its end stop;
    where the position stops = the limit. For joints you cannot backdrive by hand (e.g. curl =
    one-sided tendon — only turnable at the servo horn).
    dof_filter: measure only this DOF. hold: {dof: degrees} position + hold other DOFs first
    (e.g. thumb-curl needs flex ~30° away from home, otherwise it binds)."""
    hold = hold or {}
    finger_servos = [s for s in servo_map.servos if s.finger == finger and s.id in drv._online]
    if not finger_servos:
        raise SystemExit(f"No online servos for finger '{finger}'. Online: {sorted(drv._online)}")
    dof2sid = {s.dof: s.id for s in finger_servos}
    if dof_filter:
        sweep_ids = [dof2sid[dof_filter]] if dof_filter in dof2sid else []
    else:
        sweep_ids = [s.id for s in finger_servos if s.dof not in hold]
    drv._goal_current = int(cap_ma)

    def go(sid: int, dof: str, target_deg: float, label: str,
           quiet: bool = False) -> tuple[float, int, bool]:
        """Thin wrapper around the shared _drive() helper (same stall/settle logic as map);
        passes run_auto's loop parameters through and keeps the closing print."""
        end, cur, moved = _drive(drv, sid, dof, target_deg, label, quiet,
                                 settle_deg=settle_deg, settle_time=settle_time,
                                 timeout=timeout, hz=hz)
        if not quiet:
            print(f"   → {end:+.0f}°{'' if moved else '   ⚠ NO movement'}")
        return end, cur, moved

    print(f"=== AUTO LIMITS  finger={finger}  cap={cap_ma} mA"
          f"{'  hold ' + str(hold) if hold else ''} ===")
    print("The MOTOR drives each DOF gently into its end stop (force cap = compliant).")
    print("Leave the finger free/watch it. [Ctrl-C] = go limp immediately.\n")
    held_ids: list[int] = []
    results: dict[int, tuple] = {}
    try:
        for dofname, deg in hold.items():            # position + hold other DOFs first
            hsid = dof2sid.get(dofname)
            if hsid is None:
                continue
            drv.set_torque(hsid, True)
            print(f"holding #{hsid} {dofname} @ {float(deg):+.0f}° …")
            go(hsid, dofname, float(deg), "hold", quiet=True)
            held_ids.append(hsid)
        for sid in sweep_ids:
            sv = servo_map.by_id(sid)
            dof = sv.dof if sv else "?"
            drv.set_torque(sid, True)                # this servo (the held ones stay on)
            print(f"#{sid} {dof}:")
            hi, hi_c, hi_m = go(sid, dof, +far_deg, "+ ")
            go(sid, dof, 0.0, "→0", quiet=True)      # back to center
            lo, lo_c, lo_m = go(sid, dof, -far_deg, "− ")
            go(sid, dof, 0.0, "→0", quiet=True)
            drv.set_torque(sid, False)               # limp again
            results[sid] = (lo, hi, lo_c, hi_c, lo_m, hi_m)
    except KeyboardInterrupt:
        print("\n[Ctrl-C] — shutting down.")
    finally:
        for sid in set(sweep_ids) | set(held_ids):
            try:
                drv.set_torque(sid, False)
            except Exception:
                pass

    def classify(end: float, cur: int) -> str:
        if abs(cur) >= cap_ma * 0.6:
            return "hard stop"
        if abs(abs(end) - far_deg) < 10:
            return "FREE-RUN (slack tendon, NO stop)"
        return "soft hold (rubber balance)"

    def op_bound(end: float, cur: int, is_max: bool) -> int:
        if abs(cur) >= cap_ma * 0.6:                 # real stop -> 3° inside
            return (round(end) - 3) if is_max else (round(end) + 3)
        return 0                                      # free-run/slack -> rest pose 0°

    print("\n=== RESULT — motorized limits ===")
    for sid in sweep_ids:
        if sid not in results:
            continue
        lo, hi, lc, hc, lm, hm = results[sid]
        sv = servo_map.by_id(sid)
        print(f"\n  #{sid} {sv.finger} {sv.dof}:")
        print(f"     min {lo:+6.1f}° @ {lc:+4d} mA   → {classify(lo, lc)}")
        print(f"     max {hi:+6.1f}° @ {hc:+4d} mA   → {classify(hi, hc)}")
        print(f"     ⇒ suggested operating range: [{op_bound(lo, lc, False)}, {op_bound(hi, hc, True)}]")
    print("\nhard stop = real mechanical limit (current at the cap).")
    print("FREE-RUN = the motor spun freely (slack tendon) → no real limit, ⇒ 0°.")
    print(f"soft hold = force balance against rubber; a higher --cap (now {cap_ma}) would go further.")


# ----------------------------------------------------------------------------- shared drive helpers
def _read_deg(drv, sid: int) -> float | None:
    """Present position in degrees, or None on a DROPPED/failed read — never a bogus number.
    read_position() returns literal 0 only for an offline/dropped servo (a servo at home rest
    reads ~2048); ticks_to_rad(0) would be -180°, which corrupts the stall/settle logic mid-run.
    Callers must skip a None tick (run_range guards the same `raw == 0` way)."""
    try:
        raw = drv.read_position(sid)
    except Exception:
        return None                 # comm error -> no reading (don't fake 0.0°)
    if raw == 0:                    # offline/dropped sentinel
        return None
    return math.degrees(ticks_to_rad(raw))


def _cur_ma(drv, sid: int) -> int:
    try:
        return drv._read(sid, 126, 2, signed=True)      # Present Current (reg 126) mA
    except Exception:
        return 0


def _drive(drv, sid: int, dof: str, target_deg: float, label: str = "", quiet: bool = False,
           settle_deg: float = 1.0, settle_time: float = 0.8, timeout: float = 11.0,
           hz: float = 10.0) -> tuple[float, int, bool]:
    """Drive sid toward target; stop at an end stop (only after the first movement) or at the
    target. Returns (rest position°, current mA, has_moved)."""
    drv.set_goal_position(sid, rad_to_ticks(math.radians(target_deg)))
    t0 = time.monotonic()
    last = _read_deg(drv, sid)                    # may be None on a dropped first read
    stable_since = t0
    moved = False
    while time.monotonic() - t0 < timeout:
        time.sleep(1.0 / hz)
        now = _read_deg(drv, sid)
        if now is None:                           # dropped tick -> skip, don't corrupt stall logic
            continue
        c = _cur_ma(drv, sid)
        if not quiet:
            print(f"\r  #{sid} {dof:6} {label} actual={now:+6.1f}°  current={c:+5d}mA     ",
                  end="", flush=True)
        if last is not None and abs(now - last) > settle_deg:
            stable_since = time.monotonic()
            moved = True
        last = now
        if abs(now - target_deg) < 2.0:
            break
        if moved and (time.monotonic() - stable_since > settle_time):
            break
    end = _read_deg(drv, sid)
    if end is None:                               # final read dropped -> last good (or home)
        end = last if last is not None else 0.0
    return end, _cur_ma(drv, sid), moved


# ----------------------------------------------------------------------------- MAP (envelope)
def run_map(drv: DynamixelDriver, servo_map, finger: str, cap_ma: int,
            outer: str = "flex", inner: str = "curl", step: float = 20.0,
            outer_from: float = 0.0, outer_max: float = 200.0) -> None:
    """Envelope map: vary the OUTER DOF (e.g. flex/spread) from outer_from in steps and measure
    the INNER DOF's max at each. Shows the coupled picture — how far the inner DOF gets depending
    on where the outer one is. For spread, set outer_from negative (bidirectional)."""
    fs = [s for s in servo_map.servos if s.finger == finger and s.id in drv._online]
    dof2sid = {s.dof: s.id for s in fs}
    o_sid, i_sid = dof2sid.get(outer), dof2sid.get(inner)
    if o_sid is None or i_sid is None:
        raise SystemExit(f"Finger '{finger}' has '{outer}'/'{inner}' not online: {sorted(drv._online)}")
    drv._goal_current = int(cap_ma)

    print(f"=== ENVELOPE MAP  {finger}: {outer} × {inner}  cap={cap_ma} mA ===")
    print(f"varying {outer} in {step:.0f}° steps, measuring {inner} max at each.")
    print("Hand free/watch it. [Ctrl-C] = limp.\n")
    rows = []
    try:
        drv.set_torque(o_sid, True)
        drv.set_torque(i_sid, True)
        v = outer_from
        while v <= outer_max + 0.1:
            oend, oc, om = _drive(drv, o_sid, outer, v, quiet=True)         # hold outer @ v
            iend, ic, im = _drive(drv, i_sid, inner, +200.0, quiet=True)    # inner fully closed
            art = ("stop" if abs(ic) >= cap_ma * 0.6
                   else "FREE-RUN" if abs(iend) > 190 else "soft")
            rows.append((round(oend), round(iend), ic, art))
            print(f"  {outer}={oend:+5.0f}°   {inner}_max={iend:+6.0f}° @ {ic:+4d}mA   ({art})")
            _drive(drv, i_sid, inner, 0.0, quiet=True)                      # inner back
            if abs(oend - v) > 10:                                          # outer gets no further
                print(f"  → {outer} stops at ~{oend:.0f}° — end of the map.")
                break
            v += step
    except KeyboardInterrupt:
        print("\n[Ctrl-C] — shutting down.")
    finally:
        for sid in (o_sid, i_sid):
            try:
                drv.set_torque(sid, False)
            except Exception:
                pass

    print(f"\n=== ENVELOPE  {finger}  {outer} → {inner}_max ===")
    best = max(rows, key=lambda r: r[1]) if rows else None
    for orow, irow, ic, art in rows:
        bar = "█" * max(0, round(irow / 6))
        print(f"  {outer:5} {orow:+5d}°  →  {inner} {irow:+5d}° {bar}  ({art})")
    if best:
        print(f"\n{inner} closes furthest ({best[1]:+d}°) at {outer} = {best[0]:+d}°.")
    print("→ take the furthest as curl-max, and note at which flex position curl grips best.")


# ----------------------------------------------------------------------------- main
def main() -> None:
    ap = argparse.ArgumentParser(description="handlab motor characterization")
    ap.add_argument("mode", choices=["range", "force", "diag", "auto", "map"], nargs="?", default="range")
    ap.add_argument("--finger", choices=["index", "thumb", "mid"], default="mid",
                    help="for force/auto/map")
    ap.add_argument("--amp", type=float, default=15.0, help="force: amplitude ±° (start small!)")
    ap.add_argument("--cap", type=int, default=350, help="force/auto/map: force cap goal_current mA")
    ap.add_argument("--dof", choices=["spread", "flex", "curl"], default=None,
                    help="auto: measure only this DOF (instead of all three)")
    ap.add_argument("--hold", default=None,
                    help="auto: hold other DOFs first, e.g. 'flex=30' (comma for several)")
    ap.add_argument("--outer", choices=["spread", "flex", "curl"], default="flex",
                    help="map: varied DOF (default flex)")
    ap.add_argument("--inner", choices=["spread", "flex", "curl"], default="curl",
                    help="map: measured DOF (default curl)")
    ap.add_argument("--step", type=float, default=20.0, help="map: step size ° of the outer DOF")
    ap.add_argument("--outer-from", type=float, default=0.0, dest="outer_from",
                    help="map: start value ° of the outer DOF (negative for spread, e.g. -70)")
    ap.add_argument("--port", default=None, help="U2D2 port (default: auto-detect)")
    ap.add_argument("--baud", type=int, default=BAUD_DEFAULT)
    args = ap.parse_args()

    # range: very slow & gentle; force: slow, cap as chosen
    drv, servo_map = build_driver(
        find_port(args.port), args.baud,
        profile_velocity=15,
        goal_current=(args.cap if args.mode in ("force", "auto", "map") else 350),
        current_limit=600,
        zero=(args.mode != "diag"),          # diag needs no zeroing
    )
    try:
        if args.mode == "diag":
            run_diag(drv, servo_map)
        elif args.mode == "range":
            run_range(drv, servo_map)
        elif args.mode == "auto":
            hold = {}
            if args.hold:
                for part in args.hold.split(","):
                    k, _, v = part.partition("=")
                    if v:
                        hold[k.strip()] = float(v)
            run_auto(drv, servo_map, args.finger, args.cap, dof_filter=args.dof, hold=hold)
        elif args.mode == "map":
            run_map(drv, servo_map, args.finger, args.cap,
                    outer=args.outer, inner=args.inner, step=args.step, outer_from=args.outer_from)
        else:
            run_force(drv, servo_map, args.finger, args.amp, args.cap)
    finally:
        drv.disconnect()   # torque off + port closed
        print("✓ disconnected (hand limp).")


if __name__ == "__main__":
    main()
