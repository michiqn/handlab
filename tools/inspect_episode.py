#!/usr/bin/env python3
"""Check recorded demo episodes BEFORE you spend an afternoon on 50 of them.

An imitation-learning dataset fails quietly: the observation lags the action, a servo sat offline
for half the run, the camera held the same frame for 30 steps, the object was never in view. None of
that is visible while recording — it shows up as a policy that "almost works" and costs you the
whole batch. So: record ONE episode, run this, fix the setup, then record the rest.

    tools/inspect_episode.py demos/claw4f/grasp/episode_000        # one episode, full report
    tools/inspect_episode.py demos/claw4f/grasp                    # every episode, one line each
    tools/inspect_episode.py demos/claw4f/grasp/episode_000 --sheet   # + a contact sheet PNG

Reads only (numpy + cv2, no torch, no handlab import). See handlab/recorder.py for the schema.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

# thresholds that separate "usable demo" from "quietly poisoned dataset"
STALE_WARN = 0.25          # >25% held camera frames -> the observation lags the action
OFFLINE_WARN = 0.02        # >2% of servo-steps dropped off the bus
SHORT_WARN_S = 3.0         # shorter than this is usually a mis-click, not a demo
OBJECT_WARN = 0.5          # object marker visible in <50% of steps
STUCK_TICKS = 20           # a servo whose goal never moves further than this did nothing


def load(ep: Path) -> dict:
    d = dict(np.load(ep / "episode.npz"))
    meta_path = ep / "meta.yaml"
    meta = {}
    if meta_path.exists():                     # tiny hand-rolled reader — no yaml dependency here
        for line in meta_path.read_text().splitlines():
            if ":" in line and not line.strip().startswith("-") and not line.startswith(" "):
                k, _, v = line.partition(":")
                meta[k.strip()] = v.strip().strip('"\'')
    d["_meta"] = meta
    return d


def analyze(ep: Path) -> dict:
    """One episode -> stats + a list of human-readable problems. Pure; the CLI only prints it."""
    d = load(ep)
    t, cam_no, online = d["t"], d["cam_frame_no"], d["online"]
    goal, present = d["action_goal"], d["present_pos"]
    T = len(t)
    r: dict = {"path": str(ep), "T": T, "meta": d["_meta"], "problems": []}
    if T == 0:
        r["problems"].append("empty episode")
        return r

    r["duration_s"] = float(t[-1] - t[0])
    r["hz"] = (T - 1) / r["duration_s"] if r["duration_s"] > 0 else 0.0
    r["n_servos"] = int(goal.shape[1])

    n_png = len(list((ep / "frames").glob("*.png"))) if (ep / "frames").is_dir() else 0
    r["frames_on_disk"] = n_png

    # a REPEATED cam_frame_no means that step reused the previous image (the 10 Hz beat outran the
    # camera) -- the action moved while the observation stood still, which is what teaches lag
    r["stale_frac"] = float(np.mean(np.diff(cam_no) == 0)) if T > 1 else 0.0
    r["offline_frac"] = float(np.mean(online == 0))
    r["offline_per_servo"] = [float(x) for x in np.mean(online == 0, axis=0)]

    span = (goal.max(axis=0) - goal.min(axis=0)).astype(int)
    r["goal_span_ticks"] = [int(x) for x in span]
    r["stuck_servos"] = [i for i, s in enumerate(span) if s < STUCK_TICKS]

    # how well the hardware followed the command (tendon slack, torque off, a stalled servo)
    err = np.abs(goal - present)[online == 1] if np.any(online == 1) else np.array([0])
    r["track_err_median"] = float(np.median(err))
    r["track_err_p95"] = float(np.percentile(err, 95))

    seen = d["object_id"] >= 0
    r["object_seen_frac"] = float(np.mean(seen))

    if n_png != T:
        r["problems"].append(f"{n_png} PNGs but T={T} — frames and rows disagree")
    if r["stale_frac"] > STALE_WARN:
        r["problems"].append(f"{r['stale_frac']:.0%} held camera frames — the observation lags the "
                             f"action (raise the camera FPS or record slower)")
    if r["offline_frac"] > OFFLINE_WARN:
        worst = int(np.argmax(r["offline_per_servo"]))
        r["problems"].append(f"{r['offline_frac']:.1%} of servo-steps offline (worst: column {worst}"
                             f" at {r['offline_per_servo'][worst]:.0%}) — check the bus/power")
    if r["duration_s"] < SHORT_WARN_S:
        r["problems"].append(f"only {r['duration_s']:.1f}s long — mis-click?")
    if r["object_seen_frac"] < OBJECT_WARN:
        r["problems"].append(f"object marker visible in only {r['object_seen_frac']:.0%} of steps")
    if r["stuck_servos"]:
        r["problems"].append(f"servo column(s) {r['stuck_servos']} never moved (<{STUCK_TICKS} ticks)")
    if r["meta"].get("outcome") == "fail":
        r["problems"].append("labelled outcome: fail — keep it, but don't train on it unlabelled")
    return r


def contact_sheet(ep: Path, out: Path, cols: int = 6, rows: int = 3) -> Path | None:
    """A grid of evenly-spaced frames + the goal traces underneath: is this the motion you meant?"""
    import cv2
    files = sorted((ep / "frames").glob("*.png"))
    if not files:
        return None
    picks = [files[int(i)] for i in np.linspace(0, len(files) - 1, min(cols * rows, len(files)))]
    tiles = [cv2.imread(str(f)) for f in picks]
    h, w = tiles[0].shape[:2]
    sheet = np.zeros((rows * h, cols * w, 3), np.uint8)
    for i, tile in enumerate(tiles):
        rr, cc = divmod(i, cols)
        sheet[rr * h:(rr + 1) * h, cc * w:(cc + 1) * w] = cv2.resize(tile, (w, h))
        cv2.putText(sheet, str(files.index(picks[i])), (cc * w + 6, rr * h + 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

    goal = np.load(ep / "episode.npz")["action_goal"].astype(float)
    ph, pw = 220, cols * w
    plot = np.full((ph, pw, 3), 24, np.uint8)
    lo, hi = goal.min(), goal.max()
    rng = max(hi - lo, 1.0)
    for c in range(goal.shape[1]):
        col = tuple(int(v) for v in np.array(
            [(c * 67) % 255, 120 + (c * 43) % 135, 200 - (c * 31) % 150]))
        pts = np.stack([np.linspace(0, pw - 1, len(goal)),
                        ph - 10 - (goal[:, c] - lo) / rng * (ph - 20)], 1).astype(np.int32)
        cv2.polylines(plot, [pts], False, col, 1, cv2.LINE_AA)
    cv2.putText(plot, "action_goal (ticks) over the episode", (8, 16),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
    cv2.imwrite(str(out), np.vstack([sheet, plot]))
    return out


def _print(r: dict) -> None:
    m = r["meta"]
    print(f"\n{r['path']}")
    if r["T"] == 0:
        print("  EMPTY")
        return
    print(f"  build {m.get('build', '?')} · task {m.get('task', '?')} · outcome "
          f"{m.get('outcome', '?')}")
    print(f"  T={r['T']} steps · {r['duration_s']:.1f}s · {r['hz']:.1f} Hz · "
          f"{r['n_servos']} servos · {r['frames_on_disk']} PNGs")
    print(f"  held frames {r['stale_frac']:.0%} · offline {r['offline_frac']:.2%} · "
          f"object seen {r['object_seen_frac']:.0%}")
    print(f"  goal→present error: median {r['track_err_median']:.0f} ticks, "
          f"p95 {r['track_err_p95']:.0f}")
    print(f"  goal span/servo (ticks): {r['goal_span_ticks']}")
    for p in r["problems"]:
        print(f"  ⚠ {p}")
    if not r["problems"]:
        print("  ✓ no problems found")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("path", type=Path, help="an episode_NNN dir, or a task dir holding them")
    ap.add_argument("--sheet", action="store_true", help="also write contact_sheet.png per episode")
    a = ap.parse_args()
    if not a.path.exists():
        print(f"not found: {a.path}", file=sys.stderr)
        return 2

    eps = ([a.path] if (a.path / "episode.npz").exists()
           else sorted(p for p in a.path.glob("episode_*") if (p / "episode.npz").exists()))
    if not eps:
        print(f"no episodes under {a.path}", file=sys.stderr)
        return 2

    bad = 0
    for ep in eps:
        r = analyze(ep)
        _print(r)
        bad += bool(r["problems"])
        if a.sheet:
            out = contact_sheet(ep, ep / "contact_sheet.png")
            if out:
                print(f"  sheet → {out}")
    if len(eps) > 1:
        print(f"\n{len(eps)} episodes · {bad} with warnings")
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
