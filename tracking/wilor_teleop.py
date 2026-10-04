#!/usr/bin/env python3
"""Decoupled WiLoR hand-tracking teleop streamer for handlab.

Runs in its OWN venv (torch + WiLoR-mini — too heavy for the handlab `.app`). Captures the Brio,
runs WiLoR (MPS on Apple Silicon), decomposes the MANO 3D keypoints into per-finger flexion / curl /
abduction, normalizes each against a DEFAULT biological range, and STREAMS the per-robot-finger norms
to handlab over UDP. handlab's `TeleopStreamReceiver` + the 'wilor' teleop mode do the rest (OneEuro
smoothing → robot-range mapping → clamped `set_goal_positions`), so THIS side owns only the human
half. No EMA here — handlab's OneEuro is the single smoothing stage.

Setup + run:  see tracking/README.md.  Quick:
    ~/.venvs/wilor/bin/python tracking/wilor_teleop.py --show          # local M4, MPS, overlay
In handlab: setTeleopMode("wilor") → enable teleop → torque on. Grant Terminal camera access first;
quit the handlab .app if it holds the Brio (one process owns a camera).

CALIBRATE (CRAFT biological limits) — in the --show window, key your extremes once:
    o = ✋ open/flat   ·   f = ✊ fist   ·   p = 🖐 splayed (spread)
    t = 🤏 touch/pinch (thumb pad TOUCHING the fingertips — anchors the opposition axis)
    v = ✌️ peace (the middle finger's real spread — 🖐 leaves it sitting mid-hand)
    c = clear → defaults
Your captured range replaces the guessed defaults so the norms scale to YOUR hand (persisted to
~/.handlab/wilor_calib.json, loaded on start). flex/curl anchor open→fist; spread is bipolar around
open, spanning to whichever of the 🖐/✌️ captures moved that finger furthest.

SPIKE NOTE: the first job is the FPS readout (printed each second). If WiLoR can't clear ~10 FPS on
the M4 even with --fast, that's the signal to revisit compute (cloud) before tuning the mapping.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import socket
import time

import cv2
import numpy as np

# ── MANO 21-keypoint layout (OpenPose-style order HaMeR/WiLoR use for pred_keypoints_3d) ──
# 0 wrist · 1-4 thumb(cmc,mcp,ip,tip) · 5-8 index · 9-12 middle · 13-16 ring · 17-20 pinky
# Each non-thumb finger is [mcp, pip, dip, tip]. Verify against the FIRST-FRAME dump (--show or the
# printed structure) — if your WiLoR-mini build orders joints differently, fix these indices.
WRIST = 0
FINGER_KP = {          # robot-finger id -> (mcp/base, pip, dip, tip) keypoint indices
    "thumb": (1, 2, 3, 4),
    "index": (5, 6, 7, 8),
    "mid":   (9, 10, 11, 12),
    "ring":  (13, 14, 15, 16),
}
PINKY_MCP, INDEX_MCP, MID_MCP = 17, 5, 9

# Default HUMAN biological ranges (degrees) — replace per-operator via calibration later. flex/curl:
# (straight_lo, closed_hi). spread: (center, half_span) → bipolar. Tuned conservatively; adjust after
# watching --show. The thumb is the least certain (opposition geometry) — expect to tune it.
DEFAULT_RANGES = {
    "index": {"flex": (5.0, 90.0),  "curl": (5.0, 160.0), "spread": (0.0, 20.0)},
    "mid":   {"flex": (5.0, 90.0),  "curl": (5.0, 160.0), "spread": (0.0, 15.0)},
    "ring":  {"flex": (5.0, 90.0),  "curl": (5.0, 160.0), "spread": (0.0, 20.0)},
    "thumb": {"flex": (5.0, 70.0),  "curl": (5.0, 90.0),  "spread": (0.0, 45.0)},
}

# (finger, dof) pairs that read BACKWARDS vs the robot after calibration → invert. flex/curl are
# unipolar [0,1] so they invert as (1-n); spread is bipolar [-1,1] so it inverts as (-n). Tuned live
# on the real hand (the robot's per-finger spread convention differs from the human's raw sign).
INVERT = {("ring",  "spread"),   # ring sits opposite index; its robot +spread runs the other way
          ("mid",   "spread"),   # middle-finger spread read inverted vs the robot mid spread (live)
          ("thumb", "spread")}   # thumb spread read inverted vs the robot thumb spread (live)
#         index spread runs correct un-flipped

# A spread capture narrower than this is NOISE, not range (the tracker jitters ~1-2°): normalizing
# against it saturates the norm at ±1 on the slightest motion. It is also wrong to fall back to the
# DEFAULT_RANGES then — those are centered on 0, while the operator's neutral raw angle is not (the
# middle finger measured a 2.5° span and sat at s≈-0.7 with the finger dead straight) — so a
# degenerate span holds that spread NEUTRAL instead. Physiological: the middle finger barely abducts.
MIN_SPREAD_SPAN = 8.0

# A CURLED finger has no trustworthy spread either: `spread` is the in-plane angle of the proximal
# bone, so it is only as good as the part of that bone still lying IN the palm plane — cos(flex),
# with flex the raw elevation. Measured on the captures: 0.99 with the finger straight, but 0.23-0.33
# in a fist, where the angle turns to noise/drift (the ring finger read spread≈+1.0 curled into a ✌️
# and the robot splayed it sideways; the fist capture itself already stores 0.97). Weight the reading
# by that fraction: full below ~32° of flexion, neutral past ~57°. A curled finger barely abducts
# anyway, so fading to neutral is also the physiological answer.
SPREAD_CONF_FULL = 0.85     # cos(32°) — in-plane enough to trust the spread angle as measured
SPREAD_CONF_MIN = 0.55      # cos(57°) — below this the spread reading is discarded (neutral)

# THUMB OPPOSITION: the CMC opposition can't be mapped per-DOF (opposition ≈ abduction in the generic
# decomposition — both are in-plane, so the tracker can't tell them apart). Instead send ONE scalar
# 'opp' from the hand-normalized thumb-TIP→index-TIP distance = the PINCH APERTURE (near 0 when the
# tips touch, large when open). Tip→tip, NOT tip→knuckle: a fingertip pinch keeps the thumb far from
# the MCP, so the knuckle metric saturated around 0.4 on a real pinch (measured live). handlab blends
# home → the opposed pinch pose. The o/t captures override these defaults with YOUR measured anchors.
THUMB_OPP_OPEN = 0.9      # thumb-tip→index-tip distance (÷ hand size) when the hand is OPEN → opp 0
THUMB_OPP_CLOSED = 0.15   #                                   ... when the tips TOUCH (pinch) → opp 1

CALIB_PATH = os.path.expanduser("~/.handlab/wilor_calib.json")   # per-operator biological limits


def load_calib():
    """Load the per-operator open/fist/spread captures (raw angles). Missing/invalid → {} → the
    normalizer falls back to DEFAULT_RANGES."""
    try:
        if os.path.exists(CALIB_PATH):
            with open(CALIB_PATH) as f:
                return json.load(f) or {}
    except Exception:
        pass
    return {}


def save_calib(cal):
    try:
        os.makedirs(os.path.dirname(CALIB_PATH), exist_ok=True)
        with open(CALIB_PATH, "w") as f:
            json.dump(cal, f, indent=2)
    except Exception as e:
        print("[wilor] calib save failed:", e)


def curled_in(raw):
    """Fingers too curled in this capture for their spread to mean anything (see _spread_gate) —
    they cannot anchor a spread span, so say so at capture time instead of silently flipping one."""
    return [f for f in ("index", "mid", "ring") if _spread_gate(raw.get(f, {}).get("flex", 0.0)) <= 0.0]


def calib_status(cal):
    return " ".join(f"{p}{'ok' if (cal or {}).get(p) else '-'}"
                    for p in ("open", "fist", "spread", "pinch", "peace"))


# ── geometry helpers ──
def _unit(v):
    n = np.linalg.norm(v)
    return v / n if n > 1e-9 else v


def _angle(a, b):
    """Unsigned angle (deg) between two vectors."""
    a, b = _unit(a), _unit(b)
    return math.degrees(math.acos(float(np.clip(np.dot(a, b), -1.0, 1.0))))


def _norm_uni(angle, lo, hi):
    return float(np.clip((angle - lo) / (hi - lo + 1e-9), 0.0, 1.0))


def _norm_bi(angle, center, half):
    return float(np.clip((angle - center) / (half + 1e-9), -1.0, 1.0))


def _spread_gate(flex_raw):
    """How much of the proximal bone still lies in the palm plane (cos of its elevation) -> how far
    to trust this finger's spread. 1 = straight finger, 0 = curled (see SPREAD_CONF_MIN)."""
    conf = math.cos(math.radians(flex_raw))
    return float(np.clip((conf - SPREAD_CONF_MIN) / (SPREAD_CONF_FULL - SPREAD_CONF_MIN), 0.0, 1.0))


def palm_frame(kp):
    """Right-handed palm frame from the keypoints: e_fwd (wrist→middle-mcp), e_norm (palm normal),
    e_lat (radial↔ulnar). Sign of e_norm/e_lat is consistent per frame — the norm ranges absorb it."""
    e_fwd = _unit(kp[MID_MCP] - kp[WRIST])
    e_norm = _unit(np.cross(kp[INDEX_MCP] - kp[WRIST], kp[PINKY_MCP] - kp[WRIST]))
    e_lat = _unit(np.cross(e_norm, e_fwd))
    return e_fwd, e_norm, e_lat


def raw_angles(kp):
    """MANO keypoints (21,3) → {finger: {flex, curl, spread}} RAW angles (deg), pre-normalization.
    flex and spread are DECOUPLED axes (fixes the spread→flex cross-talk measured in the calib data):
      flex   = elevation of the proximal bone OUT of the palm plane (toward palm = flexion); spreading
               is in-plane so it does not change this.
      spread = the IN-plane sideways angle (proximal bone projected onto the palm plane); flexing is
               out-of-plane so it does not change this.
      curl   = total finger closure (proximal↔distal bone angle) — robust as-is.
    Absolute sign of flex/spread is irrelevant — the open/fist/spread calibration anchors direction."""
    e_fwd, e_norm, e_lat = palm_frame(kp)
    out = {}
    for finger, (mcp, pip, dip, tip) in FINGER_KP.items():
        v_prox = kp[pip] - kp[mcp]                          # proximal bone
        v_dist = kp[tip] - kp[dip]                          # distal bone
        vp = _unit(v_prox)
        v_ip = v_prox - float(np.dot(v_prox, e_norm)) * e_norm      # proximal bone in the palm plane
        out[finger] = {
            "flex": math.degrees(math.asin(float(np.clip(np.dot(vp, e_norm), -1.0, 1.0)))),   # out-of-plane
            "curl": _angle(v_prox, v_dist),                                                   # closure
            "spread": math.degrees(math.atan2(float(np.dot(v_ip, e_lat)),                     # in-plane
                                              float(np.dot(v_ip, e_fwd)))),
        }
    # THUMB opposition signal: hand-normalized thumb-TIP → index-TIP distance = the pinch aperture
    # (≈0 when the tips touch). Robust where the angle decomposition fails (opposition and abduction
    # are both in-plane), and far more separable than tip→knuckle (a fingertip pinch keeps the thumb
    # tip away from the MCP).
    hand = float(np.linalg.norm(kp[MID_MCP] - kp[WRIST])) or 1.0
    out["thumb"]["opp_d"] = float(
        np.linalg.norm(kp[FINGER_KP["thumb"][3]] - kp[FINGER_KP["index"][3]]) / hand)
    return out


def normalize(raw, cal):
    """RAW per-finger angles → norms (flex/curl ∈[0,1], spread ∈[-1,1]). Uses the PER-OPERATOR
    calibration `cal` = {"open": {finger: raw}, "fist": {...}, "spread": {...}} when present (any part
    missing → that DOF falls back to DEFAULT_RANGES). flex/curl anchor open→fist; spread is bipolar
    around the open capture with the span to the spread capture. This is the CRAFT normalize-then-map:
    the operator's biological range → [0,1]/[-1,1], and handlab maps that to the robot's limits."""
    op, fs = (cal or {}).get("open", {}), (cal or {}).get("fist", {})
    spread_caps = [(cal or {}).get(k, {}) for k in ("spread", "peace")]
    out = {}
    for finger, r in raw.items():
        d = DEFAULT_RANGES[finger]
        fc = {}
        for k in ("flex", "curl"):
            if finger in op and finger in fs and abs(fs[finger][k] - op[finger][k]) >= 3.0:
                fc[k] = _norm_uni(r[k], op[finger][k], fs[finger][k])            # YOUR range
            else:
                fc[k] = _norm_uni(r[k], *d[k])                                   # default fallback
        center = op.get(finger, {}).get("spread")
        # WIDEST anchor wins: 🖐 splays index/ring but leaves the middle finger sitting mid-hand
        # (2.5° = noise), while ✌️ drives it hard to the ulnar side. Whichever capture moved this
        # finger furthest from its open angle defines its span — sign included. But only captures
        # where that finger was STRAIGHT may vote: ✌️ curls the ring finger, and the drifted angle
        # it reads there (−18° against a real +15° from 🖐) is not just noise — being the larger
        # magnitude it won, and flipped the finger's whole direction on the robot.
        spans = [c[finger]["spread"] - center for c in spread_caps
                 if center is not None and finger in c and _spread_gate(c[finger]["flex"]) > 0.0]
        span = max(spans, key=abs) if spans else None
        if span is not None and abs(span) >= MIN_SPREAD_SPAN:
            spread = float(np.clip((r["spread"] - center) / span, -1.0, 1.0))     # SIGNED: splayed → +1
        elif center is not None:
            spread = 0.0            # calibrated but every span is noise -> neutral, see MIN_SPREAD_SPAN
        else:
            spread = _norm_bi(r["spread"], *d["spread"])
        spread *= _spread_gate(r["flex"])    # a curled finger has no readable spread — fade to neutral
        dofs = {"flex": fc["flex"], "curl": fc["curl"], "spread": spread}
        for dof, v in dofs.items():                          # per-DOF direction correction
            if (finger, dof) in INVERT:
                dofs[dof] = -v if dof == "spread" else 1.0 - v
        out[finger] = dofs
    # THUMB: all norms are sent, but handlab uses them selectively -- its flex/curl drive the thumb
    # (they carry the tuck: ✌️ reads f1.00 c0.66 where 3️⃣ reads f0.15 c0.00), while its SPREAD is
    # dropped and replaced by 'opp'. Reason: opposition and abduction are both in-plane, so the
    # decomposition can't separate them, and the operator's thumb-spread span is ~9° against a 160°
    # robot range -> that norm saturated and threw the thumb around on any gesture that moved it.
    # 'opp' is the one cleanly separated thumb channel. Anchors: prefer the PER-OPERATOR captures — 'o' (open hand) and 't' (a real pinch,
    # fingers touching) both store the measured opp_d — over the guessed defaults. A degenerate
    # capture pair (span < 0.1) falls back to the defaults rather than a hair-trigger opp.
    opp_d = raw.get("thumb", {}).get("opp_d")
    if opp_d is not None and "thumb" in out:
        open_d = (cal or {}).get("open", {}).get("thumb", {}).get("opp_d", THUMB_OPP_OPEN)
        closed_d = (cal or {}).get("pinch", {}).get("thumb", {}).get("opp_d", THUMB_OPP_CLOSED)
        if open_d - closed_d < 0.1:
            open_d, closed_d = THUMB_OPP_OPEN, THUMB_OPP_CLOSED
        out["thumb"]["opp"] = float(np.clip((open_d - opp_d) / (open_d - closed_d + 1e-9), 0.0, 1.0))
        out["thumb"]["opp_d"] = float(opp_d)            # raw distance, overlay-only (receiver ignores it)
    return out


# ── WiLoR output extraction (defensive: exact keys vary by build — first frame prints the structure) ──
def _to_np(x):
    try:
        import torch
        if isinstance(x, torch.Tensor):
            x = x.detach().cpu().numpy()
    except Exception:
        pass
    return np.asarray(x)


def extract_keypoints(det):
    """Pull (21,3) keypoints from one WiLoR detection dict, trying the known key paths."""
    for path in (("wilor_preds", "pred_keypoints_3d"), ("pred_keypoints_3d",), ("keypoints_3d",)):
        node = det
        ok = True
        for k in path:
            if isinstance(node, dict) and k in node:
                node = node[k]
            else:
                ok = False
                break
        if ok:
            arr = _to_np(node)
            arr = arr.reshape(-1, 3) if arr.ndim > 2 else arr
            if arr.shape[-1] == 3 and arr.shape[0] >= 21:
                return arr[:21]
    return None


def is_right(det):
    v = det.get("is_right", det.get("right", 1.0)) if isinstance(det, dict) else 1.0
    try:
        return bool(_to_np(v).astype(float).flatten()[0] >= 0.5)
    except Exception:
        return bool(v)


def dump_structure(det, kp):
    print("[wilor] first detection keys:", list(det.keys()) if isinstance(det, dict) else type(det))
    if isinstance(det, dict) and "wilor_preds" in det and isinstance(det["wilor_preds"], dict):
        print("[wilor] wilor_preds keys:", list(det["wilor_preds"].keys()))
    if kp is not None:
        print("[wilor] keypoints_3d shape:", kp.shape, "wrist:", np.round(kp[0], 3),
              "index_tip:", np.round(kp[8], 3))
    print("[wilor] ^ verify FINGER_KP indices against these before trusting the mapping.")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--device", default="auto", help="auto | mps | cpu | cuda")
    ap.add_argument("--port", type=int, default=8770, help="handlab UDP port (must match §41 default)")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--camera", type=int, default=0, help="cv2 camera index (the Brio)")
    ap.add_argument("--width", type=int, default=640, help="downscale width for speed")
    ap.add_argument("--hand", choices=["right", "left"], default="right", help="which hand to track")
    ap.add_argument("--fast", action="store_true", help="WiLoR-mini fast path if available")
    ap.add_argument("--show", action="store_true", help="cv2 overlay window (keypoints + norms)")
    a = ap.parse_args()

    os.environ.setdefault("PYTORCH_ENABLE_MPS_FALLBACK", "1")   # some ops fall back to CPU on MPS
    import torch
    # MPS quirk: some WiLoR ops leave a tensor non-contiguous, so a later `.view()` raises
    # "view size is not compatible with input tensor's size and stride ... Use .reshape()".
    # Make `.view()` transparently fall back to `.reshape()` (a copy when needed) so predict() works.
    _orig_view = torch.Tensor.view
    def _safe_view(self, *a, **k):
        try:
            return _orig_view(self, *a, **k)
        except RuntimeError:
            return self.reshape(*a, **k)
    torch.Tensor.view = _safe_view
    if a.device == "auto":
        dev = "mps" if torch.backends.mps.is_available() else ("cuda" if torch.cuda.is_available() else "cpu")
    else:
        dev = a.device
    device = torch.device(dev)
    # fp16 is only reliable on CUDA: grid_sample has no CPU-Half kernel, and MPS fp16 is flaky → fp32.
    dtype = torch.float16 if dev == "cuda" else torch.float32
    # grid_sample results are non-contiguous → a downstream reshape/view throws on MPS (and the slice
    # here is non-contiguous everywhere). Force the output contiguous. WiLoR calls `F.grid_sample`,
    # so patching the module attribute is picked up at call time.
    import torch.nn.functional as _F
    _orig_gs = _F.grid_sample
    _F.grid_sample = lambda *aa, **kk: _orig_gs(*aa, **kk).contiguous()
    # MPS conv2d chokes on the non-contiguous slice WiLoR feeds the ViT (`x[:, :, :, 32:-32]` in
    # wilor.py) → "view size not compatible". Make conv2d inputs contiguous (no-op when already so).
    _orig_conv2d = _F.conv2d
    _F.conv2d = lambda inp, *aa, **kk: _orig_conv2d(inp.contiguous(), *aa, **kk)
    print(f"[wilor] device={dev} dtype={dtype}")

    from wilor_mini.pipelines.wilor_hand_pose3d_estimation_pipeline import WiLorHandPose3dEstimationPipeline
    kw = {"verbose": False}
    try:
        pipe = WiLorHandPose3dEstimationPipeline(device=device, dtype=dtype, **kw)
    except TypeError:
        pipe = WiLorHandPose3dEstimationPipeline(device=device, dtype=dtype)

    cap = cv2.VideoCapture(a.camera)
    if not cap.isOpened():
        raise SystemExit(f"Could not open camera {a.camera}. Grant Terminal camera access; quit the "
                         "handlab .app if it holds the Brio (one process owns a camera).")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dst = (a.host, a.port)
    print(f"[wilor] streaming norms → udp://{a.host}:{a.port}  (handlab: setTeleopMode('wilor') + enable)")

    want_right = a.hand == "right"
    cal = load_calib()
    last_raw = None                         # most recent RAW angles, for the calibration captures
    first = True
    err_shown = False
    n, t0 = 0, time.monotonic()
    fps = 0.0
    print(f"[wilor] calibration: {calib_status(cal)}   (in the --show window: o=open  f=fist  "
          "p=spread  t=touch/pinch  c=clear)")
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            if a.width and frame.shape[1] > a.width:
                s = a.width / frame.shape[1]
                frame = cv2.resize(frame, (a.width, int(frame.shape[0] * s)))
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            preds = []
            try:
                preds = pipe.predict(rgb, fast=True) if a.fast else pipe.predict(rgb)
            except TypeError:
                try:
                    preds = pipe.predict(rgb)                 # older WiLoR-mini: no `fast` kwarg
                except Exception:
                    preds = []
            except Exception:
                if not err_shown:                             # print the EXACT failure location once
                    import traceback
                    print("[wilor] predict() FAILED — full traceback (shown once) ↓↓↓")
                    traceback.print_exc()
                    print("[wilor] ↑↑↑ send me this — that's where the MPS .view() breaks")
                    err_shown = True

            # pick the chosen hand (default right); fall back to the first detection
            det = None
            dets = preds if isinstance(preds, (list, tuple)) else ([preds] if preds else [])
            for d in dets:
                if is_right(d) == want_right:
                    det = d
                    break
            if det is None and dets:
                det = dets[0]

            norms = None
            if det is not None:
                kp = extract_keypoints(det)
                if first:
                    dump_structure(det, kp)
                    first = False
                if kp is not None:
                    last_raw = raw_angles(kp)                # snapshot for the calibration captures
                    norms = normalize(last_raw, cal)
                    sock.sendto(json.dumps({"t": time.monotonic(), "fingers": norms}).encode(), dst)

            n += 1
            now = time.monotonic()
            if now - t0 >= 1.0:
                fps = n / (now - t0)
                n, t0 = 0, now
                tag = "" if norms else "  (no hand)"
                print(f"[wilor] {fps:5.1f} fps{tag}")

            if a.show:
                _overlay(frame, det, norms, fps, cal)
                cv2.imshow("WiLoR teleop  (q quit · o open · f fist · p spread · t touch/pinch · c clear)", frame)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
                elif key == ord("o") and last_raw:
                    cal["open"] = last_raw; save_calib(cal); print("[wilor] captured OPEN   →", calib_status(cal))
                elif key == ord("f") and last_raw:
                    cal["fist"] = last_raw; save_calib(cal); print("[wilor] captured FIST   →", calib_status(cal))
                elif key in (ord("p"), ord("v")) and last_raw:
                    which = "spread" if key == ord("p") else "peace"   # ✌️ = the middle finger's
                    cal[which] = last_raw; save_calib(cal)             #      only real spread
                    curled = curled_in(last_raw)
                    note = f"  (spread not anchored here: {', '.join(curled)} — curled)" if curled else ""
                    print(f"[wilor] captured {which.upper():6} →", calib_status(cal) + note)
                elif key == ord("t") and last_raw:
                    # a REAL firm pinch — thumb pad touching the fingertips — held while pressing 't'
                    cal["pinch"] = last_raw; save_calib(cal); print("[wilor] captured PINCH  →", calib_status(cal))
                elif key == ord("c"):
                    cal = {}; save_calib(cal); print("[wilor] calibration cleared → defaults")
    finally:
        cap.release()
        if a.show:
            cv2.destroyAllWindows()
        sock.close()


def _overlay(frame, det, norms, fps, cal=None):
    cv2.putText(frame, f"{fps:.1f} fps", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 220, 255), 2)
    cv2.putText(frame, f"calib {calib_status(cal)}", (10, 44),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (90, 200, 200), 1)
    if norms is None:
        cv2.putText(frame, "no hand", (10, 66), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 140, 255), 2)
        return
    y = 66
    for finger in ("thumb", "index", "mid", "ring"):
        d = norms.get(finger, {})
        thumb = "opp" in d
        txt = f"{finger:5s} f{d.get('flex',0):.2f} c{d.get('curl',0):.2f} s{d.get('spread',0):+.2f}"
        if thumb:      # the thumb's spread is DROPPED by handlab; 'opp' drives that axis instead
            txt += f"(ign)  -> OPP {d['opp']:.2f} (d{d.get('opp_d', 0):.2f})"
        cv2.putText(frame, txt, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (120, 255, 120), 1)
        y += 22


if __name__ == "__main__":
    main()
