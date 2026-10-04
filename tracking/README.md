# WiLoR teleop — decoupled hand-tracking streamer

Real-time 3D hand teleop for handlab using **WiLoR** ([arxiv 2409.12259](https://arxiv.org/abs/2409.12259)),
replacing MediaPipe (Z-jitter, unreliable laterally-mounted thumb). WiLoR is torch-heavy, so it runs
as its **own process** in a **separate venv** — it can't live in the lean handlab `.app`, and running
it standalone gives it camera access via cv2 (permission is attributed to your Terminal).

```
[Brio] → wilor_teleop.py  (WiLoR → MANO keypoints → per-finger norms)
              │  UDP :8770  {"fingers":{"index":{"flex":..,"curl":..,"spread":..}, ...}}
              ▼
        handlab  (TeleopStreamReceiver → 'wilor' mode → OneEuro → robot ranges → set_goal_positions)
```
The tracker owns the **human** side (norms ∈ flex/curl [0,1], spread [-1,1]); handlab owns the
**robot** side (ranges from `calibration.yaml`) and all the safety (E-STOP, force/velocity caps,
torque gate). Smoothing is handlab's existing OneEuro — the tracker adds **no** EMA (no double-smooth).

## 1 · Setup (one-time)

**Separate venv** (Python 3.10, WiLoR-mini's requirement — NOT the handlab 3.12 venv):
```bash
python3.10 -m venv ~/.venvs/wilor
~/.venvs/wilor/bin/pip install --upgrade pip
~/.venvs/wilor/bin/pip install torch torchvision            # Apple-Silicon wheels (MPS)
~/.venvs/wilor/bin/pip install git+https://github.com/warmshao/WiLoR-mini
~/.venvs/wilor/bin/pip install opencv-python numpy
```

**MANO model.** WiLoR-mini auto-downloads its checkpoints on first run. MANO itself is license-gated:
if init complains about a missing `MANO_RIGHT.pkl`, register at **https://mano.is.tue.mpg.de**,
download `mano_v*_*.zip`, and place `MANO_RIGHT.pkl` where WiLoR-mini expects it (its error message
prints the path — usually a `mano_data/` or `_DATA/data/mano/` folder under the installed package).

**Camera permission (macOS).** A plain cv2 grab is attributed to your **Terminal/iTerm** — grant it
in System Settings → Privacy & Security → Camera. **Quit the handlab `.app` if it's holding the external camera**
(one process owns a camera at a time), or point the tracker at a second camera with `--camera N`.

## 2 · Run

```bash
# 1) handlab (dev mode is fine — the tracker owns the camera, not handlab):
~/.venvs/handlab/bin/python -m handlab
#    → set teleop mode to "wilor" and enable teleop (MCP: setTeleopMode('wilor'); or the Teleop tab).
#    → torque on (Control tab).  Keep a hand on E-STOP.

# 2) the tracker (separate venv):
~/.venvs/wilor/bin/python tracking/wilor_teleop.py --show
#    --show     overlay window with keypoints + live norms
#    --fast     WiLoR-mini fast path (if available)
#    --device   auto | mps | cpu  (auto picks MPS on Apple Silicon)
#    --camera N pick the Brio if it isn't index 0
#    --hand right|left
```

## 3 · The spike gate — FPS first

The tracker prints its **FPS every second**. That's the make-or-break number:
- **≥ ~10 FPS** → good. handlab commands servos at only ~10 Hz (telemetry cap), so ~10–15 FPS is
  plenty; the OneEuro absorbs the rest.
- **< ~6–8 FPS** → try `--fast`, a smaller `--width`, or frame-skipping; if still too slow, this is
  the signal to move WiLoR to a cloud GPU (stream frames up / norms down) — re-decide then.

MPS note: some ops fall back to CPU (`PYTORCH_ENABLE_MPS_FALLBACK=1` is set automatically) — that's
expected and only slows those ops.

## 4 · Verify + tune

1. **Keypoint order.** On the first detected frame the tracker prints the detection keys + the
   keypoints shape and a couple of sample joints. **Confirm `FINGER_KP` in `wilor_teleop.py` matches
   your WiLoR-mini build** (MANO 21-joint OpenPose order: 0 wrist, 1-4 thumb, 5-8 index, 9-12 middle,
   13-16 ring, 17-20 pinky). If the output dict keys differ, adjust `extract_keypoints()`.
2. **Angle sanity** (`--show`): open hand → all norms ~0; fist → curl→1, flex→~0.5; splay → spread
   moves. Verified against synthetic geometry; real numbers refine the ranges.
3. **Calibrate to YOUR hand (CRAFT biological limits).** In the `--show` window, hold each extreme
   and press the key — this replaces the guessed `DEFAULT_RANGES` with your real range:
   - `o` = ✋ open/flat (the 0 anchor for flex/curl + neutral spread + the open pinch aperture)
   - `f` = ✊ fist (the 1 anchor for flex/curl)
   - `p` = 🖐 fingers splayed wide (the spread span) — **splay hard**: a span under
     `MIN_SPREAD_SPAN` (8°) is tracker noise, and that spread is then held NEUTRAL rather than
     normalized against a bias (this is why a dead-straight middle finger used to read s≈−0.7)
   - `v` = ✌️ peace (the second spread anchor). 🖐 leaves the MIDDLE finger sitting mid-hand — it
     measured 2.5° there, i.e. dead — while ✌️ drives it out properly. Each finger's span comes
     from whichever of the two captures moved it furthest from its open angle, but **only captures
     where that finger was straight count**: ✌️ curls the ring finger, and the drifted −18° it reads
     there beat the real +15° from 🖐 and ran the finger backwards on the robot. The capture line
     names any finger it had to skip (`spread not anchored here: ring — curled`).
   - `t` = 🤏 pinch, thumb pad TOUCHING the fingertips (the 1 anchor for the `opp` aperture)
   - `c` = clear → back to defaults
   Saved to `~/.handlab/wilor_calib.json`, loaded on start; the overlay shows `calib open.. fist..
   spread.. pinch..`. flex/curl anchor open→fist; spread is bipolar around open. Re-capture anytime.
   **The window must have focus for the keys to register.** If a DOF still runs the wrong way after
   calibrating, that's a sign issue — flip it (per-finger, `INVERT`).
4. **The thumb runs one path between two SAVED POSES, not per-DOF.** Its tracked spread is dropped
   (the overlay marks it `(ign)`): opposition and abduction are both in-plane, so the decomposition
   cannot separate them, and a ~9° operator span against a 160° robot range saturated the norm and
   threw the thumb around. Instead handlab reads two anchors from the app's saved poses — the newest
   `pinch*` (thumb opposed at the fingertips) and the newest `peace*` (thumb folded in against the
   palm) — and drives REST → one of them. Your thumb's **flex/curl say how far** along (✌️ reads
   `f1.00 c0.72`, 3️⃣ reads `f0.15 c0.00`) and **`opp` picks which end**. Measured on the real hand,
   ✌️ needs spread 90.9 / flex 197.4 where the pinch needs 62.8 / 140.5 — one anchor cannot cover
   both. To retune the thumb: pose it in the app, save as `pinch_vN` / `peace_vN`, re-enable teleop.
   A **curled** finger's spread is faded out too (`SPREAD_CONF_FULL/MIN`): only the part of the bone
   still in the palm plane can be measured, so a fist used to splay the ring finger sideways.
5. **End-to-end, torque OFF first:** confirm handlab receives frames and the twin follows within its
   limits; hit E-STOP mid-stream (it halts within one loop); then torque-on and drive the real hand.

## Notes
- Wire format: one JSON object per UDP datagram, `{"t":sender_monotonic,"fingers":{finger:{dof:norm}}}`.
  Latest-wins; UDP drops are harmless. Finger keys are ROBOT ids (thumb/index/mid/ring); the human
  pinky is ignored.
- Nothing here imports handlab and nothing in handlab imports torch — the two are fully decoupled;
  the only contract is the UDP norm frame (handlab side: `handlab/teleop_stream.py` + the `'wilor'`
  mode in `handlab/bridge_teleop.py`).
