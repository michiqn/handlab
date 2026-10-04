# handlab — QML control center

The app's Qt Quick (Qt 6 / Qt Quick Controls 2) UI. Normally driven by the Python
`Bridge` (context property `handlab`): live MuJoCo twin frames in the viewport,
real catalogs, telemetry, hardware, agent control. **Also runs standalone**
(`qml Main.qml`) — without a bridge it falls back to the `MockCatalog` singleton
and a schematic viewport (useful for pure-UI work).

## Run

From this folder:

```bash
qml Main.qml
# or
qmlscene Main.qml
```

(Needs a Qt 6 install with Qt Quick Controls 2. The local `qmldir` registers the
`Theme` and `MockCatalog` singletons, so run the command from inside this directory.)

## Files

| File | Role |
|------|------|
| `Main.qml` | `ApplicationWindow`. All controller logic + layout (top bar / viewport / swapping right panel: Control · Agent · Teleop) + the floating Agent bubble. |
| `Theme.qml` | Singleton. Colours, spacing, radii and **per-DOF identity colours** are data here. `mode` drives dark ⇄ light; `dofColor(group)` resolves a DOF's colour. |
| `MockCatalog.qml` | Singleton. The standalone fallback catalog (finger types, mounts) when no Bridge is present. |
| `qmldir` | Registers `Theme` and `MockCatalog` as singletons. |
| `TopBar.qml` | Product mark, connection chip, Control/Teleop tabs, theme toggle, red STOP. |
| `ViewportPane.qml` | Live digital twin: `image://twin` frames + MuJoCo free-camera controls (orbit/zoom/pan) + orientation gizmo, HUD, physics toggle, e-stop overlay. Schematic base + slots only as the no-bridge fallback. |
| `ControlPanel.qml` | Joint control — header actions + a `Repeater` of finger cards / empty state. |
| `FingerControlCard.qml` | Per-finger: status header, per-servo torque switches, per-DOF stepper (number field + −/+) with a bar+knob dial beside it. |
| `PosesPanel.qml` | Poses & clips: save/recall named poses, record + play clips, drag-to-reorder. |
| `ConnectPopup.qml` | Hardware connect: port scan, baud, persistent Set home, per-servo/finger direction (`sign`) flips, revive. |
| `AgentPanel.qml` | The Agent cockpit (reached via the floating bubble): ARM/STOP, MCP status chip, live camera view + device picker + twin/cam snapshot-source toggle, demo recorder, tool-call log. |
| `TeleopPanel.qml` | WiLoR hand-tracking teleop: enable/stop, the tracker launch command, workspace camera. |
| `ActionButton.qml` | Small reusable pill button used by Control/Poses. |

## Data-driven by design

* A **machine** = a base + an adapter + fingers snapped onto the adapter's
  **typed slots**. Slots are adapter data (`palm4` declares thumb/index/mid/ring);
  a different adapter could declare any number/layout — the UI renders whatever
  the adapter provides.
* A **finger type** is a *mechanism* (`Claw finger`, `Claw thumb`). It declares
  its own DOF set, slot type and motor count — placement is a property of the
  slot, never of the type. Adding a mechanism means adding a library folder
  (`type.yaml` + MJCF + meshes) and referencing it from a build YAML.
* Slot labels ("Index slot", …) and their layout in the twin viewport come
  straight from the adapter's `ports` array.
* Counts scale automatically (3 → 15+ motors). Nothing is hardcoded to a fixed
  finger or joint count; every list is a `Repeater` over the model.
* With the Bridge, the UI is seeded from the booted build (`HANDLAB_BUILD`, default
  `claw4f`). Standalone, `loadPreset("claw" | "empty")` picks a mock build.

## States covered

servo online / hot / offline · torque on/off (per servo) · moving vs holding ·
e-stopped (global red) · armed/disarmed (agent) · camera live/off ·
empty "nothing built yet".
