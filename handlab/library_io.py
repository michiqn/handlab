"""Load library types (mounts, fingers) and build definitions from YAML on disk.

`yaml` is imported lazily so the package stays import-safe without PyYAML installed.
"""

from __future__ import annotations

import os
from pathlib import Path

from .models import (
    AdapterType, Socket, Build, Coupling, Dof, FingerInstance, FingerType, Hardware, MountType,
)

PKG_DIR = Path(__file__).resolve().parent
LIBRARY_DIR = PKG_DIR / "library"
BUILDS_DIR = PKG_DIR / "builds"


def _yaml():
    import yaml  # lazy
    return yaml


def _read(path: Path) -> dict:
    with open(path, "r") as f:
        return _yaml().safe_load(f) or {}


def load_finger_type(name: str, library_dir: Path | None = None) -> FingerType:
    d = (library_dir or LIBRARY_DIR) / "fingers" / name
    m = _read(d / "type.yaml")
    dofs = [
        Dof(
            key=x["key"], label=x.get("label", x["key"]), joint=x["joint"],
            range=tuple(x.get("range", (0.0, 1.0))), coupled=bool(x.get("coupled", False)),
        )
        for x in m.get("dofs", [])
    ]
    couplings = [
        Coupling(driver=c["driver"], follower=c["follower"], ratio=float(c.get("ratio", 1.0)))
        for c in m.get("couplings", [])
    ]
    return FingerType(
        name=m["name"], label=m.get("label", m["name"]), motors=int(m.get("motors", len(dofs))),
        dofs=dofs, model=m.get("model", "model.xml"),
        couplings=couplings, tuning=m.get("tuning", {}),
        ui=m.get("ui", {}), slot=m.get("slot", ""), dir=str(d),
    )


def load_mount_type(name: str, library_dir: Path | None = None) -> MountType:
    d = (library_dir or LIBRARY_DIR) / "mounts" / name
    m = _read(d / "mount.yaml")
    sockets = [
        Socket(
            id=s["id"], kind=s.get("kind", "arm"),
            pos=tuple(s.get("frame", {}).get("pos", (0, 0, 0))),
            euler=tuple(s.get("frame", {}).get("euler", (0, 0, 0))),
        )
        for s in m.get("sockets", [])
    ]
    mf = m.get("model_frame", {})
    return MountType(name=m["name"], label=m.get("label", m["name"]), sockets=sockets,
                     model=m.get("model"),
                     model_pos=tuple(mf.get("pos", (0, 0, 0))),
                     model_euler=tuple(mf.get("euler", (0, 0, 0))), dir=str(d))


def load_adapter_type(name: str, library_dir: Path | None = None) -> AdapterType:
    d = (library_dir or LIBRARY_DIR) / "adapters" / name
    m = _read(d / "adapter.yaml")
    ports = [
        Socket(
            id=p["id"], kind="port",
            pos=tuple(p.get("frame", {}).get("pos", (0, 0, 0))),
            euler=tuple(p.get("frame", {}).get("euler", (0, 0, 0))),
            accepts=p.get("accepts", ""),
            dof_ranges_deg={k: tuple(v) for k, v in (p.get("dof_ranges_deg") or {}).items()},
        )
        for p in m.get("ports", [])
    ]
    mf = m.get("mesh_frame", {})
    return AdapterType(
        name=m["name"], label=m.get("label", m["name"]), ports=ports,
        fits=m.get("fits", "center"), mesh=m.get("mesh"),
        mesh_scale=float(m.get("mesh_scale", 0.001)),
        mesh_pos=tuple(mf.get("pos", (0, 0, 0))), mesh_euler=tuple(mf.get("euler", (0, 0, 0))),
        dir=str(d),
    )


def list_library(library_dir: Path | None = None) -> dict[str, list[str]]:
    base = library_dir or LIBRARY_DIR
    def names(sub: str) -> list[str]:
        p = base / sub
        return sorted(c.name for c in p.iterdir() if c.is_dir()) if p.exists() else []
    return {"mounts": names("mounts"), "fingers": names("fingers"), "adapters": names("adapters")}


CALIBRATION_FILE = BUILDS_DIR / "calibration.yaml"              # legacy/bundled SEED (migration only)
# Canonical calibration lives in the USER dir so it SURVIVES .app rebuilds — the bundle's copy is
# gitignored + not packaged, so a `pyinstaller` rebuild would otherwise drop it and every servo would
# fall back to the +1 default sign (mirror-inverted hardware, tendons unwind). Reads prefer this;
# writes always land here; the bundled/dev file is only a one-time migration seed. (14→16.07 incident.)
USER_CALIBRATION_FILE = Path.home() / ".handlab" / "calibration.yaml"


def load_calibration() -> dict[int, dict]:
    """Per-servo calibration, machine-/hardware-bound (keyed by servo id):
      sign     — HARDWARE wiring direction (+1 / -1), PERSISTENT (a wiring fact).
      offset   — optional manual zero override; normally re-captured at each connect
                 (delta-home), because Extended Position wraps after a power cycle.
      range_deg — optional user [min, max] DOF-range override (degrees), user "edit limits" in
                  the Control tab. Sets the effective slider/joint/actuator range (sim + the
                  commanded servo travel). None → use the base/adapter range.
    Returns {id: {"sign","offset","range_deg"}}."""
    path = USER_CALIBRATION_FILE if USER_CALIBRATION_FILE.exists() else CALIBRATION_FILE
    if not path.exists():
        return {}
    m = _read(path)
    out: dict[int, dict] = {}
    for k, v in (m.get("servos") or {}).items():
        rd = v.get("range_deg")
        out[int(k)] = {"sign": int(v.get("sign", 1)),
                       "offset": (None if v.get("offset") is None else int(v["offset"])),
                       "range_deg": ([float(rd[0]), float(rd[1])] if rd else None),
                       "home_shaft": (None if v.get("home_shaft") is None else int(v["home_shaft"]))}
    return out


def save_calibration(cal: dict[int, dict]) -> None:
    """Persist sign (always) + offset + range_deg (each only when non-default).
    Preserves range_deg (the 'edit limits' override) for servos where `cal` doesn't specify it —
    so the HARDWARE path (setServoSign passes only sign/offset from the driver) never clobbers
    it. An explicit range_deg=None in `cal` clears it (reset)."""
    prev = load_calibration()
    servos = {}
    for sid, c in cal.items():
        sid = int(sid)
        entry = {"sign": int(c.get("sign", 1))}
        if c.get("offset") is not None:
            entry["offset"] = int(c["offset"])
        rd = c.get("range_deg", prev.get(sid, {}).get("range_deg"))   # absent → preserve; None → clear
        if rd:
            entry["range_deg"] = [float(rd[0]), float(rd[1])]
        hs = c.get("home_shaft", prev.get(sid, {}).get("home_shaft"))  # absent → preserve; None → clear
        if hs is not None:
            entry["home_shaft"] = int(hs)
        servos[sid] = entry
    for sid, c in prev.items():                     # keep range/home overrides for servos absent from `cal`
        if sid not in servos and (c.get("range_deg") or c.get("home_shaft") is not None):
            e = {"sign": int(c.get("sign", 1))}
            if c.get("range_deg"):
                e["range_deg"] = [float(c["range_deg"][0]), float(c["range_deg"][1])]
            if c.get("home_shaft") is not None:
                e["home_shaft"] = int(c["home_shaft"])
            servos[sid] = e
    USER_CALIBRATION_FILE.parent.mkdir(parents=True, exist_ok=True)
    with open(USER_CALIBRATION_FILE, "w") as f:
        f.write("# Servo calibration (machine-specific). sign = HARDWARE wiring direction (+1/-1).\n"
                "# offset = optional manual zero override (normally re-captured each connect).\n"
                "# range_deg = [min,max]° DOF-range override (Control-tab 'edit limits').\n"
                "# home_shaft = absolute encoder angle (0-4095) at home — recovers home every connect.\n")
        _yaml().safe_dump({"servos": servos}, f)


USER_POSES_DIR = Path.home() / ".handlab" / "poses"    # canonical — survives .app rebuilds (the
#                            bundled builds/ copy is user state the pyinstaller rebuild WIPES; the
#                            same incident class as calibration.yaml, fixed the same way 23.07)


def poses_path(scope: str) -> Path:
    return USER_POSES_DIR / f"{scope}.poses.yaml"


def load_poses(scope: str) -> list[dict]:
    """Saved poses + recordings for a build. Each item: {name, kind: pose|clip, ...}.
    Poses store `dofs` = {socket: [joint_deg, ...]} (per-DOF, in dof order, socket-keyed
    so they survive restarts). Clips add `dt` + `frames: [{socket: [...]}, ...]`.
    Reads the USER dir first; the bundled/dev builds/ file is a one-time migration seed."""
    p = poses_path(scope)
    if not p.exists():
        p = BUILDS_DIR / f"{scope}.poses.yaml"       # migration seed (pre-23.07 location)
    if not p.exists():
        return []
    return list((_read(p).get("items") or []))


def save_poses(scope: str, items: list[dict]) -> None:
    USER_POSES_DIR.mkdir(parents=True, exist_ok=True)
    with open(poses_path(scope), "w") as f:
        f.write("# Saved poses (snapshots) + recordings (clips) for this build.\n"
                "# dofs/frames are joint DEGREES per socket, in dof order. Managed by the app.\n")
        _yaml().safe_dump({"items": items}, f, sort_keys=False)


def load_build(name_or_path: str) -> Build:
    p = Path(name_or_path)
    if not p.exists():
        p = BUILDS_DIR / (name_or_path if name_or_path.endswith(".yaml") else f"{name_or_path}.yaml")
    m = _read(p)
    fingers = [
        FingerInstance(id=f["id"], type=f["type"], socket=f["socket"],
                       servos=list(f["servos"]))
        for f in m.get("fingers", [])
    ]
    hw = m.get("hardware", {})
    d = Hardware()                                  # single source for the defaults
    return Build(
        name=m["name"], mount=m["mount"], fingers=fingers, adapter=m.get("adapter"),
        hardware=Hardware(
            model=hw.get("model", d.model), ticks_per_rev=int(hw.get("ticks_per_rev", d.ticks_per_rev)),
            port=hw.get("port", d.port), baud=int(hw.get("baud", d.baud)),
            profile_velocity=int(hw.get("profile_velocity", d.profile_velocity)),
            profile_acceleration=int(hw.get("profile_acceleration", d.profile_acceleration)),
            current_limit=int(hw.get("current_limit", d.current_limit)),
            goal_current=int(hw.get("goal_current", d.goal_current)),
            hold_current=(int(hw["hold_current"]) if hw.get("hold_current") is not None
                          else d.hold_current),
            pos_p=int(hw.get("pos_p", d.pos_p)),
            pos_i=int(hw.get("pos_i", d.pos_i)),
            pos_d=int(hw.get("pos_d", d.pos_d)),
        ),
    )
