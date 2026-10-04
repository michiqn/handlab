"""compose.py — turn a Build + library into (1) a ServoMap and (2) a composed MuJoCo scene.

The composer assembles N fingers into one scene: each finger's bodies/joints/geoms/meshes are
PREFIXED with the finger id ("thumb_", "index_"…) to avoid collisions, the finger's root body is
placed under a body at its mount socket frame, and one position actuator per DOF + the couplings
+ the learned tuning are added. Fingers whose type isn't in the library are skipped in the sim
(they stay in the build/servo-map for ordinal alignment with the UI).
"""

from __future__ import annotations

import math
import shutil
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

from .models import Build, FingerType, Servo, ServoMap
from . import library_io

COMPOSED_DIR = Path(__file__).resolve().parent / "sim" / "composed"

# Sim-playground (physics/grasp mode): where the test object spawns + the floor height.
PLAY_SPAWN = (0.0, 0.0, 0.34)
PLAY_FLOOR_Z = -0.25


@dataclass
class Composed:
    servo_map: ServoMap
    scene_path: Path | None


def _try_load(type_name: str) -> FingerType | None:
    try:
        return library_io.load_finger_type(type_name)
    except Exception:
        return None


def _copy_mesh(src, dst) -> None:
    """Copy a mesh into the composed dir, skipping when an identical-size copy is already there.
    Library meshes are static, so this avoids re-copying every STL on each recompose (edit-limits /
    physics toggle reuse the same composed dir)."""
    src, dst = Path(src), Path(dst)
    if dst.exists() and dst.stat().st_size == src.stat().st_size:
        return
    shutil.copyfile(src, dst)


def derive_servo_map(build: Build) -> ServoMap:
    """Flatten the build into the servo list (skips fingers whose type isn't in the library)."""
    servos: list[Servo] = []
    for f in build.fingers:
        ft = _try_load(f.type)
        if ft is None:
            continue
        if len(f.servos) != len(ft.dofs):
            raise ValueError(f"finger {f.id!r}: {len(f.servos)} servos for {len(ft.dofs)} dofs")
        for sid, dof in zip(f.servos, ft.dofs):
            servos.append(Servo(id=sid, finger=f.id, dof=dof.key, actuator=f"{f.id}_{dof.key}",
                                joint=f"{f.id}_{dof.joint}"))
    return ServoMap(servos=servos)


def compose(build: Build, out_dir: Path | None = None, physics: bool = False,
            dof_ranges: dict[int, list] | None = None) -> Composed:
    """dof_ranges: {servo_id: [min_deg, max_deg]} — user "edit limits" override for the effective
    slider/joint/actuator range. None → the base/adapter range; keeps home (qpos 0) reachable."""
    servo_map = derive_servo_map(build)
    scene_path = _compose_scene(build, out_dir, physics, dof_ranges)
    return Composed(servo_map=servo_map, scene_path=scene_path)


def effective_range_rad(dof, port_over: dict | None, user_over_deg) -> tuple[float, float]:
    """Effective (lo, hi) RADIANS for a dof: user override (deg) > adapter port override (deg) >
    base range. The single source used by the joint range, the actuator ctrlrange and jogNorm,
    so the slider, the twin and the commanded target all agree."""
    if user_over_deg:
        return (math.radians(user_over_deg[0]), math.radians(user_over_deg[1]))
    if port_over and dof.key in port_over:
        p = port_over[dof.key]
        return (math.radians(p[0]), math.radians(p[1]))
    return (dof.range[0], dof.range[1])


def _prefix_subtree(el: ET.Element, prefix: str) -> None:
    """Prefix every name (and geom mesh-reference) in a body subtree."""
    for e in el.iter():
        if "name" in e.attrib:
            e.set("name", prefix + e.attrib["name"])
        if e.tag == "geom" and "mesh" in e.attrib:
            e.set("mesh", prefix + e.attrib["mesh"])


def _compose_scene(build: Build, out_dir: Path | None, physics: bool = False,
                   dof_ranges: dict[int, list] | None = None) -> Path | None:
    out = (out_dir or COMPOSED_DIR) / build.name
    (out / "meshes").mkdir(parents=True, exist_ok=True)

    mount = None
    try:
        mount = library_io.load_mount_type(build.mount)
    except Exception:
        pass
    frames = {s.id: (s.pos, s.euler) for s in (mount.sockets if mount else [])}

    scene = ET.Element("mujoco", model=build.name)
    ET.SubElement(scene, "compiler", angle="radian", meshdir="meshes")
    # physics/grasp mode: gravity on (only the test object falls — fingers get gravcomp).
    ET.SubElement(scene, "option", integrator="implicitfast",
                  gravity="0 0 -9.81" if physics else "0 0 0")
    vis = ET.SubElement(scene, "visual")
    ET.SubElement(vis, "global", offwidth="1200", offheight="1200")
    ET.SubElement(vis, "quality", shadowsize="2048")
    default = ET.SubElement(scene, "default")
    ET.SubElement(default, "joint", damping="0.005", armature="1e-6")
    # off = no collisions (fast). physics = fingers collide with object/floor (contype 2,
    # conaffinity 1) but NOT each other (adjacent links would explode).
    if physics:
        ET.SubElement(default, "geom", contype="2", conaffinity="1")
    else:
        ET.SubElement(default, "geom", contype="0", conaffinity="0")
    asset = ET.SubElement(scene, "asset")
    world = ET.SubElement(scene, "worldbody")
    ET.SubElement(world, "light", directional="true", pos="-0.5 0.5 3", dir="0 0 -1")
    actuator = ET.SubElement(scene, "actuator")
    equality = ET.SubElement(scene, "equality")

    # Mount geometry: if the mount declares a model (STL in the mount's dir, exported in
    # the same world frame as the sockets), render it as a static world geom.
    if mount is not None and mount.model:
        mfn = Path(mount.model).name
        _copy_mesh(Path(mount.dir) / mount.model, out / "meshes" / mfn)
        ET.SubElement(asset, "mesh", name="mount__mesh", file=mfn, scale="0.001 0.001 0.001")
        mp, me = mount.model_pos, mount.model_euler
        ET.SubElement(world, "geom", name="mount__geom", type="mesh", mesh="mount__mesh",
                      pos=f"{mp[0]} {mp[1]} {mp[2]}", euler=f"{me[0]} {me[1]} {me[2]}")

    # Center adapter: a static coupler body placed at the mount's center norm. Fingers whose
    # socket is one of its port ids nest inside it at the port frame (MuJoCo composes the
    # transforms). The adapter mesh renders as a plain geom — geometry only, no joints.
    adapter = None
    if build.adapter:
        try:
            adapter = library_io.load_adapter_type(build.adapter)
        except Exception:
            pass
    port_frames = {p.id: (p.pos, p.euler) for p in (adapter.ports if adapter else [])}
    port_ranges = {p.id: p.dof_ranges_deg for p in (adapter.ports if adapter else [])}
    adapter_body = None
    if adapter is not None:
        cpos, ceuler = frames.get(adapter.fits, ((0, 0, 0), (0, 0, 0)))
        adapter_body = ET.SubElement(world, "body", name="adapter",
                                     pos=f"{cpos[0]} {cpos[1]} {cpos[2]}",
                                     euler=f"{ceuler[0]} {ceuler[1]} {ceuler[2]}")
        if adapter.mesh:
            fn = Path(adapter.mesh).name
            _copy_mesh(Path(adapter.dir) / adapter.mesh, out / "meshes" / fn)
            s = adapter.mesh_scale
            ET.SubElement(asset, "mesh", name="adapter__mesh", file=fn, scale=f"{s} {s} {s}")
            mp, me = adapter.mesh_pos, adapter.mesh_euler
            ET.SubElement(adapter_body, "geom", name="adapter__geom", type="mesh",
                          mesh="adapter__mesh",
                          pos=f"{mp[0]} {mp[1]} {mp[2]}", euler=f"{me[0]} {me[1]} {me[2]}")

    # Some finger bundles carry a STATIC base body as their root (a shared coupler/plate).
    # Render that base geom only ONCE (prefer the center/adapter finger) so N fingers
    # don't stack N copies. Jointed roots (claw mcp_yoke) are real links — never deduped.
    kinds = {s.id: s.kind for s in (mount.sockets if mount else [])}
    known = [f for f in build.fingers if _try_load(f.type) is not None]
    mount_fid = next((f.id for f in known
                      if kinds.get(f.socket) == "center" or f.socket in port_frames), None)
    if mount_fid is None and known:
        mount_fid = known[0].id

    copied: set[str] = set()
    if adapter is not None and adapter.mesh:
        copied.add(Path(adapter.mesh).name)        # don't let a finger mesh overwrite it
    if mount is not None and mount.model:
        copied.add(Path(mount.model).name)
    for f in build.fingers:
        ft = _try_load(f.type)
        if ft is None:
            continue
        prefix = f"{f.id}_"
        model = ET.parse(Path(ft.dir) / ft.model).getroot()

        for mesh in model.findall("./asset/mesh"):
            fn = Path(mesh.get("file")).name
            if fn not in copied:
                _copy_mesh(Path(ft.dir) / mesh.get("file"), out / "meshes" / fn)
                copied.add(fn)
            ET.SubElement(asset, "mesh", name=prefix + mesh.get("name"),
                          file=fn, scale=mesh.get("scale", "1 1 1"))

        root_body = model.find("./worldbody/body")
        if root_body is None:
            continue
        _prefix_subtree(root_body, prefix)
        # Drop duplicate base geom on the other fingers — but only for a STATIC root.
        # A jointed root is a real moving link and keeps its geom on every finger.
        if f.id != mount_fid and root_body.find("joint") is None:
            for g in list(root_body.findall("geom")):
                root_body.remove(g)
        if f.socket in port_frames and adapter_body is not None:
            pos, euler = port_frames[f.socket]
            parent = adapter_body                  # nested: adapter frame ∘ port frame
        else:
            pos, euler = frames.get(f.socket, ((0, 0, 0), (0, 0, 0)))
            parent = world
        # effective per-dof range = user override (Control tab) > adapter port override > base.
        # per-slot port overrides (e.g. a mid slot mirroring the index spread travel):
        overrides = port_ranges.get(f.socket, {})
        # user "edit limits" (range_deg) keyed by servo id:
        user_ranges = {d.key: dof_ranges[sid] for sid, d in zip(f.servos, ft.dofs)
                       if (dof_ranges or {}).get(sid)} if dof_ranges else {}
        eff = {d.key: effective_range_rad(d, overrides, user_ranges.get(d.key)) for d in ft.dofs}
        for d in ft.dofs:
            if not (d.key in overrides or d.key in user_ranges):
                continue
            jel = root_body.find(f".//joint[@name='{prefix + d.joint}']")
            if jel is None:
                continue
            lo, hi = eff[d.key]                                 # effective range on the joint
            jel.set("range", f"{lo:.6f} {hi:.6f}")
        place = ET.SubElement(parent, "body", name=f"{prefix}mount",
                              pos=f"{pos[0]} {pos[1]} {pos[2]}",
                              euler=f"{euler[0]} {euler[1]} {euler[2]}")
        place.append(root_body)

        tn = ft.tuning
        # in physics mode the servo needs stiffness + a force cap to actually grip
        kp = 10.0 if physics else tn.get("kp", 1.0)
        for d in ft.dofs:
            lo, hi = eff[d.key]                          # same effective range as the joint (consistency)
            act = ET.SubElement(actuator, "position", name=f"{f.id}_{d.key}", joint=prefix + d.joint,
                                kp=str(kp), kv=str(tn.get("kv", 0.03)),
                                ctrlrange=f"{lo} {hi}")
            if physics:
                act.set("forcerange", "-3 3")        # limit grip force (tunable)
        for c in ft.couplings:
            # MuJoCo joint-equality constrains joint1 as a polynomial of joint2
            # (p_joint1 = a0 + a1*p_joint2 + ...), i.e. joint2 is the INDEPENDENT variable.
            # So the actuated PIP (driver) must be joint2 and the passive DIP (follower) joint1
            # -> follower = ratio*driver. At ratio 1.0 it's symmetric; this makes ratio != 1 mean
            # what the YAML says (a DIP that bends less than the PIP) instead of its reciprocal.
            ET.SubElement(equality, "joint", joint1=prefix + c.follower, joint2=prefix + c.driver,
                          polycoef=f"0 {c.ratio} 0 0 0")

    if physics:
        # gravity-compensate every hand body so the fingers HOLD; only the object falls
        for b in world.iter("body"):
            b.set("gravcomp", "1")
        ET.SubElement(world, "geom", name="playfloor", type="plane",
                      pos=f"0 0 {PLAY_FLOOR_Z}", size="2 2 0.1",
                      contype="1", conaffinity="3", rgba="0.16 0.18 0.22 1")
        obj = ET.SubElement(world, "body", name="playobj",
                            pos=f"{PLAY_SPAWN[0]} {PLAY_SPAWN[1]} {PLAY_SPAWN[2]}")
        ET.SubElement(obj, "freejoint", name="playobj_free")
        ET.SubElement(obj, "geom", name="playobj_geom", type="sphere", size="0.018",
                      mass="0.02", contype="1", conaffinity="3", rgba="0.92 0.55 0.30 1")

    scene_path = out / "scene.xml"
    ET.ElementTree(scene).write(scene_path, encoding="unicode")
    return scene_path
