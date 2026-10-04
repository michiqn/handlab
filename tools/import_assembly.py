"""Turn a CLEAN ACDC4Robot hand-assembly export into handlab library parts — VERBATIM.

This is the registration-FREE path. A correctly authored Fusion export (fingers NOT
mirrored, assembly straightened to joint-zero) carries every mounting frame exactly; this
tool just re-expresses them as library parts. No STL registration, no ground truth.

  base body  (worldbody child, no joint, no jointed descendants) -> mount.yaml
  adapter body (worldbody child, no joint, carries the finger chains) -> adapter.yaml
  each yoke->proximal->mid->distal chain -> a finger bundle (root normalized to identity,
      inner rels + joints + ranges verbatim; curl/dip auto-coupled by axis sign)

The ONLY thing that defeats this is the ACDC4Robot mirror bug: mirrored/patterned
occurrences get corrupted palm->yoke rels, so their ports land wrong (see the --check
report). Fix by exporting those fingers as independent copies ("Make Independent") and
re-export — the direction/frames live in Fusion, not in software.

Usage:
  ~/.venvs/handlab/bin/python tools/import_assembly.py <assembly_dir> [--out DIR]
      [--reuse mid=index] [--check]

<assembly_dir> holds the ACDC .xml + a meshes/ dir. Emits a staging library under --out
(default: <assembly_dir>/_library) with mounts/, adapters/, fingers/ — review, then copy
into handlab/library/. --check adds a self-consistency report (pin-on-part per port).
"""
from __future__ import annotations

import argparse
import math
import shutil
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent


# ---- small geometry helpers (formerly tools/extract_claw.py) ----
def vec(s):
    return np.array([float(x) for x in s.split()])


def euler_to_R(e):
    cx, cy, cz = np.cos(e)
    sx, sy, sz = np.sin(e)
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return Rx @ Ry @ Rz    # MuJoCo 'xyz' euler = INTRINSIC x-y-z (verified empirically)


def R_to_euler(R):
    """inverse of euler_to_R: intrinsic xyz. R = Rx(a) @ Ry(b) @ Rz(c)."""
    cb = math.sqrt(R[0, 0] ** 2 + R[0, 1] ** 2)
    b = math.atan2(R[0, 2], cb)
    if cb > 1e-9:
        return [math.atan2(-R[1, 2], R[2, 2]), b, math.atan2(-R[0, 1], R[0, 0])]
    return [math.atan2(R[2, 1], R[1, 1]), b, 0.0]


def read_stl(p: Path) -> np.ndarray:
    """Binary STL -> (n, 3, 3) float64 triangle vertices."""
    b = p.read_bytes()
    n = struct.unpack_from("<I", b, 80)[0]
    a = np.frombuffer(b, dtype=np.uint8, count=n * 50, offset=84)
    a = a.reshape(n, 50)[:, :48].copy().view("<f4").reshape(n, 12)
    return a[:, 3:12].reshape(n, 3, 3).astype(np.float64)


def bodies_of(parent):
    return parent.findall("body")


def is_static(b):
    return b.find("joint") is None


def chain_of(yoke):
    """the linear body chain from a yoke down (each body has exactly one child body)."""
    chain, b = [], yoke
    while b is not None:
        chain.append(b)
        kids = bodies_of(b)
        b = kids[0] if len(kids) == 1 else None
    return chain


def fam_of(name):
    """the finger family = the trailing token of the yoke name (mcp_yoke_INDEX -> index)."""
    return name.rsplit("_", 1)[-1]


def dof_key(joint_name):
    return joint_name.rsplit("_", 1)[-1]      # ..._spread -> spread


def fmt3(v):
    return " ".join(f"{x:.9f}" for x in v)


def yaml_frame(pos, euler):
    return (f"{{ pos: [{pos[0]:.6f}, {pos[1]:.6f}, {pos[2]:.6f}],\n"
            f"             euler: [{euler[0]:.9f}, {euler[1]:.9f}, {euler[2]:.9f}] }}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("assembly_dir")
    ap.add_argument("--out", default=None)
    ap.add_argument("--reuse", action="append", default=[],
                    help="portFam=typeFam, e.g. mid=index (that port reuses index's bundle)")
    ap.add_argument("--check", action="store_true")
    a = ap.parse_args()

    src = Path(a.assembly_dir)
    xml = next(src.glob("*.xml"))
    meshdir = src / "meshes"
    out = Path(a.out) if a.out else src / "_library"
    reuse = dict(kv.split("=") for kv in a.reuse)      # portFam -> typeFam

    root = ET.parse(xml).getroot()
    world = root.find("worldbody")
    tops = bodies_of(world)
    statics = [b for b in tops if is_static(b)]
    # adapter = the static top body that carries jointed finger chains; base = the other
    adapter = next(b for b in statics if any(not is_static(c) for c in bodies_of(b)))
    base = next((b for b in statics if b is not adapter), None)

    def world_pose(b, R=np.eye(3), t=np.zeros(3)):
        Rb = euler_to_R(vec(b.get("euler", "0 0 0")))
        tb = vec(b.get("pos", "0 0 0"))
        return R @ Rb, R @ tb + t

    Ra, ta = world_pose(adapter)
    yokes = [c for c in bodies_of(adapter) if not is_static(c)]
    fams = [fam_of(y.get("name")) for y in yokes]
    print(f"base={base.get('name') if base is not None else '-'}  "
          f"adapter={adapter.get('name')}  fingers={fams}")

    (out / "mounts").mkdir(parents=True, exist_ok=True)
    (out / "adapters").mkdir(parents=True, exist_ok=True)
    (out / "fingers").mkdir(parents=True, exist_ok=True)

    def copy_mesh(name, dest):
        (dest / "meshes").mkdir(parents=True, exist_ok=True)
        shutil.copyfile(meshdir / f"{name}.stl", dest / "meshes" / f"{name}.stl")

    # ---- mount.yaml (base) ----
    mount_name = f"{base.get('name')}_mount" if base is not None else "mount"
    md = out / "mounts" / mount_name
    md.mkdir(parents=True, exist_ok=True)
    lines = [f"name: {mount_name}", 'label: "Base"']
    if base is not None:
        bmesh = base.find("geom").get("mesh")
        copy_mesh(bmesh, md)
        Rb, tb = euler_to_R(vec(base.get("euler", "0 0 0"))), vec(base.get("pos", "0 0 0"))
        lines += [f"model: {bmesh}.stl",
                  "model_frame:",
                  f"  pos: [{tb[0]:.6f}, {tb[1]:.6f}, {tb[2]:.6f}]",
                  f"  euler: [{R_to_euler(Rb)[0]:.9f}, {R_to_euler(Rb)[1]:.9f}, {R_to_euler(Rb)[2]:.9f}]"]
    lines += ["sockets:", "  - id: center", "    kind: center", "    frame:",
              f"      pos: [{ta[0]:.6f}, {ta[1]:.6f}, {ta[2]:.6f}]",
              f"      euler: [{R_to_euler(Ra)[0]:.9f}, {R_to_euler(Ra)[1]:.9f}, {R_to_euler(Ra)[2]:.9f}]"]
    (md / "mount.yaml").write_text("\n".join(lines) + "\n")

    # ---- adapter.yaml (ports = each yoke pose RELATIVE to the adapter, verbatim) ----
    adapter_name = adapter.get("name")
    ad = out / "adapters" / adapter_name
    ad.mkdir(parents=True, exist_ok=True)
    ameshes = adapter.findall("geom")
    port_blocks = []
    for y, fam in zip(yokes, fams):
        Ry, ty = world_pose(y, Ra, ta)          # yoke world
        pos = Ra.T @ (ty - ta)
        euler = R_to_euler(Ra.T @ Ry)
        accepts = "thumb" if "thumb" in fam else "finger"
        port_blocks.append(
            f"  - id: {fam}\n    label: \"{fam.capitalize()} slot\"\n"
            f"    accepts: {accepts}\n    frame: {yaml_frame(pos, euler)}")
    al = [f"name: {adapter_name}", 'label: "Adapter"', "fits: center"]
    if ameshes:
        amesh = ameshes[0].get("mesh")
        copy_mesh(amesh, ad)
        al += [f"mesh: {amesh}.stl", "mesh_scale: 0.001",
               "mesh_frame: { pos: [0.0, 0.0, 0.0], euler: [0.0, 0.0, 0.0] }"]
    al += ["ports:"] + port_blocks
    (ad / "adapter.yaml").write_text("\n".join(al) + "\n")

    # ---- finger bundles ----
    def emit_finger(yoke, fam):
        chain = chain_of(yoke)
        dest = out / "fingers" / f"{adapter_name}_{fam}"
        dest.mkdir(parents=True, exist_ok=True)
        mj = ET.Element("mujoco", model=f"{fam}_finger")
        ET.SubElement(mj, "compiler", angle="radian", autolimits="true")
        asset = ET.SubElement(mj, "asset")
        wb = ET.SubElement(mj, "worldbody")
        parent = wb
        joints = []
        for i, b in enumerate(chain):
            name = b.get("name")
            ET.SubElement(asset, "mesh", name=name, file=f"meshes/{name}.stl",
                          scale="0.001 0.001 0.001")
            copy_mesh(name, dest)
            attrs = {"name": name}
            if i == 0:
                attrs.update(pos="0 0 0", euler="0 0 0")       # root normalized to identity
            else:
                attrs.update(pos=b.get("pos", "0 0 0"), euler=b.get("euler", "0 0 0"))
            body = ET.SubElement(parent, "body", **attrs)
            j = b.find("joint")
            if j is not None:
                ET.SubElement(body, "joint", name=j.get("name"), type="hinge",
                              axis=j.get("axis"), pos=j.get("pos"), range=j.get("range"))
                joints.append(j)
            ET.SubElement(body, "geom", name=f"{name}_geom", type="mesh", mesh=name)
            ine = b.find("inertial")
            if ine is not None:
                import copy as _c
                body.append(_c.deepcopy(ine))
            parent = body
        ET.indent(mj)
        ET.ElementTree(mj).write(dest / "model.xml", encoding="unicode")

        # type.yaml: joints[0..2] = driven dofs, joints[3] = curl-coupled follower (if the
        # last two joint axes are ~parallel — the tendon curl/dip convention).
        def ax(j):
            return vec(j.get("axis"))
        rad = lambda s: [float(x) for x in s.split()]
        dofs, couplings = [], []
        drive = joints[:3]
        # Tendon-hand DOF convention (chain order): yoke=spread, proximal=flex, mid=curl,
        # distal=dip (the curl-coupled follower, not a driven dof). Fusion's default joint
        # names ("Umdrehung-N") carry no semantics, so assign the driven keys by position.
        SEMANTIC = ["spread", "flex", "curl"]
        for k, j in enumerate(drive):
            key = SEMANTIC[k] if k < len(SEMANTIC) else dof_key(j.get("name"))
            lo, hi = rad(j.get("range"))
            coupled = (len(joints) == 4 and k == 2)
            dofs.append((key, j.get("name"), lo, hi, coupled))
        if len(joints) == 4:
            curl, dip = joints[2], joints[3]
            ratio = 1.0 if float(np.dot(ax(curl), ax(dip))) >= 0 else -1.0
            couplings.append((curl.get("name"), dip.get("name"), ratio))
        yl = [f"# Auto-generated by tools/import_assembly.py from {xml.name} (VERBATIM).",
              f"name: {adapter_name}_{fam}", f'label: "{fam.capitalize()} finger"',
              f"motors: {len(dofs)}",
              f"slot: {'thumb' if 'thumb' in fam else 'finger'}", "", "dofs:"]
        for key, jn, lo, hi, coupled in dofs:
            c = ", coupled: true" if coupled else ""
            yl.append(f'  - {{ key: {key}, label: "{key.capitalize()}", joint: "{jn}", '
                      f"range: [{lo:.6f}, {hi:.6f}]{c} }}")
        if couplings:
            yl += ["", "couplings:"]
            for drv, fol, ratio in couplings:
                yl.append(f'  - {{ driver: "{drv}", follower: "{fol}", ratio: {ratio} }}')
        yl += ["", "model: model.xml",
               "", "tuning: { damping: 0.005, armature: 1.0e-6, kp: 1.0, kv: 0.03 }",
               "", "ui:", f"  id: {adapter_name}_{fam}", '  mech: "tendon finger"',
               f'  blurb: "{fam} finger \\u00b7 {len(dofs)} dof"', "  dofs:"]
        for key, jn, lo, hi, coupled in dofs:
            yl.append(f'    {key}: {{ id: {key}, group: {key}, unit: "\\u00b0", '
                      f"min: {round(math.degrees(lo))}, max: {round(math.degrees(hi))}, home: 0 }}")
        (dest / "type.yaml").write_text("\n".join(yl) + "\n")
        return chain

    emitted = {}
    for y, fam in zip(yokes, fams):
        if fam in reuse:            # this port reuses another family's bundle
            continue
        emitted[fam] = emit_finger(y, fam)
    print(f"emitted mount={mount_name}, adapter={adapter_name}, "
          f"fingers={sorted(emitted)} (reuse: {reuse or 'none'})")
    print(f"-> staging library at {out}")

    if a.check:
        # The spread pin is the hinge shared by the adapter and the yoke. Mapped through the
        # palm->yoke rel into ADAPTER coords it must land ON the adapter mesh. A mirror-bug
        # rel throws it far off — this is the joint-angle-invariant that flags broken ports.
        print("\n== port self-consistency (spread pin in adapter frame vs adapter mesh) ==")
        av = read_stl(meshdir / f"{ameshes[0].get('mesh')}.stl").reshape(-1, 3) if ameshes else None
        for y, fam in zip(yokes, fams):
            Ry_rel = euler_to_R(vec(y.get("euler", "0 0 0")))
            ty_rel = vec(y.get("pos", "0 0 0")) * 1000.0
            pin = vec(y.find("joint").get("pos")) * 1000.0
            q = Ry_rel @ pin + ty_rel               # pin in adapter coords (mm)
            d = float(np.linalg.norm(av - q, axis=1).min()) if av is not None else 0.0
            ok = d < 8
            print(f"  {fam:6s} pin->adapter mesh: {d:7.1f} mm  "
                  + ("OK verbatim" if ok else "BROKEN (mirror bug) -> re-export un-mirrored from Fusion"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
