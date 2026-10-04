"""Smoke tests for the modular core: load the claw build, derive the servo map.

Needs PyYAML. Run: pytest  (from the project root, with deps installed).
"""

import pytest

from handlab import library_io, compose


def test_load_example_build():
    b = library_io.load_build("claw3f")
    assert b.mount == "palm_mount"
    assert len(b.fingers) == 3
    assert b.motor_count == 9


def test_derive_servo_map_prefixes_actuators():
    b = library_io.load_build("claw3f")
    sm = compose.derive_servo_map(b)
    assert sm.servo_count == 9
    # every servo maps to a finger-prefixed actuator name (index_spread, thumb_flex, ...)
    assert {s.actuator for s in sm.servos} == {
        f"{fid}_{dof}" for fid in ("index", "mid", "thumb") for dof in ("spread", "flex", "curl")
    }
    assert {s.id for s in sm.servos} == set(range(1, 10))


def test_servo_count_mismatch_raises():
    b = library_io.load_build("claw3f")
    b.fingers[0].servos = [1, 2]   # 2 servos for 3 dofs
    with pytest.raises(ValueError):
        compose.derive_servo_map(b)


def test_library_listing():
    lib = library_io.list_library()
    assert {"claw_finger", "claw_mid", "claw_thumb"} <= set(lib["fingers"])
    assert "palm_mount" in lib["mounts"]
    assert "palm" in lib["adapters"]


def test_compose_with_adapter(tmp_path):
    """The claw build (palm adapter + 3 fingers on typed slots) composes: the adapter body
    sits at the base's center norm and each finger's root body nests inside its port."""
    import math
    import xml.etree.ElementTree as ET

    b = library_io.load_build("claw3f")
    assert b.adapter == "palm"
    c = compose.compose(b, out_dir=tmp_path)
    assert c.servo_map.servo_count == 9

    scene = ET.parse(c.scene_path).getroot()
    adapter = scene.find(".//body[@name='adapter']")
    assert adapter is not None
    assert adapter.find("geom[@name='adapter__geom']") is not None   # the palm renders
    for fid in ("index", "mid", "thumb"):                        # one nested mount per slot
        assert adapter.find(f"body[@name='{fid}_mount']") is not None
    # the mid finger (claw_mid) carries its own baked spread range on its own joint (radians);
    # joint names are the corrected re-export's (mid spread = index_mcp-spread, index = -23)
    j = scene.find(".//joint[@name='mid_palm_index_mcp-spread']")
    lo, hi = (float(x) for x in j.get("range").split())
    assert abs(lo - math.radians(-45)) < 1e-3 and abs(hi - math.radians(55)) < 1e-3
    j = scene.find(".//joint[@name='index_palm_Umdrehung-23']")
    lo, hi = (float(x) for x in j.get("range").split())
    assert abs(lo - (-0.139626)) < 1e-3 and abs(hi - 1.047198) < 1e-3
    # every finger link keeps its geom (jointed roots are never deduped)
    for fid, root in (("index", "mcp_yoke_index"), ("mid", "mcp_yoke_mid"),
                      ("thumb", "mcp_yoke_thumb")):
        assert scene.find(f".//geom[@name='{fid}_{root}_geom']") is not None


def test_effective_range_override_is_consistent(tmp_path):
    """A user 'edit limits' override (dof_ranges) hits ALL range sites at once — the joint
    range AND the actuator ctrlrange in the scene, matching effective_range_rad (what jogNorm
    and the slider read). Narrows thumb-flex (base [-5,+190]°) to [-2,180]°; home (0 rad) stays
    reachable. Regression: without an override the sites still carry the base range verbatim."""
    import math
    import xml.etree.ElementTree as ET
    from handlab.models.library import Dof

    b = library_io.load_build("claw3f")
    sm = compose.derive_servo_map(b)
    flex_id = next(s.id for s in sm.servos if s.actuator == "thumb_flex")     # thumb-flex servo
    jname = "thumb_mcp_yoke_thumb_thumb_mcp_flex"
    want = (math.radians(-2), math.radians(180))

    c = compose.compose(b, out_dir=tmp_path / "over", dof_ranges={flex_id: [-2, 180]})
    scene = ET.parse(c.scene_path).getroot()
    jlo, jhi = (float(x) for x in scene.find(f".//joint[@name='{jname}']").get("range").split())
    alo, ahi = (float(x) for x in
                scene.find(".//actuator/position[@name='thumb_flex']").get("ctrlrange").split())
    for got in ((jlo, jhi), (alo, ahi)):
        assert abs(got[0] - want[0]) < 1e-5 and abs(got[1] - want[1]) < 1e-5
    assert want[0] <= 0.0 <= want[1]                     # home (stretched rest) still in range

    # the shared helper (jogNorm + slider bounds read THIS) returns the same effective range
    d = Dof(key="flex", label="Flex", joint=jname, range=(-0.087266, 3.316126))
    elo, ehi = compose.effective_range_rad(d, None, [-2, 180])
    assert abs(elo - want[0]) < 1e-5 and abs(ehi - want[1]) < 1e-5

    # regression: no override → the actuator ctrlrange is the base range, byte-for-byte
    c0 = compose.compose(b, out_dir=tmp_path / "base")
    s0 = ET.parse(c0.scene_path).getroot()
    assert s0.find(".//actuator/position[@name='thumb_flex']").get("ctrlrange") == "-0.087266 3.316126"


def test_compose_adapter_mesh_when_set(tmp_path):
    """A mesh-bearing adapter DOES render its geom + copies the STL.
    Uses a throwaway adapter with a mesh dropped into the library."""
    import shutil, xml.etree.ElementTree as ET
    src = library_io.LIBRARY_DIR / "fingers" / "claw_finger" / "meshes" / "mcp_yoke_index.stl"
    d = library_io.LIBRARY_DIR / "adapters" / "_mesh_test"
    d.mkdir(parents=True, exist_ok=True)
    try:
        shutil.copyfile(src, d / "m.stl")
        (d / "adapter.yaml").write_text(
            "name: _mesh_test\nfits: center\nmesh: m.stl\nmesh_scale: 0.001\n"
            "ports:\n"
            "  - { id: index, accepts: finger, frame: { pos: [0,0,0], euler: [0,0,0] } }\n"
            "  - { id: thumb, accepts: thumb,  frame: { pos: [0,0,0], euler: [0,0,0] } }\n"
            "  - { id: mid,   accepts: finger, frame: { pos: [0,0,0], euler: [0,0,0] } }\n")
        b = library_io.load_build("claw3f"); b.adapter = "_mesh_test"
        c = compose.compose(b, out_dir=tmp_path)
        scene = ET.parse(c.scene_path).getroot()
        assert scene.find(".//body[@name='adapter']/geom[@name='adapter__geom']") is not None
        assert (c.scene_path.parent / "meshes" / "m.stl").exists()
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_physics_compose(tmp_path):
    """Physics mode: gravity on, a falling test object, finger force caps, and the
    composed scene still loads in MuJoCo."""
    import xml.etree.ElementTree as ET
    b = library_io.load_build("claw3f")
    c = compose.compose(b, out_dir=tmp_path, physics=True)
    scene = ET.parse(c.scene_path).getroot()
    assert scene.find("option").get("gravity") == "0 0 -9.81"
    assert scene.find(".//body[@name='playobj']/freejoint[@name='playobj_free']") is not None
    assert scene.find(".//geom[@name='playfloor']") is not None
    assert all(a.get("forcerange") for a in scene.findall(".//actuator/position"))
    # gravcomp keeps the hand from sagging
    assert scene.find(".//body[@name='adapter']").get("gravcomp") == "1"
    import mujoco
    mj = mujoco.MjModel.from_xml_path(str(c.scene_path))
    assert mj.njnt > 12 and mj.nu == 9           # 12 hinges + the object's free joint


def test_palm_adapter_typed_slots():
    """The palm adapter's ports carry slot types (index/mid = finger, thumb = thumb),
    ordered index, thumb, mid (thumb = P2 in the hub)."""
    at = library_io.load_adapter_type("palm")
    assert [(p.id, p.accepts) for p in at.ports] == [
        ("index", "finger"), ("thumb", "thumb"), ("mid", "finger")]
    assert at.mesh == "palm_adapter.stl"  # the palm renders its own link mesh
    assert not at.ports[2].dof_ranges_deg   # mid finger carries its own baked range, no override
    mt = library_io.load_mount_type("palm_mount")
    assert [s.kind for s in mt.sockets] == ["center"]
    assert mt.model == "palm_base.stl"    # Sockel link mesh, placed via model_frame
    assert abs(mt.model_euler[1] - 0.785398163) < 1e-6
