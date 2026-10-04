"""tools/import_assembly.py: a clean ACDC hand export -> library parts, VERBATIM.

Runs against the corrected export (tests/fixtures/sim_assembly_v67.xml — all yokes
native, directions fixed in Fusion): the thumb bundle and the adapter port frames come out
bit-for-bit like the shipped library, the emitted bundle passes a real import, and --check
confirms every port's spread pin lands on the adapter mesh (the mirror-bug guard).
"""
import subprocess
import sys
from pathlib import Path

import pytest

from handlab import library_io

REPO = Path(__file__).resolve().parent.parent
FIXTURE_XML = REPO / "tests" / "fixtures" / "sim_assembly_v67.xml"   # the corrected ACDC export
LIB = REPO / "handlab" / "library"

# The export's meshes are byte-identical to the library copies (the library WAS built from this
# export), so the test re-assembles <assembly_dir>/meshes/ from the library instead of shipping
# the same STLs twice.
EXPORT_MESHES = {
    "palm_base": "mounts/palm_mount/palm_base.stl",
    "palm": "adapters/palm4/palm_adapter.stl",
    **{n: f"fingers/claw_thumb/meshes/{n}.stl"
       for n in ("mcp_yoke_thumb", "thumb_proximal", "thumb_mid", "thumb_distal")},
    **{n: f"fingers/claw_ring/meshes/{n}.stl"
       for n in ("mcp_yoke_index", "index_proximal", "index_mid", "index_distal")},
    **{n: f"fingers/claw_mid/meshes/{n}.stl"
       for n in ("mcp_yoke_mid", "mid_proximal", "mid_mid", "mid_distal")},
}


@pytest.fixture
def ASM(tmp_path_factory):
    d = tmp_path_factory.mktemp("sim_assembly_v67")
    (d / "sim_assembly_v67.xml").write_bytes(FIXTURE_XML.read_bytes())
    (d / "meshes").mkdir()
    for name, rel in EXPORT_MESHES.items():
        (d / "meshes" / f"{name}.stl").symlink_to(LIB / rel)
    return d


def _run(asm, out, *extra):
    subprocess.run([sys.executable, str(REPO / "tools" / "import_assembly.py"),
                    str(asm), "--out", str(out), *extra], check=True,
                   capture_output=True, text=True)


def test_thumb_is_verbatim(ASM, tmp_path):
    """Importing the shipped export reproduces the library's thumb bundle + palm thumb port
    bit-for-bit (the library WAS built from this export)."""
    _run(ASM, tmp_path)
    # emitted thumb finger type parses; 3 semantic dofs; curl coupled to dip with ratio +1
    # (corrected export: the dip axis is PARALLEL to curl — the old export had it anti-parallel)
    ft = library_io.load_finger_type("palm_thumb", library_dir=tmp_path)
    assert [d.key for d in ft.dofs] == ["spread", "flex", "curl"]
    assert ft.slot == "thumb"
    assert len(ft.couplings) == 1
    assert ft.couplings[0].ratio == 1.0
    assert ft.couplings[0].follower == "thumb_mid_Umdrehung-12"
    # thumb-flex runs positive-dominant (direction fixed at the CAD source)
    flex = next(d for d in ft.dofs if d.key == "flex")
    assert flex.range[0] == pytest.approx(-0.087266, abs=1e-6)
    assert flex.range[1] == pytest.approx(3.316126, abs=1e-6)

    # emitted thumb PORT frame == the shipped, verified palm thumb port (verbatim)
    at = library_io.load_adapter_type("palm", library_dir=tmp_path)
    thumb = next(p for p in at.ports if p.id == "thumb")
    shipped = next(p for p in library_io.load_adapter_type("palm").ports if p.id == "thumb")
    assert thumb.pos == pytest.approx(shipped.pos, abs=1e-6)
    assert thumb.euler == pytest.approx(shipped.euler, abs=1e-6)

    # emitted thumb model.xml == the shipped library model.xml (whitespace-normalized)
    def norm(p):
        return " ".join(p.read_text().split())
    assert norm(tmp_path / "fingers" / "palm_thumb" / "model.xml") == \
           norm(library_io.LIBRARY_DIR / "fingers" / "claw_thumb" / "model.xml")


def test_check_confirms_ports_clean(ASM, tmp_path):
    """--check: every port's spread pin lands ON the adapter mesh (mirror-bug guard) —
    the corrected export is clean, so all three ports must be OK and none BROKEN."""
    r = subprocess.run([sys.executable, str(REPO / "tools" / "import_assembly.py"),
                        str(ASM), "--out", str(tmp_path), "--check"],
                       check=True, capture_output=True, text=True)
    out = r.stdout
    assert "BROKEN" not in out
    for fam in ("thumb", "index", "mid"):
        line = next(l for l in out.splitlines()
                    if l.strip().startswith(fam) and "pin->adapter" in l)
        assert "OK verbatim" in line
