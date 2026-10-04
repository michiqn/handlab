#!/usr/bin/env python3
"""make_markers.py — print-ready ArUco marker sheets for the rig.

The rig markers are DICT_4X4_50 (measured edge lengths go in ~/.handlab/vision.yaml). This
tool renders chosen IDs at an EXACT edge length (mm) on a 300-dpi PNG with a white quiet zone, a
thin cut guide, and an "ID · mm · dict" caption. Print at 100% (no "fit to page"), then MEASURE the
black square — printers scale ~93-98%; the measured value is what goes in vision.yaml `markers:`.
Cut OUTSIDE the black edge (≥1 mm white quiet zone) or the marker won't be detected (13.07 lesson).

Examples:
  # index fingertip marker, the registry's ID 2 at 14.5 mm, 3 spare copies:
  python tools/make_markers.py --ids 2 --mm 14.5 --copies 3 --out docs/markers/index_4x4.png
  # a block of fresh IDs:
  python tools/make_markers.py --ids 5,6,7 --mm 12 --out sheet.png
"""
from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

DPI = 300
QUIET_MM = 2.0          # white border around the black square (≥1 mm required; 2 is safe)
CAPTION_MM = 7.0        # caption strip height under each marker (two lines: id·label / mm)
GAP_MM = 4.0            # gap between tiles


def mm2px(mm: float) -> int:
    return int(round(mm / 25.4 * DPI))


def marker_bitmap(dictionary, marker_id: int, side_px: int) -> np.ndarray:
    """A crisp side_px×side_px marker (rendered oversized on a 6-cell grid, then nearest-resized
    so the printed edge is exactly the requested mm with hard cell edges)."""
    cell = max(1, side_px // 6)                      # 4x4 data + 1-cell black border = 6 cells
    big = cv2.aruco.generateImageMarker(dictionary, marker_id, cell * 6)
    return cv2.resize(big, (side_px, side_px), interpolation=cv2.INTER_NEAREST)


def _font(px: int):
    for name in ("Menlo.ttc", "DejaVuSansMono.ttf", "Arial.ttf"):
        try:
            return ImageFont.truetype(name, px)
        except Exception:
            continue
    return ImageFont.load_default()


def build_sheet(ids: list[int], mm: float, dict_name: str, copies: int, cols: int,
                labels: list[str] | None = None) -> Image.Image:
    dict_id = getattr(cv2.aruco, dict_name)
    dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
    side = mm2px(mm)
    quiet = mm2px(QUIET_MM)
    cap = mm2px(CAPTION_MM)
    gap = mm2px(GAP_MM)
    tile_w = side + 2 * quiet
    tile_h = side + 2 * quiet + cap

    tiles: list[tuple[int, Image.Image]] = []
    for idx, mid in enumerate(ids):
        tag = f" · {labels[idx]}" if labels and idx < len(labels) and labels[idx] else ""
        for _ in range(copies):
            tile = Image.new("L", (tile_w, tile_h), 255)
            bmp = marker_bitmap(dictionary, mid, side)
            tile.paste(Image.fromarray(bmp), (quiet, quiet))
            d = ImageDraw.Draw(tile)
            # cut guide at the quiet-zone edge (light gray, printed but easy to trim off)
            d.rectangle([quiet - 1, quiet - 1, quiet + side, quiet + side], outline=200, width=1)
            font = _font(mm2px(2.1))
            d.text((quiet, side + 2 * quiet + mm2px(0.4)), f"ID {mid}{tag}", fill=70, font=font)
            d.text((quiet, side + 2 * quiet + mm2px(3.0)), f"{mm:g}mm", fill=120, font=font)
            tiles.append((mid, tile))

    n = len(tiles)
    cols = max(1, min(cols, n))
    rows = (n + cols - 1) // cols
    W = cols * tile_w + (cols + 1) * gap
    H = rows * tile_h + (rows + 1) * gap
    sheet = Image.new("L", (W, H), 255)
    for i, (_mid, tile) in enumerate(tiles):
        r, c = divmod(i, cols)
        sheet.paste(tile, (gap + c * (tile_w + gap), gap + r * (tile_h + gap)))
    return sheet


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ids", required=True, help="comma-separated marker ids, e.g. 2,5,6")
    ap.add_argument("--mm", type=float, required=True, help="edge length of the black square in mm")
    ap.add_argument("--dict", default="DICT_4X4_50", help="ArUco dictionary (default DICT_4X4_50)")
    ap.add_argument("--copies", type=int, default=1, help="copies of each id (spares for cutting)")
    ap.add_argument("--cols", type=int, default=4, help="tiles per row")
    ap.add_argument("--labels", default="", help="comma-separated caption per id, e.g. base,prox,mid,tip")
    ap.add_argument("--out", required=True, help="output PNG path")
    a = ap.parse_args()

    ids = [int(x) for x in a.ids.split(",") if x.strip() != ""]
    labels = [s.strip() for s in a.labels.split(",")] if a.labels else None
    sheet = build_sheet(ids, a.mm, a.dict, a.copies, a.cols, labels)
    out = Path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    sheet.save(out, dpi=(DPI, DPI))
    print(f"wrote {out}  ({sheet.width}x{sheet.height}px @ {DPI}dpi = "
          f"{sheet.width/DPI*25.4:.0f}x{sheet.height/DPI*25.4:.0f}mm) — print at 100%, then measure a marker.")


if __name__ == "__main__":
    main()
