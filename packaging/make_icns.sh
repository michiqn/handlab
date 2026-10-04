#!/bin/zsh
# Build packaging/handlab.icns from the app logo (handlab/ui/assets/logo_mark.png).
# macOS-only (uses sips + iconutil). Re-run whenever the logo changes.
set -e
cd "$(dirname "$0")/.."

SRC="handlab/ui/assets/logo_mark.png"
SET="packaging/handlab.iconset"
OUT="packaging/handlab.icns"

rm -rf "$SET" && mkdir -p "$SET"
for s in 16 32 64 128 256 512; do
  sips -z $s $s             "$SRC" --out "$SET/icon_${s}x${s}.png"      >/dev/null
  sips -z $((s*2)) $((s*2)) "$SRC" --out "$SET/icon_${s}x${s}@2x.png"   >/dev/null
done
iconutil -c icns "$SET" -o "$OUT"
rm -rf "$SET"
echo "wrote $OUT"
