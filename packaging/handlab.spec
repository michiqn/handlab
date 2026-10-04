# -*- mode: python ; coding: utf-8 -*-
# PyInstaller spec — builds the double-clickable handlab.app (macOS).
#
#   ~/.venvs/handlab/bin/pyinstaller packaging/handlab.spec --noconfirm
#
# Output: dist/handlab.app. The package data (QML UI, parts library, builds) is bundled
# so the frozen app keeps the same handlab/<...> layout app.py resolves at runtime. The build is
# fixed from the bundled builds/<name>.yaml; there is no in-app assembly editor.

from pathlib import Path

ROOT = Path(SPECPATH).parent          # project root (packaging/..)
PKG = ROOT / "handlab"

a = Analysis(
    [str(ROOT / "packaging" / "launch.py")],
    pathex=[str(ROOT)],
    binaries=[],
    datas=[
        (str(PKG / "ui" / "qml"), "handlab/ui/qml"),
        (str(PKG / "ui" / "assets"), "handlab/ui/assets"),
        (str(PKG / "library"), "handlab/library"),
        # build DEFINITIONS only — builds/ also holds per-machine user state (calibration.yaml,
        # *.poses.yaml, both gitignored) that must not ship inside the .app
        (str(PKG / "builds" / "claw3f.yaml"), "handlab/builds"),
        (str(PKG / "builds" / "claw4f.yaml"), "handlab/builds"),   # 4-finger hand (default build)
    ],
    hiddenimports=[
        "handlab.bridge",             # imported lazily inside app.main
        "handlab.camera",             # pulls in PySide6.QtMultimedia -> triggers its PyInstaller hook
        "handlab.vision",             # ArUco detection; its `import cv2` is auto-collected IFF the
        #                               [vision] extra is installed at build time (else detection is
        #                               dormant — the app still runs, camera + snapshot unaffected)
        "PySide6.QtMultimedia",       # webcam capture (bundles the native multimedia plugins + ffmpeg)
        "PySide6.QtQuickControls2",
    ],
    hookspath=[],
    runtime_hooks=[],
    excludes=["PySide6.QtWebEngineCore", "PySide6.QtWebEngineWidgets"],  # keep it lean
    noarchive=False,
)

pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    exclude_binaries=True,
    name="handlab",
    debug=False,
    strip=False,
    upx=False,
    console=False,
)

coll = COLLECT(exe, a.binaries, a.datas, strip=False, upx=False, name="handlab")

app = BUNDLE(
    coll,
    name="handlab.app",
    icon=str(ROOT / "packaging" / "handlab.icns"),
    bundle_identifier="dev.handlab.app",
    info_plist={
        "NSHighResolutionCapable": True,
        "LSMinimumSystemVersion": "12.0",
        # required or macOS TCC hard-kills camera capture in the signed .app (Agent-tab live view)
        "NSCameraUsageDescription":
            "handlab shows a live camera view in the Agent tab so you can watch the robotic hand "
            "while Claude drives it, and tracks your hand in the Teleop tab to drive the hand.",
    },
)
