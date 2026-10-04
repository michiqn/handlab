"""app.py — launch the handlab desktop app (PySide6 + QML).

Loads the QML UI (ui/qml/Main.qml) and wires the Bridge as context property `handlab`.
The current QML is still mock-data-driven, so the Bridge is optional — if it can't be
created (missing deps), the UI still loads. Run with:
    python -m handlab
"""

from __future__ import annotations

import sys
from pathlib import Path

QML_DIR = Path(__file__).resolve().parent / "ui" / "qml"


def main(argv: list[str] | None = None) -> int:
    from PySide6.QtGui import QGuiApplication, QIcon
    from PySide6.QtQml import QQmlApplicationEngine
    from PySide6.QtQuickControls2 import QQuickStyle

    app = QGuiApplication(argv if argv is not None else sys.argv)
    app.setApplicationName("handlab")
    app.setOrganizationName("handlab")          # gives QtCore Settings a stable store
    app.setWindowIcon(QIcon(str(QML_DIR.parent / "assets" / "logo_mark.png")))
    # The custom Slider/Switch styling needs a non-native style (the macOS native style
    # refuses customization of handle/background).
    QQuickStyle.setStyle("Basic")

    engine = QQmlApplicationEngine()
    engine.addImportPath(str(QML_DIR))

    # Bridge owns the build + MuJoCo twin and exposes it to QML.
    # HANDLAB_BUILD selects which builds/<name>.yaml to boot (default claw4f = the
    # 4-finger hand; claw3f is still there for the 3-finger claw).
    bridge = None
    try:
        import os
        from .bridge import Bridge
        bridge = Bridge(os.environ.get("HANDLAB_BUILD", "claw4f"))
        engine.addImageProvider("twin", bridge.image_provider)   # image://twin/<id> -> live frames
        if bridge.camera is not None:
            engine.addImageProvider("cam", bridge.camera.provider)   # image://cam/f<n> -> live webcam
            app.aboutToQuit.connect(bridge.camera.stop)              # release the device on quit
        engine.rootContext().setContextProperty("handlab", bridge)
    except Exception as e:  # never let backend issues block the UI
        print(f"[handlab] Bridge not active ({e}); UI runs on mock data.", file=sys.stderr)

    # Agent control plane: a token-gated localhost API onto the live Bridge, so the handlab MCP
    # server (→ Claude) drives the SAME hand a human drives (see mcp_server.py). Starts DISARMED
    # (a human arms it in the Agent panel); opt out with HANDLAB_AGENT=0.
    if bridge is not None and os.environ.get("HANDLAB_AGENT", "1") != "0":
        try:
            import json
            import secrets
            from .control_server import ControlServer, write_discovery
            token = os.environ.get("HANDLAB_AGENT_TOKEN") or secrets.token_urlsafe(18)
            want_port = int(os.environ.get("HANDLAB_AGENT_PORT", "8765"))
            # each mutating command Claude issues → one line in the Agent-tab log. The emit runs
            # inside the ControlApi._act → qt_invoke hop (Qt thread), so QML receives it safely.
            log = lambda e: bridge.agentActivity.emit(json.dumps(e))   # noqa: E731
            server = None
            for p in (want_port, 0):                 # fall back to an ephemeral port if 8765 is taken
                try:
                    server = ControlServer(bridge, bridge.qt_invoke, token, port=p, log=log)
                    break
                except OSError:
                    continue
            if server is not None:
                server.start()
                bridge._agent_port = server.port     # Agent-tab status chip reads bridge.agentPort
                disc = write_discovery(server.port, token)
                print(f"[handlab] agent control API on 127.0.0.1:{server.port} "
                      f"(disarmed; token in {disc})")
                app.aboutToQuit.connect(server.stop)
        except Exception as e:
            print(f"[handlab] agent control API not started ({e}).", file=sys.stderr)

    engine.load(str(QML_DIR / "Main.qml"))
    if not engine.rootObjects():
        print("Failed to load ui/qml/Main.qml", file=sys.stderr)
        return 1
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
