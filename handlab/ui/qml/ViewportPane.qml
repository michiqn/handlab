import QtQuick
import QtQuick.Controls
import "."

// The live MuJoCo twin viewport: an Image fed image://twin frames + free-camera
// controls (orbit/zoom/pan) + orientation gizmo, shown on every tab once frames are
// available. Without a bridge (standalone QML) it falls back to the drawn base
// schematic + finger slots so the modular layout stays visible.
Rectangle {
    id: root
    property var controller
    color: Theme.viewport
    Behavior on color { ColorAnimation { duration: 180 } }
    clip: true

    readonly property var mount: controller.curMount()
    // the live articulated twin shows on every tab once the Bridge produces frames (Control drives
    // it, the Agent cockpit + Teleop watch it); the schematic is only the no-bridge fallback.
    readonly property bool liveTwin: controller.twinFrame > 0
    readonly property bool twinPaused: (typeof handlab !== "undefined" && handlab) ? handlab.twinPaused : false

    // radial vignette
    Rectangle {
        anchors.fill: parent
        gradient: Gradient {
            GradientStop { position: 0.0; color: Theme.viewport2 }
            GradientStop { position: 1.0; color: Theme.viewport }
        }
        opacity: 0.9
    }

    // grid
    Canvas {
        id: gridCanvas
        anchors.fill: parent
        onPaint: {
            var ctx = getContext("2d"); ctx.reset()
            ctx.strokeStyle = Theme.dark ? "rgba(130,150,170,0.07)" : "rgba(40,60,90,0.07)"
            ctx.lineWidth = 1
            var step = 34
            for (var x = 0; x < width; x += step) { ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, height); ctx.stroke() }
            for (var y = 0; y < height; y += step) { ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(width, y); ctx.stroke() }
        }
        onWidthChanged: requestPaint()
        onHeightChanged: requestPaint()
        Component.onCompleted: requestPaint()
        Connections { target: Theme; function onModeChanged() { gridCanvas.requestPaint() } }
    }

    // ── live MuJoCo twin frame (Control tab) ──────────────────────
    Image {
        anchors.fill: parent
        anchors.margins: 8
        visible: root.liveTwin
        cache: false
        fillMode: Image.PreserveAspectFit
        smooth: true
        source: root.liveTwin ? "image://twin/f" + controller.twinFrame : ""
    }

    // ── viewport camera control (drives the real MuJoCo free camera via the Bridge) ──
    // mouse:    drag = orbit (around center) · ⇧drag = pan · ⌥drag = pan world-X · wheel = zoom
    // trackpad: 2-finger = pan · ⌥2-finger = pan world-X · ⇧2-finger = orbit · pinch/⌘ = zoom
    // dbl-click = reset.  Deltas inverted so the model follows the cursor / fingers.
    MouseArea {
        id: camMouse
        anchors.fill: parent
        enabled: root.liveTwin && typeof handlab !== "undefined"
        visible: enabled
        cursorShape: pressed ? Qt.ClosedHandCursor : Qt.OpenHandCursor
        property real lastX: 0
        property real lastY: 0
        onPressed: function(m) { lastX = m.x; lastY = m.y }
        onPositionChanged: function(m) {
            if (m.modifiers & Qt.AltModifier) {
                // ⌥ Option + drag → pan locked to the world X axis
                handlab.panX((lastX - m.x) / 500)
            } else if (m.modifiers & Qt.ShiftModifier) {
                // pixels → view fractions; the model follows the cursor
                handlab.pan((lastX - m.x) / 500, (m.y - lastY) / 500)
            } else {
                // pixels → degrees. Azimuth: drag right → turn right (m.x - lastX).
                // Elevation: drag up → tilt up (lastY - m.y). Both match the cursor.
                handlab.orbit((m.x - lastX) * 0.4, (lastY - m.y) * 0.4)
            }
            lastX = m.x; lastY = m.y
        }
        onWheel: function(w) {
            // A trackpad 2-finger swipe carries pixelDelta; a real mouse wheel only sets
            // angleDelta (120-steps). Route trackpad gestures to pan/orbit by modifier; a
            // bare wheel stays zoom.
            var px = w.pixelDelta
            var trackpad = (px.x !== 0 || px.y !== 0)
            var dx = trackpad ? px.x : w.angleDelta.x
            var dy = trackpad ? px.y : w.angleDelta.y
            if (w.modifiers & Qt.MetaModifier) {              // ⌘ + 2-finger → zoom
                if (dy !== 0) handlab.zoom(Math.pow(0.992, dy))
            } else if (w.modifiers & Qt.ShiftModifier) {      // ⇧ + 2-finger → orbit around center
                handlab.orbit(dx * 0.35, -dy * 0.35)           // azimuth: swipe right → turn right (match mouse-drag)
            } else if (w.modifiers & Qt.AltModifier) {        // ⌥ + 2-finger → pan world-X
                handlab.panX(-dx / 400)
            } else if (trackpad) {                            // 2-finger → free pan (view follows fingers)
                handlab.pan(dx / 400, -dy / 400)
            } else if (dy !== 0) {                            // mouse wheel → zoom
                handlab.zoom(dy > 0 ? 0.9 : 1.0 / 0.9)
            }
        }
        onDoubleClicked: handlab.resetView()
    }

    // pinch-to-zoom (macOS trackpad native gesture; coexists with the handlers above)
    PinchHandler {
        target: null
        enabled: root.liveTwin && typeof handlab !== "undefined"
        property real prevScale: 1.0
        onActiveChanged: if (active) prevScale = 1.0
        onActiveScaleChanged: {
            if (activeScale > 0.0001) {
                handlab.zoom(prevScale / activeScale)   // spread fingers → zoom in
                prevScale = activeScale
            }
        }
    }

    // ── top-left meta ─────────────────────────────────────────────
    Column {
        x: 16; y: 16; spacing: 7
        Row {
            spacing: 8
            Rectangle { width: 6; height: 6; radius: 3; color: Theme.accent; anchors.verticalCenter: parent.verticalCenter }
            Text { text: "digital twin · MuJoCo frame"; color: Theme.text2; font.family: Theme.mono; font.pixelSize: 11 }
        }
        Text {
            text: (root.mount ? root.mount.name : "no mount") + " · "
                  + controller.fingers.length + "/" + controller.socketCount() + " sockets · "
                  + controller.motorsUsed() + " dof"
            color: Theme.text3; font.family: Theme.mono; font.pixelSize: 10
        }
    }

    // ── physics / grasp playground controls (live twin only) ──────
    Row {
        id: physRow
        visible: root.liveTwin && typeof handlab !== "undefined"
        x: 16; y: 52; spacing: 8
        readonly property bool on: (typeof handlab !== "undefined" && handlab) ? handlab.physicsOn : false
        Rectangle {
            radius: 8; height: 28; implicitWidth: phT.implicitWidth + 22
            color: physRow.on ? Theme.accentSoft : Theme.card2
            border.color: physRow.on ? Theme.accent : Theme.line
            Text { id: phT; anchors.centerIn: parent; text: (physRow.on ? "■ " : "□ ") + "Physics"
                   color: physRow.on ? Theme.accent : Theme.text2; font.family: Theme.mono; font.pixelSize: 11 }
            MouseArea { anchors.fill: parent; cursorShape: Qt.PointingHandCursor
                        onClicked: if (typeof handlab !== "undefined") handlab.setPhysics(!physRow.on) }
        }
        Rectangle {
            visible: physRow.on
            radius: 8; height: 28; implicitWidth: drT.implicitWidth + 22
            color: drHov.hovered ? Theme.accentSoft : Theme.card2; border.color: drHov.hovered ? Theme.accent : Theme.line
            Text { id: drT; anchors.centerIn: parent; text: "● Drop"; color: Theme.text2; font.family: Theme.mono; font.pixelSize: 11 }
            HoverHandler { id: drHov; cursorShape: Qt.PointingHandCursor }
            TapHandler { onTapped: if (typeof handlab !== "undefined") handlab.dropObject() }
        }
        // perf escape hatch: freeze the offscreen render (teleop keeps driving; twin shows last frame)
        Rectangle {
            radius: 8; height: 28; implicitWidth: pzT.implicitWidth + 22
            color: root.twinPaused ? Theme.soft(Theme.amber, 0.18) : Theme.card2
            border.color: root.twinPaused ? Theme.amber : Theme.line
            Text { id: pzT; anchors.centerIn: parent; text: (root.twinPaused ? "▶ " : "⏸ ") + "Twin"
                   color: root.twinPaused ? Theme.amber : Theme.text2; font.family: Theme.mono; font.pixelSize: 11 }
            MouseArea { anchors.fill: parent; cursorShape: Qt.PointingHandCursor
                        onClicked: if (typeof handlab !== "undefined") handlab.setTwinPaused(!root.twinPaused) }
        }
    }

    // ── top-right orientation gizmo (LIVE — turns with the MuJoCo camera) ──────────
    // Projects the world axes onto the screen from the camera azimuth/elevation, so it
    // reads like Fusion's nav-triad: it tells you which way X/Y/Z point RIGHT NOW (a static
    // legend can't). Re-reads az/el every rendered frame (twinFrame ticks ~30 Hz); depth-
    // sorted so the axis pointing at you draws on top. Colours match the in-scene world triad.
    Item {
        id: gizmo
        x: parent.width - width - 14; y: 12
        width: 82; height: 82
        visible: root.liveTwin && typeof handlab !== "undefined" && handlab
        readonly property real az: { controller.twinFrame; return gizmo.visible ? handlab.camAzimuth() : 45 }
        readonly property real el: { controller.twinFrame; return gizmo.visible ? handlab.camElevation() : 20 }
        onAzChanged: giz.requestPaint()
        onElChanged: giz.requestPaint()
        Connections { target: Theme; function onModeChanged() { giz.requestPaint() } }
        Canvas {
            id: giz
            anchors.fill: parent
            onPaint: {
                var ctx = getContext("2d"); ctx.reset()
                var cx = width / 2, cy = height / 2, R = 27
                var a = gizmo.az * Math.PI / 180, e = gizmo.el * Math.PI / 180
                var sa = Math.sin(a), ca = Math.cos(a), se = Math.sin(e), ce = Math.cos(e)
                // MuJoCo free cam: right=(-sa,ca,0)  up=(-ca·se,-sa·se,ce)  into-screen=(ca·ce,sa·ce,se)
                var ax = [
                    { n: "X", c: Theme.red,    vx: 1, vy: 0, vz: 0 },
                    { n: "Y", c: Theme.green,  vx: 0, vy: 1, vz: 0 },
                    { n: "Z", c: Theme.accent, vx: 0, vy: 0, vz: 1 }
                ]
                for (var i = 0; i < ax.length; ++i) {
                    var v = ax[i]
                    // Calibrated vs the in-scene world triad: MuJoCo renders +X to the right at az=90,
                    // so screen_x = dot(v, [sa,-ca,0]); canvas y is down → screen_y = -dot(v, up).
                    v.sx = sa * v.vx - ca * v.vy
                    v.sy = -((-ca * se) * v.vx + (-sa * se) * v.vy + ce * v.vz)
                    v.near = -((ca * ce) * v.vx + (sa * ce) * v.vy + se * v.vz)   // >0 = toward viewer
                }
                ax.sort(function(p, q) { return p.near - q.near })              // far first, near last (drawn on top)
                ctx.lineWidth = 2.4; ctx.lineCap = "round"
                ctx.font = "600 11px monospace"; ctx.textAlign = "center"; ctx.textBaseline = "middle"
                for (i = 0; i < ax.length; ++i) {
                    v = ax[i]
                    var ex = cx + v.sx * R, ey = cy + v.sy * R
                    ctx.globalAlpha = 0.4 + 0.6 * (0.5 + 0.5 * Math.max(-1, Math.min(1, v.near)))
                    ctx.strokeStyle = v.c
                    ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(ex, ey); ctx.stroke()
                    ctx.fillStyle = v.c
                    ctx.beginPath(); ctx.arc(ex, ey, 7, 0, 2 * Math.PI); ctx.fill()
                    ctx.globalAlpha = 1.0
                    ctx.fillStyle = Theme.dark ? "#0c0f13" : "#ffffff"
                    ctx.fillText(v.n, ex, ey + 0.5)
                }
            }
        }
    }

    // ── base schematic + finger slots ─────────────────────────────
    Item {
        id: stage
        visible: !root.liveTwin
        width: 300; height: 300
        anchors.centerIn: parent
        readonly property real cx: width / 2
        readonly property real cy: height / 2
        // finger sockets = the adapter's ports (at the hub)
        readonly property var socketDefs: controller.socketList
        readonly property int nPorts: { var ad = controller.curAdapter(); return ad ? ad.ports.length : 0 }

        // hub plate (the center norm — adapter + its ports sit on top of this)
        Rectangle {
            anchors.centerIn: parent
            width: 74; height: 74; radius: 20
            color: Theme.panel; border.color: Theme.line
        }

        // adapter placeholder — the center norm awaits an adapter
        Rectangle {
            visible: stage.nPorts === 0
            width: 50; height: 50; radius: 16
            x: stage.cx - width / 2; y: stage.cy - height / 2
            color: "transparent"; border.width: 1.5; border.color: Theme.line
            opacity: 0.85
            Column {
                anchors.centerIn: parent; spacing: 2
                Text { anchors.horizontalCenter: parent.horizontalCenter; text: "A"
                       color: Theme.text3; font.family: Theme.mono; font.pixelSize: 13 }
                Text { anchors.horizontalCenter: parent.horizontalCenter; text: "adapter"
                       color: Theme.text3; font.family: Theme.mono; font.pixelSize: 7; font.letterSpacing: 0.5 }
            }
        }

        // sockets — the adapter's ports, fanned on a small ring around the hub
        Repeater {
            model: stage.socketDefs
            delegate: Item {
                id: sock
                required property var modelData
                required property int index
                readonly property real ang: index * 2 * Math.PI / Math.max(stage.nPorts, 1) - Math.PI / 2
                readonly property real r: stage.nPorts > 1 ? 24 : 0
                readonly property var finger: controller.fingerOnSocket(modelData.id)
                readonly property bool filled: finger !== null
                readonly property color idc: controller.idColorFor(index)
                width: 64; height: 64
                x: stage.cx + r * Math.cos(ang) - width / 2
                y: stage.cy + r * Math.sin(ang) - height / 2

                // filled socket
                Rectangle {
                    visible: sock.filled
                    anchors.centerIn: parent
                    width: 54; height: 54; radius: 16
                    color: Theme.card
                    border.width: 1.5; border.color: sock.idc
                    Column {
                        anchors.centerIn: parent; spacing: 5
                        Text { anchors.horizontalCenter: parent.horizontalCenter
                               text: modelData.short; color: sock.idc
                               font.family: Theme.mono; font.pixelSize: 13; font.weight: Font.DemiBold }
                        Row {
                            anchors.horizontalCenter: parent.horizontalCenter; spacing: 4
                            Repeater {
                                model: (sock.finger && controller.typeById(sock.finger.typeId)) ? controller.typeById(sock.finger.typeId).dofs : []
                                delegate: Rectangle { width: 6; height: 6; radius: 3; color: Theme.dofColor(modelData.group) }
                            }
                        }
                    }
                }

                // empty socket
                Rectangle {
                    visible: !sock.filled
                    anchors.centerIn: parent
                    width: 50; height: 50; radius: 16
                    color: "transparent"
                    border.width: 1.5; border.color: Theme.line
                    Text { anchors.centerIn: parent; text: modelData.short
                           color: Theme.text3; font.family: Theme.mono; font.pixelSize: 12 }
                }
            }
        }
    }

    // empty hint
    Text {
        visible: controller.fingers.length === 0
        anchors.horizontalCenter: parent.horizontalCenter
        y: parent.height - 96
        text: "no fingers in this build - edit builds/<name>.yaml"
        color: Theme.text3; font.family: Theme.mono; font.pixelSize: 11
    }

    // bottom hints
    Row {
        x: 16; y: parent.height - 28; spacing: 12
        Text { text: "\u2299 orbit"; color: Theme.text3; font.family: Theme.mono; font.pixelSize: 11 }
        Text { text: "\u2325 pan-X"; color: Theme.text3; font.family: Theme.mono; font.pixelSize: 11 }
        Text { text: "\u21E7 pan"; color: Theme.text3; font.family: Theme.mono; font.pixelSize: 11 }
        Text { text: "\u2318/pinch zoom"; color: Theme.text3; font.family: Theme.mono; font.pixelSize: 11 }
        Text { text: "\u29BF 2\u00D7reset"; color: Theme.text3; font.family: Theme.mono; font.pixelSize: 11 }
    }
    Row {
        x: parent.width - width - 16; y: parent.height - 28; spacing: 16
        Text { text: controller.twinFps + " fps"; color: Theme.text3; font.family: Theme.mono; font.pixelSize: 11 }
        Text { text: controller.motorsUsed() + " servos"; color: Theme.text3; font.family: Theme.mono; font.pixelSize: 11 }
    }

    // ── e-stop overlay ────────────────────────────────────────────
    Rectangle {
        anchors.fill: parent; visible: controller.eStopped
        color: "transparent"; border.width: 3; border.color: Theme.red
        SequentialAnimation on opacity {
            running: controller.eStopped; loops: Animation.Infinite
            NumberAnimation { to: 0.4; duration: 550 }
            NumberAnimation { to: 0.9; duration: 550 }
        }
    }
    Rectangle {
        visible: controller.eStopped
        anchors.horizontalCenter: parent.horizontalCenter; y: 16
        radius: 10; color: Theme.red
        implicitWidth: estopRow.implicitWidth + 36; implicitHeight: 38
        Row {
            id: estopRow; anchors.centerIn: parent; spacing: 10
            Rectangle { width: 11; height: 11; radius: 2; color: "#fff"; anchors.verticalCenter: parent.verticalCenter }
            Text { text: "EMERGENCY STOP — ALL TORQUE DISABLED"; color: "#fff"; font.pixelSize: 13; font.weight: Font.Bold; font.letterSpacing: 1 }
        }
    }
}
