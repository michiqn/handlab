import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import "."

// Hardware connection panel, opened by clicking the status chip in the top bar.
// Scan for the U2D2 serial port, connect or disconnect the Dynamixel bus, set the
// persistent home. Servos stay torque OFF after connecting (engage them deliberately).
Popup {
    id: root
    property var controller
    parent: Overlay.overlay
    anchors.centerIn: parent
    width: 500
    // cap to the window height so the panel never runs off-screen; the body scrolls (Flickable below)
    height: Math.min(bodyCol.implicitHeight + topPadding + bottomPadding,
                     (parent ? parent.height : 900) - 40)
    modal: true
    focus: true
    padding: 18
    background: Rectangle { color: Theme.card; border.color: Theme.line; radius: Theme.rLg }

    property var ports: []
    property string msg: ""
    property bool ok: true
    property bool homeSaved: false      // transient "Saved" state on the Set home button
    property var signs: ({})            // { servoId: +1|-1 }
    readonly property bool hw: controller.driverName === "Dynamixel"
    readonly property bool bridgeLive: typeof handlab !== "undefined" && handlab !== null
    // flat list of all servos in the build (for the per-servo direction toggles)
    readonly property var allServos: {
        var l = []
        for (var i = 0; i < controller.fingers.length; i++)
            for (var j = 0; j < controller.fingers[i].servos.length; j++)
                l.push(controller.fingers[i].servos[j])
        return l
    }

    function rescan() {
        ports = bridgeLive ? JSON.parse(handlab.scanPorts()) : []
        portBox.model = ports
        if (ports.length > 0 && portBox.currentIndex < 0) portBox.currentIndex = 0
    }
    function refreshSigns() { signs = bridgeLive ? JSON.parse(handlab.servoSigns) : ({}) }
    onOpened: { msg = ""; homeSaved = false; rescan(); refreshSigns() }

    Timer { id: homeSavedReset; interval: 1400; onTriggered: root.homeSaved = false }

    contentItem: Flickable {
        contentHeight: bodyCol.implicitHeight
        clip: true
        boundsBehavior: Flickable.StopAtBounds
        ScrollBar.vertical: ScrollBar { policy: ScrollBar.AsNeeded }

        ColumnLayout {
        id: bodyCol
        width: parent.width
        spacing: 14

        // -- header --
        RowLayout {
            Layout.fillWidth: true
            Text { text: "Hardware connection"; color: Theme.text; font.pixelSize: 16; font.weight: Font.Bold }
            Item { Layout.fillWidth: true }
            Rectangle {
                width: 24; height: 24; radius: 7; color: "transparent"; border.color: Theme.line
                Text { anchors.centerIn: parent; text: "×"; color: Theme.text3; font.pixelSize: 14 }
                MouseArea { anchors.fill: parent; cursorShape: Qt.PointingHandCursor; onClicked: root.close() }
            }
        }

        // -- current driver --
        RowLayout {
            spacing: 8
            Rectangle { width: 8; height: 8; radius: 4; color: root.hw ? Theme.green : Theme.accent }
            Text { text: "Driver: " + controller.driverName + (root.hw ? "" : "  (twin stands in for hardware)")
                   color: Theme.text2; font.family: Theme.mono; font.pixelSize: 12 }
        }

        Rectangle { Layout.fillWidth: true; height: 1; color: Theme.line2 }

        // -- port + baud --
        GridLayout {
            Layout.fillWidth: true
            columns: 3; columnSpacing: 10; rowSpacing: 10

            Text { text: "Port"; color: Theme.text3; font.family: Theme.mono; font.pixelSize: 11 }
            ComboBox {
                id: portBox
                Layout.fillWidth: true; Layout.preferredHeight: 32
                model: root.ports
                enabled: !root.hw && root.ports.length > 0
                font.family: Theme.mono; font.pixelSize: 12
            }
            Rectangle {
                Layout.preferredWidth: 76; Layout.preferredHeight: 32; radius: 8
                color: rescanHov.hovered ? Theme.card2 : Theme.panel2
                border.color: rescanHov.hovered ? Theme.accent : Theme.line
                Text { anchors.centerIn: parent; text: "rescan"; color: Theme.text2; font.family: Theme.mono; font.pixelSize: 11 }
                HoverHandler { id: rescanHov; cursorShape: Qt.PointingHandCursor }
                TapHandler {
                    onTapped: {
                        root.rescan()
                        root.ok = root.ports.length > 0
                        root.msg = root.ports.length > 0
                            ? (root.ports.length + " port(s) found")
                            : "No U2D2 found. Plugged in? Check the cable and driver."
                    }
                }
            }

            Text { text: "Baud"; color: Theme.text3; font.family: Theme.mono; font.pixelSize: 11 }
            Rectangle {
                Layout.fillWidth: true; Layout.preferredHeight: 32; radius: 8
                color: Theme.panel2; border.color: Theme.line
                TextInput {
                    id: baudIn
                    anchors.fill: parent; anchors.margins: 7
                    text: "" + (root.bridgeLive ? handlab.hwBaud : 57600)
                    enabled: !root.hw
                    color: Theme.text; font.family: Theme.mono; font.pixelSize: 12
                    validator: IntValidator { bottom: 9600; top: 4500000 }
                    verticalAlignment: TextInput.AlignVCenter
                }
            }
            Item { Layout.preferredWidth: 76 }
        }

        Text {
            visible: root.ports.length === 0 && !root.hw
            text: "No U2D2 found (looking for /dev/cu.usbserial* and cu.usbmodem*). Plug it in and rescan."
            color: Theme.text3; font.family: Theme.mono; font.pixelSize: 11
            wrapMode: Text.WordWrap; Layout.fillWidth: true
        }

        // -- connect / disconnect --
        RowLayout {
            Layout.fillWidth: true; spacing: 8
            Item { Layout.fillWidth: true }
            Rectangle {
                visible: !root.hw
                radius: 8; color: Theme.accentSoft; border.color: Theme.accent
                opacity: (root.ports.length > 0) ? 1 : 0.4
                implicitWidth: cT.implicitWidth + 28; implicitHeight: 32
                Text { id: cT; anchors.centerIn: parent; text: "Connect"; color: Theme.accent; font.pixelSize: 12; font.weight: Font.DemiBold }
                MouseArea {
                    anchors.fill: parent; enabled: root.ports.length > 0
                    cursorShape: enabled ? Qt.PointingHandCursor : Qt.ArrowCursor
                    onClicked: {
                        var r = JSON.parse(handlab.connectHardware(portBox.currentText, parseInt(baudIn.text)))
                        root.ok = r.ok === true
                        root.refreshSigns()
                        root.msg = r.ok ? ("Connected. Servos online: " + r.online.join(", ") + ". Torque is off (turn it on deliberately).")
                                        : ("Connection failed: " + r.error)
                    }
                }
            }
            Rectangle {
                visible: root.hw
                radius: 8; color: Theme.soft(Theme.red, 0.15); border.color: Theme.red
                implicitWidth: dT.implicitWidth + 28; implicitHeight: 32
                Text { id: dT; anchors.centerIn: parent; text: "Disconnect"; color: Theme.red; font.pixelSize: 12; font.weight: Font.DemiBold }
                MouseArea {
                    anchors.fill: parent; cursorShape: Qt.PointingHandCursor
                    onClicked: {
                        handlab.disconnectHardware()
                        root.ok = true; root.msg = "Disconnected. Back on the sim (the hand stays limp)."
                    }
                }
            }
        }

        // first moves are speed-capped by default
        Text {
            text: "Motion is speed limited at first (profile velocity). Raise it carefully on day one."
            color: Theme.text3; font.family: Theme.mono; font.pixelSize: 10
            wrapMode: Text.WordWrap; Layout.fillWidth: true
        }

        Rectangle { Layout.fillWidth: true; height: 1; color: Theme.line2 }

        // -- persistent absolute-encoder home (survives power-cycle) --
        ColumnLayout {
            Layout.fillWidth: true; spacing: 8
            Text { text: "HOME"; color: Theme.text3; font.family: Theme.mono; font.pixelSize: 11; font.letterSpacing: 1.5 }
            Text {
                Layout.fillWidth: true; wrapMode: Text.WordWrap; lineHeight: 1.3
                text: "Spread and flex have no return spring, so they stay where you leave them. Hold the hand cleanly stretched once and press Set home. The absolute encoder angle is stored persistently, so the app finds home again on every connect no matter how the hand is resting (it only reads, it moves nothing)."
                color: Theme.text2; font.pixelSize: 12
            }
            Flow {
                Layout.fillWidth: true; spacing: 8
                Rectangle {
                    radius: 8; color: root.homeSaved ? Theme.soft(Theme.green, 0.16) : Theme.card2
                    border.color: root.homeSaved ? Theme.green : (root.hw ? Theme.accent : Theme.line)
                    opacity: root.hw ? 1 : 0.4
                    implicitWidth: shT.implicitWidth + 28; implicitHeight: 32
                    Text { id: shT; anchors.centerIn: parent
                           text: root.homeSaved ? "Saved ✓" : "Set home (hand stretched)"
                           color: root.homeSaved ? Theme.green : (root.hw ? Theme.accent : Theme.text3)
                           font.pixelSize: 12; font.weight: Font.DemiBold }
                    MouseArea {
                        anchors.fill: parent; enabled: root.hw
                        cursorShape: enabled ? Qt.PointingHandCursor : Qt.ArrowCursor
                        onClicked: {
                            var r = JSON.parse(handlab.setAbsoluteHome())
                            root.ok = r.ok === true
                            if (r.ok) { root.homeSaved = true; homeSavedReset.restart() }
                            root.msg = r.ok ? "Home stored. It survives restart and power cycle."
                                            : ("Set home failed: " + r.error)
                        }
                    }
                }
                // per-finger: anchor ONE finger's home (e.g. a freshly-strung thumb), the rest keep theirs
                Repeater {
                    model: controller.fingers
                    delegate: Rectangle {
                        required property var modelData
                        required property int index
                        radius: 8; color: Theme.card2; border.color: root.hw ? Theme.line : Theme.line2
                        opacity: root.hw ? 1 : 0.4
                        implicitWidth: shfT.implicitWidth + 24; implicitHeight: 32
                        Text { id: shfT; anchors.centerIn: parent; text: "Home " + modelData.socket
                               color: root.hw ? Theme.text2 : Theme.text3; font.family: Theme.mono; font.pixelSize: 11 }
                        MouseArea {
                            anchors.fill: parent; enabled: root.hw
                            cursorShape: enabled ? Qt.PointingHandCursor : Qt.ArrowCursor
                            onClicked: {
                                var r = JSON.parse(handlab.setAbsoluteHomeFinger(index))
                                root.ok = r.ok === true
                                root.msg = r.ok ? ("Home set: " + modelData.socket) : ("Set home failed: " + r.error)
                            }
                        }
                    }
                }
            }
        }

        // -- direction (sign): mirror the real hand onto the twin --
        ColumnLayout {
            visible: root.hw
            Layout.fillWidth: true; spacing: 8
            Text { text: "DIRECTION"; color: Theme.text3; font.family: Theme.mono; font.pixelSize: 11; font.letterSpacing: 1.5 }
            Text {
                Layout.fillWidth: true; wrapMode: Text.WordWrap; lineHeight: 1.3
                text: "Does the real hand run mirrored to the twin (slider up gives joint down)? Flip the hardware direction here, per finger or all at once. The twin is untouched (its direction comes from the CAD export). Saved in calibration.yaml."
                color: Theme.text2; font.pixelSize: 12
            }
            // primary: flip a whole finger (or the whole hand) in one click
            Flow {
                Layout.fillWidth: true; spacing: 8
                Repeater {
                    model: controller.fingers
                    delegate: Rectangle {
                        required property var modelData
                        required property int index
                        // finger is "inverted" only when ALL its servos are -1
                        readonly property bool inv: {
                            if (!modelData.servos.length) return false
                            for (var j = 0; j < modelData.servos.length; j++)
                                if (root.signs[modelData.servos[j].id] !== -1) return false
                            return true
                        }
                        radius: 8; color: inv ? Theme.accentSoft : Theme.card2
                        border.color: inv ? Theme.accent : Theme.line
                        implicitWidth: fLbl.implicitWidth + 22; implicitHeight: 30
                        Text { id: fLbl; anchors.centerIn: parent
                               text: "⇄ " + modelData.socket + (inv ? "  inverted" : "")
                               color: inv ? Theme.accent : Theme.text2; font.family: Theme.mono; font.pixelSize: 11 }
                        MouseArea { anchors.fill: parent; cursorShape: Qt.PointingHandCursor
                            onClicked: {
                                var r = JSON.parse(handlab.setFingerSign(index, inv ? 1 : -1))
                                if (r.ok) root.refreshSigns()
                            }
                        }
                    }
                }
                Rectangle {
                    id: allBtn                       // id so nested Text/MouseArea can see allInv
                    readonly property bool allInv: {
                        if (!root.allServos.length) return false
                        for (var k = 0; k < root.allServos.length; k++)
                            if (root.signs[root.allServos[k].id] !== -1) return false
                        return true
                    }
                    radius: 8; color: allInv ? Theme.accentSoft : Theme.card2
                    border.color: allInv ? Theme.accent : Theme.line
                    implicitWidth: aLbl.implicitWidth + 22; implicitHeight: 30
                    Text { id: aLbl; anchors.centerIn: parent; text: "⇄ all"
                           color: allBtn.allInv ? Theme.accent : Theme.text2; font.family: Theme.mono; font.pixelSize: 11 }
                    MouseArea { anchors.fill: parent; cursorShape: Qt.PointingHandCursor
                        onClicked: {
                            var r = JSON.parse(handlab.setAllSigns(allBtn.allInv ? 1 : -1))
                            if (r.ok) root.refreshSigns()
                        }
                    }
                }
            }
            // fine-tuning: per individual servo
            Text { text: "per servo:"; color: Theme.text3; font.family: Theme.mono; font.pixelSize: 10 }
            Flow {
                Layout.fillWidth: true; spacing: 8
                Repeater {
                    model: root.allServos
                    delegate: Rectangle {
                        required property var modelData
                        readonly property bool inv: root.signs[modelData.id] === -1
                        radius: 8; color: inv ? Theme.accentSoft : Theme.card2
                        border.color: inv ? Theme.accent : Theme.line
                        implicitWidth: sLbl.implicitWidth + 22; implicitHeight: 28
                        Text { id: sLbl; anchors.centerIn: parent
                               text: "#" + modelData.id + " " + modelData.dofId + (inv ? "  ⇄" : "")
                               color: inv ? Theme.accent : Theme.text2; font.family: Theme.mono; font.pixelSize: 11 }
                        MouseArea { anchors.fill: parent; cursorShape: Qt.PointingHandCursor
                            onClicked: {
                                var r = JSON.parse(handlab.setServoSign(modelData.id, inv ? 1 : -1))
                                if (r.ok) root.refreshSigns()
                            }
                        }
                    }
                }
            }
        }

        // -- revive: clear a latched hardware error (overload shutdown) via reboot --
        ColumnLayout {
            id: reviveSec
            visible: root.hw
            Layout.fillWidth: true; spacing: 8
            readonly property bool anyBad: {
                for (var i = 0; i < root.allServos.length; i++) {
                    var t = controller.telemetry[root.allServos[i].id]
                    if (t !== undefined && ((t.err || 0) !== 0 || !t.online)) return true
                }
                return false
            }
            Text { text: "REVIVE"; color: Theme.text3; font.family: Theme.mono; font.pixelSize: 11; font.letterSpacing: 1.5 }
            Text {
                Layout.fillWidth: true; wrapMode: Text.WordWrap; lineHeight: 1.3
                text: reviveSec.anyBad
                      ? "⚠ A servo shut down (overload) or is not responding. Reboot clears the error. Then re-anchor that finger with Set home (the rest pose re-wraps on reboot) and turn torque back on."
                      : "No servo in an error state ✓ (overload shutdowns show up here, with a warning on the servo chip)."
                color: reviveSec.anyBad ? Theme.text2 : Theme.text3; font.pixelSize: 12
            }
            Flow {
                Layout.fillWidth: true; spacing: 8
                Repeater {
                    model: root.allServos
                    delegate: Rectangle {
                        required property var modelData
                        readonly property var tele: controller.telemetry[modelData.id]
                        readonly property int err: (tele !== undefined && tele.err !== undefined) ? tele.err : 0
                        readonly property bool bad: err !== 0 || (tele !== undefined && !tele.online)
                        visible: bad
                        radius: 8; color: Theme.card2; border.color: Theme.red
                        implicitWidth: rbT.implicitWidth + 22; implicitHeight: 30
                        Text { id: rbT; anchors.centerIn: parent
                               text: "↻ Reboot #" + modelData.id + (err ? "  (err " + err + ")" : "")
                               color: Theme.red; font.family: Theme.mono; font.pixelSize: 11 }
                        MouseArea { anchors.fill: parent; cursorShape: Qt.PointingHandCursor
                            onClicked: {
                                var r = JSON.parse(handlab.rebootServo(modelData.id))
                                root.ok = r.ok === true
                                root.msg = r.ok ? ("Servo #" + modelData.id + " rebooted. " + r.hint) : ("Reboot failed: " + r.error)
                            }
                        }
                    }
                }
            }
        }

        // -- status line --
        Text {
            visible: root.msg !== ""
            text: root.msg
            color: root.ok ? Theme.green : Theme.red
            font.family: Theme.mono; font.pixelSize: 11
            wrapMode: Text.WordWrap; Layout.fillWidth: true
        }
        }
    }
}
