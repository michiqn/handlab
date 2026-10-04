import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import "."

// Screen 3 — the Agent cockpit: the human's supervision surface while CLAUDE drives the hand
// through the MCP control server. ARM (a deliberate human action — the MCP client can only
// DISARM), a big STOP, an MCP status chip, and a live log of every command Claude issued. The
// twin itself renders in the shared left ViewportPane (also live on this tab). Read tools work
// disarmed; moves need ARM. Nothing here bypasses the existing safety — it only surfaces it.
Item {
    id: root
    property var controller

    readonly property bool armed: (typeof handlab !== "undefined" && handlab) ? handlab.armed : false
    readonly property int mcpPort: (typeof handlab !== "undefined" && handlab) ? handlab.agentPort : 0
    readonly property bool estop: controller.eStopped

    // webcam (Phase 2): live view + which source the MCP snapshot returns
    readonly property int camFrame: (typeof handlab !== "undefined" && handlab) ? handlab.camFrame : 0
    readonly property bool camActive: (typeof handlab !== "undefined" && handlab) ? handlab.cameraActive : false
    readonly property string snapSource: (typeof handlab !== "undefined" && handlab) ? handlab.snapshotSource : "twin"
    property var camDevices: (typeof handlab !== "undefined" && handlab) ? JSON.parse(handlab.cameraDevices) : []
    property string camError: ""
    readonly property int markerCount: (typeof handlab !== "undefined" && handlab) ? handlab.markerCount : 0

    ColumnLayout {
        anchors.fill: parent
        spacing: 0

        // ── header ────────────────────────────────────────────────
        Rectangle {
            Layout.fillWidth: true; implicitHeight: 66; color: "transparent"
            Rectangle { anchors.bottom: parent.bottom; width: parent.width; height: 1; color: Theme.line }
            RowLayout {
                anchors.fill: parent; anchors.leftMargin: 20; anchors.rightMargin: 20; spacing: 10
                ColumnLayout {
                    spacing: 3
                    Text { text: "Agent"; color: Theme.text; font.pixelSize: 17; font.weight: Font.DemiBold }
                    Text { text: "supervised · Claude drives via MCP"; color: Theme.text2; font.pixelSize: 12 }
                }
                Item { Layout.fillWidth: true }
                // MCP status chip — the local control API port (green = up)
                Rectangle {
                    radius: 8; implicitHeight: 26; implicitWidth: mcpRow.implicitWidth + 20
                    color: Theme.panel2; border.color: root.mcpPort > 0 ? Theme.line : Theme.line2
                    Row {
                        id: mcpRow; anchors.centerIn: parent; spacing: 7
                        Rectangle { width: 7; height: 7; radius: 3.5; anchors.verticalCenter: parent.verticalCenter
                                    color: root.mcpPort > 0 ? Theme.green : Theme.text3 }
                        Text { text: root.mcpPort > 0 ? ("MCP :" + root.mcpPort) : "MCP off"
                               color: Theme.text2; font.family: Theme.mono; font.pixelSize: 11 }
                    }
                }
            }
        }

        // ── ARM + STOP ────────────────────────────────────────────
        RowLayout {
            Layout.fillWidth: true; Layout.margins: 20; spacing: 14

            // ARM toggle — the software gate. Disabled while e-stopped (release the stop first).
            Rectangle {
                Layout.fillWidth: true; implicitHeight: 86; radius: Theme.rLg
                enabled: !root.estop
                opacity: enabled ? 1 : 0.4
                color: root.armed ? Theme.soft(Theme.green, 0.16) : Theme.card
                border.color: root.armed ? Theme.green : Theme.line
                border.width: root.armed ? 2 : 1
                Behavior on border.color { ColorAnimation { duration: 140 } }
                ColumnLayout {
                    anchors.centerIn: parent; spacing: 4
                    Text { Layout.alignment: Qt.AlignHCenter
                           text: root.armed ? "● ARMED" : "○ DISARMED"
                           color: root.armed ? Theme.green : Theme.text2
                           font.pixelSize: 19; font.weight: Font.Bold; font.letterSpacing: 1 }
                    Text { Layout.alignment: Qt.AlignHCenter
                           text: root.armed ? "Claude can move the hand — click to disarm"
                                            : "click to ARM — lets Claude move the hand"
                           color: Theme.text3; font.pixelSize: 11 }
                }
                MouseArea {
                    anchors.fill: parent; enabled: parent.enabled
                    cursorShape: enabled ? Qt.PointingHandCursor : Qt.ArrowCursor
                    onClicked: if (typeof handlab !== "undefined" && handlab) handlab.setArmed(!root.armed)
                }
            }

            // STOP — the shared app-wide e-stop (aborts homing, disarms, drops all torque).
            Rectangle {
                Layout.preferredWidth: 155; implicitHeight: 86; radius: Theme.rLg
                color: root.estop ? Theme.red : Theme.soft(Theme.red, 0.14)
                border.color: Theme.red; border.width: root.estop ? 0 : 1
                SequentialAnimation on opacity {
                    running: root.estop; loops: Animation.Infinite
                    NumberAnimation { to: 0.55; duration: 550 }
                    NumberAnimation { to: 1.0;  duration: 550 }
                }
                ColumnLayout {
                    anchors.centerIn: parent; spacing: 2
                    Text { Layout.alignment: Qt.AlignHCenter; text: "■ STOP"
                           color: root.estop ? "#fff" : Theme.red
                           font.pixelSize: 19; font.weight: Font.Bold; font.letterSpacing: 1 }
                    Text { Layout.alignment: Qt.AlignHCenter
                           text: root.estop ? "STOPPED · click to release" : "cut torque + disarm"
                           color: root.estop ? "#fff" : Theme.red; font.pixelSize: 10; opacity: 0.9 }
                }
                MouseArea { anchors.fill: parent; cursorShape: Qt.PointingHandCursor
                            onClicked: controller.toggleStop() }
            }
        }

        // policy hint
        Text {
            Layout.fillWidth: true; Layout.leftMargin: 20; Layout.rightMargin: 20
            wrapMode: Text.WordWrap
            text: "Arming is a human action — the MCP client can only disarm, never arm. "
                + "Read tools (get_state / snapshot) work anytime; moves refuse until armed."
            color: Theme.text3; font.pixelSize: 11
        }

        // ── camera: live view + device picker + snapshot source ───
        Rectangle {
            Layout.fillWidth: true
            Layout.leftMargin: 20; Layout.rightMargin: 20; Layout.topMargin: 14
            implicitHeight: camCol.implicitHeight + 24
            radius: Theme.rLg; color: Theme.panel2; border.color: Theme.line

            ColumnLayout {
                id: camCol
                anchors.fill: parent; anchors.margins: 12; spacing: 10

                RowLayout {
                    Layout.fillWidth: true
                    Text { text: "Camera"; color: Theme.text2; font.pixelSize: 12; font.weight: Font.DemiBold }
                    Item { Layout.fillWidth: true }
                    Rectangle { Layout.preferredWidth: 7; Layout.preferredHeight: 7; radius: 3.5
                                Layout.alignment: Qt.AlignVCenter
                                color: root.camActive ? Theme.green : Theme.text3 }
                    Text { text: root.camActive ? "live" : "off"
                           color: Theme.text3; font.family: Theme.mono; font.pixelSize: 10 }
                }

                // live preview — polls image://cam as frames arrive (same idiom as the twin Image)
                Rectangle {
                    Layout.fillWidth: true; Layout.preferredHeight: 168
                    radius: 8; color: "#0b0d10"; border.color: Theme.line; clip: true
                    Image {
                        anchors.fill: parent; anchors.margins: 1
                        cache: false; fillMode: Image.PreserveAspectFit; smooth: true
                        visible: root.camActive && root.camFrame > 0
                        source: (root.camActive && root.camFrame > 0) ? ("image://cam/f" + root.camFrame) : ""
                    }
                    Text {
                        anchors.centerIn: parent; width: parent.width - 24
                        horizontalAlignment: Text.AlignHCenter; wrapMode: Text.WordWrap
                        visible: !(root.camActive && root.camFrame > 0)
                        text: root.camError !== "" ? root.camError
                            : root.camActive ? "warming up…"
                            : "camera off — pick a device and press Start"
                        color: root.camError !== "" ? Theme.red : Theme.text3
                        font.family: Theme.mono; font.pixelSize: 11
                    }
                }

                // ArUco detections readout — markers feed the MCP detect_markers tool + the overlay
                RowLayout {
                    Layout.fillWidth: true; spacing: 8
                    visible: root.camActive
                    Rectangle { Layout.preferredWidth: 7; Layout.preferredHeight: 7; radius: 3.5
                                Layout.alignment: Qt.AlignVCenter
                                color: root.markerCount > 0 ? Theme.green : Theme.text3 }
                    Text { text: root.markerCount + (root.markerCount === 1 ? " marker detected" : " markers detected")
                           color: Theme.text2; font.family: Theme.mono; font.pixelSize: 11 }
                    Item { Layout.fillWidth: true }
                }

                // device picker + start/stop
                RowLayout {
                    Layout.fillWidth: true; spacing: 8
                    ComboBox {
                        id: devBox
                        Layout.fillWidth: true
                        enabled: root.camDevices.length > 0
                        model: root.camDevices
                        textRole: "name"
                    }
                    Rectangle {
                        Layout.preferredWidth: 92; Layout.preferredHeight: 34; radius: 8
                        color: root.camActive ? Theme.soft(Theme.red, 0.14) : Theme.soft(Theme.green, 0.16)
                        border.color: root.camActive ? Theme.red : Theme.green
                        Text { anchors.centerIn: parent
                               text: root.camActive ? "Stop" : "Start"
                               color: root.camActive ? Theme.red : Theme.green
                               font.pixelSize: 12; font.weight: Font.DemiBold }
                        MouseArea {
                            anchors.fill: parent; cursorShape: Qt.PointingHandCursor
                            enabled: (typeof handlab !== "undefined" && handlab)
                                     && (root.camActive || root.camDevices.length > 0)
                            onClicked: {
                                if (root.camActive) { handlab.stopCamera() }
                                else if (devBox.currentIndex >= 0) {
                                    root.camError = ""
                                    handlab.startCamera(root.camDevices[devBox.currentIndex].id)
                                }
                            }
                        }
                    }
                }

                // snapshot source — what the MCP `snapshot` tool returns (twin vs live camera)
                RowLayout {
                    Layout.fillWidth: true; spacing: 8
                    Text { text: "snapshot:"; color: Theme.text3; font.pixelSize: 11
                           Layout.alignment: Qt.AlignVCenter }
                    Repeater {
                        model: [ { k: "twin", label: "Twin" }, { k: "cam", label: "Camera" } ]
                        Rectangle {
                            Layout.preferredWidth: 78; Layout.preferredHeight: 30; radius: 7
                            property bool sel: root.snapSource === modelData.k
                            color: sel ? Theme.soft(Theme.green, 0.16) : Theme.card
                            border.color: sel ? Theme.green : Theme.line
                            Text { anchors.centerIn: parent; text: modelData.label
                                   color: sel ? Theme.green : Theme.text2; font.pixelSize: 11 }
                            MouseArea { anchors.fill: parent; cursorShape: Qt.PointingHandCursor
                                enabled: (typeof handlab !== "undefined" && handlab)
                                onClicked: handlab.setSnapshotSource(modelData.k) }
                        }
                    }
                    Item { Layout.fillWidth: true }
                }
            }
        }
        // ── demo recorder (visuomotor-IL episodes) ────────────────
        // Same card as the Teleop tab (DemoRecorderCard.qml). Here Claude drives, not the operator,
        // so the teleop/tracker preconditions don't apply.
        DemoRecorderCard {
            controller: root.controller
            requireTeleop: false
            Layout.fillWidth: true
            Layout.leftMargin: 20; Layout.rightMargin: 20; Layout.topMargin: 14
        }

        // ── tool-call log ─────────────────────────────────────────
        Rectangle {
            Layout.fillWidth: true; Layout.fillHeight: true
            Layout.leftMargin: 20; Layout.rightMargin: 20; Layout.topMargin: 14; Layout.bottomMargin: 20
            radius: Theme.rLg; color: Theme.panel2; border.color: Theme.line

            ColumnLayout {
                anchors.fill: parent; anchors.margins: 12; spacing: 8
                RowLayout {
                    Layout.fillWidth: true
                    Text { text: "Tool-call log"; color: Theme.text2; font.pixelSize: 12; font.weight: Font.DemiBold }
                    Item { Layout.fillWidth: true }
                    Text { text: logModel.count + " calls"; color: Theme.text3; font.family: Theme.mono; font.pixelSize: 10 }
                }
                ListView {
                    id: logView
                    Layout.fillWidth: true; Layout.fillHeight: true
                    clip: true; model: logModel; spacing: 5
                    delegate: RowLayout {
                        width: ListView.view ? ListView.view.width : 0
                        spacing: 8
                        Rectangle { Layout.preferredWidth: 6; Layout.preferredHeight: 6; radius: 3
                                    Layout.alignment: Qt.AlignVCenter
                                    color: model.ok ? Theme.green : Theme.red }
                        Text { text: model.tool; color: Theme.text; font.family: Theme.mono
                               font.pixelSize: 11; font.weight: Font.DemiBold }
                        Text { text: model.detail; color: Theme.text2; font.family: Theme.mono; font.pixelSize: 11
                               Layout.fillWidth: true; elide: Text.ElideRight }
                    }
                }
            }
            // empty-state overlay (not in the layout, so it never fights ListView for height)
            Text {
                visible: logModel.count === 0
                anchors.centerIn: parent; width: parent.width - 40
                horizontalAlignment: Text.AlignHCenter; wrapMode: Text.WordWrap
                text: "no commands yet — connect Claude (MCP), then ARM to let it drive"
                color: Theme.text3; font.family: Theme.mono; font.pixelSize: 11
            }
        }
    }

    // the log itself: newest on top, capped so it never grows without bound
    ListModel { id: logModel }
    Connections {
        target: (typeof handlab !== "undefined") ? handlab : null
        function onAgentActivity(s) {
            try {
                var e = JSON.parse(s)
                logModel.insert(0, { tool: e.tool || "?", detail: e.detail || "", ok: e.ok !== false })
                if (logModel.count > 200) logModel.remove(200, logModel.count - 200)
            } catch (err) { /* ignore a malformed line rather than break the log */ }
        }
        function onCameraDevicesChanged() { root.camDevices = JSON.parse(handlab.cameraDevices) }
        function onCameraError(s) { root.camError = s }
    }
}
