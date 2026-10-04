import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import "."

// Screen 4 - WiLoR hand-tracking teleop: hold YOUR hand in front of a camera and the claw mirrors
// it. The hand tracking runs OUTSIDE handlab (tracking/wilor_teleop.py, a separate process) and
// streams per-finger targets over UDP; handlab maps them onto the claw's calibrated range and
// applies all the safety. Motion is applied directly in Python (the same clamp + torque gate as the
// sliders); teleopFrame only syncs the display. Teleop is default OFF (a conscious enable); it needs
// no ARM (that gates Claude, not you) but nothing reaches the wire unless torque is on.
// The workspace camera below is what you WATCH while teleoperating (robot + object; recorder/ArUco).
Item {
    id: root
    property var controller

    readonly property bool teleopOn:    (typeof handlab !== "undefined" && handlab) ? handlab.teleopEnabled : false
    // workspace (robot) camera - the SAME controller/provider the Agent tab drives (image://cam)
    readonly property bool robotActive: (typeof handlab !== "undefined" && handlab) ? handlab.cameraActive : false
    readonly property int  robotFrame:  (typeof handlab !== "undefined" && handlab) ? handlab.camFrame : 0
    property var robotCamDevices: (typeof handlab !== "undefined" && handlab) ? JSON.parse(handlab.cameraDevices) : []
    readonly property int  markerCount: (typeof handlab !== "undefined" && handlab) ? handlab.markerCount : 0
    readonly property bool estop:       controller.eStopped
    readonly property string robotCamId: (typeof handlab !== "undefined" && handlab) ? handlab.cameraDeviceId : ""
    property string camError: ""

    readonly property string trackerCmd: "~/.venvs/wilor/bin/python tracking/wilor_teleop.py --show"

    function idxOf(devs, id) {
        for (var i = 0; i < devs.length; i++) if (devs[i].id === id) return i
        for (var j = 0; j < devs.length; j++) if (devs[j].default) return j
        return devs.length > 0 ? 0 : -1
    }
    function syncPickers() {
        if (robotCamId !== "") robotDevBox.currentIndex = idxOf(robotCamDevices, robotCamId)
    }
    Component.onCompleted: syncPickers()

    Connections {
        target: (typeof handlab !== "undefined") ? handlab : null
        ignoreUnknownSignals: true
        function onCameraError(msg) { root.camError = msg }
        function onCameraStateChanged() { if (root.robotCamId !== "") robotDevBox.currentIndex = root.idxOf(root.robotCamDevices, root.robotCamId) }
    }

    // hidden helper for the "copy command" button (QML has no direct clipboard API)
    TextEdit { id: clip; visible: false }

    ColumnLayout {
        anchors.fill: parent
        spacing: 0

        // -- header (fixed) --------------------------------------------
        Rectangle {
            Layout.fillWidth: true; implicitHeight: 66; color: "transparent"
            Rectangle { anchors.bottom: parent.bottom; width: parent.width; height: 1; color: Theme.line }
            RowLayout {
                anchors.fill: parent; anchors.leftMargin: 20; anchors.rightMargin: 20; spacing: 10
                ColumnLayout {
                    spacing: 3
                    Text { text: "Teleop"; color: Theme.text; font.pixelSize: 17; font.weight: Font.DemiBold }
                    Text { text: "your hand drives the claw via WiLoR"
                           color: Theme.text2; font.pixelSize: 12 }
                }
                Item { Layout.fillWidth: true }
                // teleop on/off chip
                Rectangle {
                    radius: 8; implicitHeight: 26; implicitWidth: seenRow.implicitWidth + 20
                    color: Theme.panel2; border.color: Theme.line
                    Row {
                        id: seenRow; anchors.centerIn: parent; spacing: 7
                        Rectangle { width: 7; height: 7; radius: 3.5; anchors.verticalCenter: parent.verticalCenter
                                    color: root.teleopOn ? Theme.green : Theme.text3 }
                        Text { text: root.teleopOn ? "teleop on" : "teleop off"
                               color: Theme.text2; font.family: Theme.mono; font.pixelSize: 11 }
                    }
                }
            }
        }

        // -- scrollable content ----------------------------------------
        Flickable {
            Layout.fillWidth: true
            Layout.fillHeight: true
            contentHeight: contentCol.implicitHeight + 20
            clip: true
            boundsBehavior: Flickable.StopAtBounds
            ScrollBar.vertical: ScrollBar {}

            ColumnLayout {
                id: contentCol
                width: parent.width
                spacing: 0

                // -- ENABLE + STOP ---------------------------------
                RowLayout {
                    Layout.fillWidth: true
                    Layout.leftMargin: 20; Layout.rightMargin: 20; Layout.topMargin: 16
                    spacing: 12

                    Rectangle {
                        Layout.fillWidth: true; implicitHeight: 86; radius: Theme.rLg
                        color: root.teleopOn ? Theme.soft(Theme.green, 0.16) : Theme.card
                        border.color: root.teleopOn ? Theme.green : Theme.line; border.width: 1
                        opacity: root.estop ? 0.5 : 1.0
                        ColumnLayout {
                            anchors.centerIn: parent; spacing: 4
                            Text { Layout.alignment: Qt.AlignHCenter
                                   text: root.teleopOn ? "TELEOP ON" : "TELEOP OFF"
                                   color: root.teleopOn ? Theme.green : Theme.text2
                                   font.pixelSize: 20; font.weight: Font.DemiBold }
                            Text { Layout.alignment: Qt.AlignHCenter
                                   text: root.teleopOn ? "your hand is driving  torque must be on to move"
                                                       : "click to enable  receives the WiLoR stream"
                                   color: Theme.text3; font.pixelSize: 10 }
                        }
                        MouseArea {
                            anchors.fill: parent
                            cursorShape: root.estop ? Qt.ArrowCursor : Qt.PointingHandCursor
                            enabled: !root.estop && (typeof handlab !== "undefined" && handlab)
                            onClicked: {
                                if (root.teleopOn) { handlab.disableTeleop(); return }
                                root.camError = ""
                                JSON.parse(handlab.enableTeleop())
                            }
                        }
                    }

                    Rectangle {
                        Layout.preferredWidth: 155; implicitHeight: 86; radius: Theme.rLg
                        color: root.estop ? Theme.red : Theme.soft(Theme.red, 0.14)
                        border.color: Theme.red; border.width: root.estop ? 0 : 1
                        ColumnLayout {
                            anchors.centerIn: parent; spacing: 4
                            Text { Layout.alignment: Qt.AlignHCenter; text: "STOP"
                                   color: root.estop ? "#fff" : Theme.red
                                   font.pixelSize: 20; font.weight: Font.DemiBold }
                            Text { Layout.alignment: Qt.AlignHCenter
                                   text: root.estop ? "stopped  click to release" : "stop teleop + cut torque"
                                   color: root.estop ? "#fff" : Theme.red; font.pixelSize: 10; opacity: 0.9 }
                        }
                        MouseArea { anchors.fill: parent; cursorShape: Qt.PointingHandCursor
                                    onClicked: controller.toggleStop() }
                    }
                }

                // -- torque + home (the same slots the Control tab uses) ----
                // Teleop needs torque ON to reach the wire, and you want to park the hand without
                // switching tabs mid-session, so both live here too rather than one tab away.
                RowLayout {
                    Layout.fillWidth: true
                    Layout.leftMargin: 20; Layout.rightMargin: 20; Layout.topMargin: 10
                    spacing: 10
                    ActionButton {
                        text: "Return home"
                        active: !controller.eStopped
                        onClicked: controller.returnHomeAll()
                    }
                    ActionButton {
                        // toggles meaning with reality: re-engaging after an e-stop holds the
                        // current pose (never jumps), so "all on" is always safe
                        readonly property bool anyOn: controller.anyTorqueOn()
                        text: anyOn ? "Torque off all" : "Torque all on"
                        accent: anyOn
                        active: !controller.eStopped
                        onClicked: anyOn ? controller.torqueOffAll() : controller.torqueOnAll()
                    }
                    Item { Layout.fillWidth: true }
                    Text {
                        text: root.teleopOn && !controller.anyTorqueOn() ? "torque is off — nothing moves" : ""
                        color: Theme.amber; font.pixelSize: 10
                        Layout.alignment: Qt.AlignVCenter
                    }
                }

                // -- demo recorder (visuomotor-IL episodes) ----------------
                // Here, not one tab away: a 50-episode session is record → grasp → save → repeat,
                // and you drive it from this tab. Same component as the Agent tab.
                DemoRecorderCard {
                    controller: root.controller
                    Layout.fillWidth: true
                    Layout.leftMargin: 20; Layout.rightMargin: 20; Layout.topMargin: 14
                }

                // -- run the tracker (the human side runs outside handlab) --
                Rectangle {
                    Layout.fillWidth: true
                    Layout.leftMargin: 20; Layout.rightMargin: 20; Layout.topMargin: 14
                    implicitHeight: trkCol.implicitHeight + 24
                    radius: Theme.rLg; color: Theme.panel2; border.color: Theme.line

                    ColumnLayout {
                        id: trkCol
                        anchors.fill: parent; anchors.margins: 12; spacing: 10

                        Text { text: "Run the tracker"; color: Theme.text2
                               font.pixelSize: 12; font.weight: Font.DemiBold }
                        Text {
                            Layout.fillWidth: true; wrapMode: Text.WordWrap
                            text: "The hand tracking runs in its own process. Enable teleop above, turn "
                                + "torque on, then start the tracker in a terminal. Calibrate to your hand "
                                + "in its window (it must have focus): o = open, f = fist, p = splayed, "
                                + "v = ✌️ peace, t = 🤏 pinch. The thumb rides the saved pinch*/peace* "
                                + "poses — re-capture those in Control after a re-home."
                            color: Theme.text3; font.pixelSize: 11
                        }

                        // the command + a copy button
                        RowLayout {
                            Layout.fillWidth: true; spacing: 8
                            Rectangle {
                                Layout.fillWidth: true; implicitHeight: 38; radius: 8
                                color: Theme.card; border.color: Theme.line
                                Text {
                                    anchors.fill: parent; anchors.leftMargin: 12; anchors.rightMargin: 12
                                    verticalAlignment: Text.AlignVCenter; elide: Text.ElideRight
                                    text: root.trackerCmd; color: Theme.text2
                                    font.family: Theme.mono; font.pixelSize: 11
                                }
                            }
                            Rectangle {
                                Layout.preferredWidth: 74; Layout.preferredHeight: 38; radius: 8
                                color: Theme.soft(Theme.accent, 0.14); border.color: Theme.accent
                                Text { id: copyLabel; anchors.centerIn: parent; text: "Copy"
                                       color: Theme.accent; font.pixelSize: 12; font.weight: Font.DemiBold }
                                MouseArea {
                                    anchors.fill: parent; cursorShape: Qt.PointingHandCursor
                                    onClicked: {
                                        clip.text = root.trackerCmd
                                        clip.selectAll(); clip.copy()
                                        copyLabel.text = "Copied"; copyReset.restart()
                                    }
                                }
                                Timer { id: copyReset; interval: 1200; onTriggered: copyLabel.text = "Copy" }
                            }
                        }

                        // camera hint — the tracker grabs camera 0 by default
                        Text {
                            Layout.fillWidth: true; wrapMode: Text.WordWrap
                            text: "Grabbing the wrong camera? Add  --camera N  to the command (e.g. --camera 1)."
                            color: Theme.text3; font.pixelSize: 11
                        }
                    }
                }

                // -- workspace cam (robot + object - what you WATCH; recorder/ArUco) --
                Rectangle {
                    Layout.fillWidth: true
                    Layout.leftMargin: 20; Layout.rightMargin: 20; Layout.topMargin: 14
                    implicitHeight: wsCol.implicitHeight + 24
                    radius: Theme.rLg; color: Theme.panel2; border.color: Theme.line

                    ColumnLayout {
                        id: wsCol
                        anchors.fill: parent; anchors.margins: 12; spacing: 10

                        RowLayout {
                            Layout.fillWidth: true
                            Text { text: "Workspace camera"; color: Theme.text2
                                   font.pixelSize: 12; font.weight: Font.DemiBold }
                            Item { Layout.fillWidth: true }
                            Rectangle { Layout.preferredWidth: 7; Layout.preferredHeight: 7; radius: 3.5
                                        Layout.alignment: Qt.AlignVCenter
                                        color: root.robotActive ? Theme.green : Theme.text3 }
                            Text { text: root.robotActive ? "live" : "off"
                                   color: Theme.text3; font.family: Theme.mono; font.pixelSize: 10 }
                        }

                        RowLayout {
                            Layout.fillWidth: true; spacing: 8
                            Text { text: "device"; color: Theme.text3; font.pixelSize: 11 }
                            ComboBox {
                                id: robotDevBox
                                Layout.fillWidth: true; Layout.preferredHeight: 32
                                enabled: root.robotCamDevices.length > 0
                                model: root.robotCamDevices
                                textRole: "name"
                            }
                            Rectangle {
                                Layout.preferredWidth: 76; Layout.preferredHeight: 32; radius: 8
                                color: root.robotActive ? Theme.soft(Theme.red, 0.14) : Theme.soft(Theme.green, 0.16)
                                border.color: root.robotActive ? Theme.red : Theme.green
                                Text { anchors.centerIn: parent
                                       text: root.robotActive ? "Stop" : "Start"
                                       color: root.robotActive ? Theme.red : Theme.green
                                       font.pixelSize: 12; font.weight: Font.DemiBold }
                                MouseArea {
                                    anchors.fill: parent; cursorShape: Qt.PointingHandCursor
                                    enabled: (typeof handlab !== "undefined" && handlab)
                                             && (root.robotActive || root.robotCamDevices.length > 0)
                                    onClicked: {
                                        if (root.robotActive) { handlab.stopCamera() }
                                        else if (robotDevBox.currentIndex >= 0) {
                                            root.camError = ""
                                            handlab.startCamera(root.robotCamDevices[robotDevBox.currentIndex].id)
                                        }
                                    }
                                }
                            }
                        }

                        Rectangle {
                            Layout.fillWidth: true; Layout.preferredHeight: 190
                            radius: 8; color: "#0b0d10"; border.color: Theme.line; clip: true
                            Image {
                                anchors.fill: parent; anchors.margins: 1
                                cache: false; fillMode: Image.PreserveAspectFit; smooth: true
                                visible: root.robotActive && root.robotFrame > 0
                                source: (root.robotActive && root.robotFrame > 0) ? ("image://cam/f" + root.robotFrame) : ""
                            }
                            Text {
                                anchors.centerIn: parent; width: parent.width - 24
                                horizontalAlignment: Text.AlignHCenter; wrapMode: Text.WordWrap
                                visible: !(root.robotActive && root.robotFrame > 0)
                                text: root.camError !== "" ? root.camError
                                    : root.robotActive ? "warming up..."
                                    : "pick the workspace camera and press Start  feeds the recorder + ArUco"
                                color: root.camError !== "" ? Theme.red : Theme.text3
                                font.family: Theme.mono; font.pixelSize: 11
                            }
                        }

                        RowLayout {
                            Layout.fillWidth: true; spacing: 8
                            visible: root.robotActive
                            Rectangle { Layout.preferredWidth: 7; Layout.preferredHeight: 7; radius: 3.5
                                        Layout.alignment: Qt.AlignVCenter
                                        color: root.markerCount > 0 ? Theme.green : Theme.text3 }
                            Text { text: root.markerCount + (root.markerCount === 1 ? " marker detected" : " markers detected")
                                   color: Theme.text2; font.family: Theme.mono; font.pixelSize: 11 }
                            Item { Layout.fillWidth: true }
                        }
                    }
                }
            }
        }
    }
}
