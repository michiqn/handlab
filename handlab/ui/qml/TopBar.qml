import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import "."

// Top bar: product mark · connection chip · Control/Teleop tabs · theme · STOP
// (Agent lives in a floating bubble, not here — it's a deliberate, occasional action.)
Rectangle {
    id: root
    property var controller
    implicitHeight: 58
    color: Theme.panel
    Behavior on color { ColorAnimation { duration: 180 } }

    Rectangle { anchors.bottom: parent.bottom; width: parent.width; height: 1; color: Theme.line }

    readonly property bool stopped: controller.eStopped

    RowLayout {
        anchors.fill: parent
        anchors.leftMargin: 16
        anchors.rightMargin: 16
        spacing: 16

        // ── product mark ──────────────────────────────────────────
        RowLayout {
            Layout.alignment: Qt.AlignVCenter
            spacing: 10
            Image {
                source: "../assets/logo_mark.png"              // circular badge, transparent corners
                sourceSize.width: 64; sourceSize.height: 64    // decode small for crisp downscale
                Layout.preferredWidth: 32; Layout.preferredHeight: 32   // layout honors these (not width/height)
                Layout.alignment: Qt.AlignVCenter
                fillMode: Image.PreserveAspectFit
                smooth: true; mipmap: true; antialiasing: true
            }
            ColumnLayout {
                spacing: 2
                Text { text: "handlab"; color: Theme.text; font.pixelSize: 16; font.weight: Font.Bold; font.letterSpacing: -0.3 }
                Text { text: "CONTROL CENTER"; color: Theme.text3; font.family: Theme.mono; font.pixelSize: 9; font.letterSpacing: 1.5 }
            }
        }

        // ── status chip — honest: driver, real counts, real fps ──
        // Click opens the hardware connection panel (port scan / connect / zero cal).
        Rectangle {
            id: statusChip
            height: 30; radius: 15
            color: chipHov.hovered ? Theme.card2 : Theme.card
            border.color: chipHov.hovered ? Theme.accent : Theme.line
            implicitWidth: connRow.implicitWidth + 24
            readonly property bool simLive: controller.twinFps > 0
            HoverHandler { id: chipHov; cursorShape: Qt.PointingHandCursor }
            TapHandler { onTapped: controller.openConnectPanel() }
            RowLayout {
                id: connRow
                anchors.centerIn: parent
                spacing: 7
                Rectangle {
                    width: 7; height: 7; radius: 3.5
                    color: root.stopped ? Theme.red : (statusChip.simLive ? Theme.green : Theme.amber)
                }
                Text { text: controller.driverName; color: Theme.text2; font.family: Theme.mono; font.pixelSize: 12 }
                Text { text: "·"; color: Theme.text3; font.family: Theme.mono; font.pixelSize: 12 }
                Text { text: "" + controller.motorsUsed(); color: Theme.text; font.family: Theme.mono; font.pixelSize: 12 }
                Text { text: "servos"; color: Theme.text3; font.family: Theme.mono; font.pixelSize: 12 }
                Text { text: "·"; color: Theme.text3; font.family: Theme.mono; font.pixelSize: 12 }
                Text { text: "" + controller.twinFps; color: Theme.text; font.family: Theme.mono; font.pixelSize: 12 }
                Text { text: "fps"; color: Theme.text3; font.family: Theme.mono; font.pixelSize: 12 }
            }
        }

        Item { Layout.fillWidth: true }

        // ── tabs ──────────────────────────────────────────────────
        Rectangle {
            radius: 10
            color: Theme.panel2
            border.color: Theme.line
            implicitWidth: tabRow.implicitWidth + 6
            implicitHeight: 36
            Row {
                id: tabRow
                anchors.centerIn: parent
                spacing: 3
                Repeater {
                    // labels map to explicit StackLayout indices (Agent = index 1, reached via the
                    // floating bubble, is intentionally omitted here)
                    model: [{ name: "Control", idx: 0 }, { name: "Teleop", idx: 2 }]
                    delegate: Rectangle {
                        required property var modelData
                        readonly property bool active: controller.tabIndex === modelData.idx
                        width: tlabel.implicitWidth + 32; height: 30; radius: 8
                        color: active ? Theme.accentSoft : "transparent"
                        Text { id: tlabel; anchors.centerIn: parent; text: modelData.name
                               color: active ? Theme.accent : Theme.text2; font.pixelSize: 13; font.weight: Font.DemiBold }
                        MouseArea { anchors.fill: parent; cursorShape: Qt.PointingHandCursor; onClicked: controller.tabIndex = modelData.idx }
                    }
                }
            }
        }

        Item { Layout.fillWidth: true }

        // ── theme toggle ──────────────────────────────────────────
        Rectangle {
            width: 34; height: 34; radius: 9
            color: Theme.card; border.color: Theme.line
            Rectangle {
                anchors.centerIn: parent
                width: 16; height: 16; radius: 8
                border.width: 1.5; border.color: Theme.text2; color: "transparent"
                Rectangle { width: 8; height: 16; color: Theme.text2; radius: 0
                            anchors.left: parent.left }  // half-fill "moon"
                clip: true
            }
            MouseArea { anchors.fill: parent; cursorShape: Qt.PointingHandCursor; onClicked: Theme.toggle() }
        }

        // ── STOP ──────────────────────────────────────────────────
        Rectangle {
            id: stopBtn
            implicitWidth: stopRow.implicitWidth + 32; height: 36; radius: 10
            color: root.stopped ? "#ffffff" : Theme.red
            border.width: root.stopped ? 2 : 0
            border.color: Theme.red
            RowLayout {
                id: stopRow
                anchors.centerIn: parent
                spacing: 9
                Rectangle { width: 11; height: 11; radius: 2; color: root.stopped ? Theme.red : "#ffffff" }
                Text { text: root.stopped ? "STOPPED" : "E-STOP"
                       color: root.stopped ? Theme.red : "#ffffff"
                       font.pixelSize: 13; font.weight: Font.Bold; font.letterSpacing: 1.5 }
            }
            SequentialAnimation on opacity {
                running: root.stopped; loops: Animation.Infinite
                NumberAnimation { to: 0.45; duration: 500 }
                NumberAnimation { to: 1.0;  duration: 500 }
            }
            MouseArea { anchors.fill: parent; cursorShape: Qt.PointingHandCursor; onClicked: controller.toggleStop() }
        }
    }
}
