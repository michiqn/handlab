import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import "."

// Screen 2 — one card per mounted finger (Repeater over the model).
Item {
    id: root
    property var controller

    ColumnLayout {
        anchors.fill: parent
        spacing: 0

        // header
        Rectangle {
            Layout.fillWidth: true; implicitHeight: 66; color: "transparent"
            Rectangle { anchors.bottom: parent.bottom; width: parent.width; height: 1; color: Theme.line }
            RowLayout {
                anchors.fill: parent; anchors.leftMargin: 20; anchors.rightMargin: 20; spacing: 10
                ColumnLayout {
                    spacing: 3
                    Text { text: "Control"; color: Theme.text; font.pixelSize: 17; font.weight: Font.DemiBold }
                    Text { text: controller.fingers.length + " finger(s) · " + controller.motorsUsed() + " servos live"
                           color: Theme.text2; font.pixelSize: 12 }
                }
                Item { Layout.fillWidth: true }
                // limp servos take no goals — both actions are meaningless while e-stopped
                ActionButton { text: "Return home"; accent: false; active: !controller.eStopped; onClicked: controller.returnHomeAll() }
                ActionButton {
                    // toggles meaning with reality: re-engaging after an e-stop holds the
                    // current pose (never jumps), so "all on" is always safe
                    readonly property bool anyOn: controller.anyTorqueOn()
                    text: anyOn ? "Torque off all" : "Torque all on"
                    accent: anyOn
                    active: !controller.eStopped
                    onClicked: anyOn ? controller.torqueOffAll() : controller.torqueOnAll()
                }
            }
        }

        // body
        ScrollView {
            Layout.fillWidth: true; Layout.fillHeight: true
            contentWidth: availableWidth; clip: true

            ColumnLayout {
                width: root.width
                spacing: 0

                // empty state
                ColumnLayout {
                    visible: controller.fingers.length === 0
                    Layout.fillWidth: true; Layout.topMargin: 60; spacing: 14
                    Rectangle {
                        Layout.alignment: Qt.AlignHCenter
                        width: 56; height: 56; radius: 16; color: "transparent"; border.width: 1.5; border.color: Theme.line
                        Rectangle { anchors.centerIn: parent; width: 20; height: 20; radius: 10; color: "transparent"; border.width: 2; border.color: Theme.text3 }
                    }
                    Text { Layout.alignment: Qt.AlignHCenter; text: "Nothing built yet"; color: Theme.text; font.pixelSize: 15; font.weight: Font.DemiBold }
                    Text { Layout.alignment: Qt.AlignHCenter; Layout.maximumWidth: 260; horizontalAlignment: Text.AlignHCenter; wrapMode: Text.WordWrap
                           text: "This build has no fingers. Edit builds/<name>.yaml (the servo mapping lives there) and relaunch."; color: Theme.text2; font.pixelSize: 13 }
                }

                // finger cards
                ColumnLayout {
                    Layout.fillWidth: true
                    Layout.margins: 20; spacing: 14
                    Repeater {
                        // display order = socket/port order (P1..P4) from the adapter → thumb,
                        // index, mid, ring for palm4. jogNorm routes by finger-id + dof-key (not
                        // ordinal), so reordering the display is servo-safe. Sort a COPY.
                        model: {
                            var list = (controller.fingers || []).slice()
                            list.sort(function(a, b) {
                                return controller.socketIndexOf(a.socket) - controller.socketIndexOf(b.socket)
                            })
                            return list
                        }
                        delegate: FingerControlCard {
                            required property var modelData
                            controller: root.controller
                            finger: modelData
                            Layout.fillWidth: true
                        }
                    }
                    // poses & recordings
                    PosesPanel { visible: controller.fingers.length > 0; controller: root.controller; Layout.fillWidth: true }
                }
            }
        }
    }

}
