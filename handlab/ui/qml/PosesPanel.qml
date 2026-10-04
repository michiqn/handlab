import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import "."

// Poses & Recordings (Control tab). Capture the current pose, or record a motion
// (move sliders, or backdrive the real hand with torque off) and replay it.
Rectangle {
    id: root
    property var controller
    readonly property bool live: typeof handlab !== "undefined" && handlab !== null
    readonly property var items: controller.poses

    property bool loop: false                 // replay clips in a loop

    // Local mirror of the saved items, so the ListView below can be reordered live by dragging.
    // Rebuilt from `items` on every change EXCEPT while a drag is in progress (`reordering`), so a
    // background posesChanged (playback/homing) can't clobber the drag. On drop we persist the new
    // order to the bridge, which re-emits posesChanged → items → rebuild to the same order.
    ListModel { id: poseModel }
    property bool reordering: false
    property bool didReorder: false
    function rebuildModel() {
        if (reordering) return
        poseModel.clear()
        for (var i = 0; i < items.length; i++) {
            var it = items[i]
            poseModel.append({ pName: "" + it.name, pKind: "" + (it.kind || "pose"),
                               pSecs: (it.secs !== undefined ? it.secs : 0) })
        }
    }
    onItemsChanged: rebuildModel()
    Component.onCompleted: rebuildModel()
    function persistOrder() {
        if (!live) return
        var names = []
        for (var i = 0; i < poseModel.count; i++) names.push(poseModel.get(i).pName)
        handlab.reorderPoses(JSON.stringify(names))
    }

    radius: Theme.rLg; color: Theme.card; border.color: Theme.line
    implicitHeight: col.implicitHeight + 28

    ColumnLayout {
        id: col
        anchors.left: parent.left; anchors.right: parent.right; anchors.top: parent.top
        anchors.margins: 16; spacing: 12

        // ── header + capture/record ───────────────────────────────
        RowLayout {
            Layout.fillWidth: true; spacing: 10
            ColumnLayout {
                spacing: 2
                Text { text: "Poses & Recordings"; color: Theme.text; font.pixelSize: 15; font.weight: Font.DemiBold }
                Text { text: root.controller.isRecording ? "recording — move the hand…"
                           : (root.controller.isPlaying ? "playing…" : items.length + " saved")
                       color: root.controller.isRecording ? Theme.red
                            : (root.controller.isPlaying ? Theme.accent : Theme.text3)
                       font.family: Theme.mono; font.pixelSize: 11
                       SequentialAnimation on opacity {
                           running: root.controller.isRecording; loops: Animation.Infinite
                           NumberAnimation { to: 0.3; duration: 500 }
                           NumberAnimation { to: 1.0; duration: 500 }
                       }
                }
            }
            Item { Layout.fillWidth: true }
            // loop toggle
            Rectangle {
                radius: 8; color: root.loop ? Theme.accentSoft : Theme.card2
                border.color: root.loop ? Theme.accent : Theme.line
                implicitWidth: lpT.implicitWidth + 22; implicitHeight: 30
                Text { id: lpT; anchors.centerIn: parent; text: "↻ loop"; color: root.loop ? Theme.accent : Theme.text3; font.family: Theme.mono; font.pixelSize: 11 }
                MouseArea { anchors.fill: parent; cursorShape: Qt.PointingHandCursor; onClicked: root.loop = !root.loop }
            }
            Rectangle {
                visible: root.controller.isPlaying
                radius: 8; color: Theme.soft(Theme.accent, 0.15); border.color: Theme.accent
                implicitWidth: spT.implicitWidth + 24; implicitHeight: 30
                Text { id: spT; anchors.centerIn: parent; text: "Stop"; color: Theme.accent; font.pixelSize: 12; font.weight: Font.DemiBold }
                MouseArea { anchors.fill: parent; cursorShape: Qt.PointingHandCursor; onClicked: if (root.live) handlab.stopPlayback() }
            }
        }

        // name + actions
        RowLayout {
            Layout.fillWidth: true; spacing: 8
            Rectangle {
                Layout.fillWidth: true; implicitHeight: 32; radius: 8
                color: Theme.panel2; border.color: Theme.line
                TextInput {
                    id: nameIn
                    anchors.fill: parent; anchors.leftMargin: 10; anchors.rightMargin: 10
                    verticalAlignment: TextInput.AlignVCenter
                    color: Theme.text; font.pixelSize: 12; clip: true
                    enabled: !root.controller.isRecording && !root.controller.isPlaying
                }
                Text { anchors.left: parent.left; anchors.leftMargin: 10; anchors.verticalCenter: parent.verticalCenter
                       visible: nameIn.text === ""; text: "name (optional)"; color: Theme.text3; font.pixelSize: 12 }
            }
            // Capture pose (snapshot)
            ActionButton {
                baseColor: Theme.card2; textColor: Theme.text2
                text: "＋ Pose"; active: !root.controller.isRecording && !root.controller.isPlaying && !root.controller.eStopped
                onClicked: { if (root.live) handlab.capturePose(nameIn.text); nameIn.text = "" }
            }
            // Record toggle
            ActionButton {
                baseColor: Theme.card2; textColor: Theme.text2
                text: root.controller.isRecording ? "■ Stop & save" : "● Record"
                accent: root.controller.isRecording
                active: !root.controller.isPlaying
                onClicked: {
                    if (!root.live) return
                    if (root.controller.isRecording) { handlab.stopRecording(nameIn.text); nameIn.text = "" }
                    else handlab.startRecording()
                }
            }
        }

        Rectangle { Layout.fillWidth: true; height: 1; color: Theme.line2; visible: items.length > 0 }

        // ── saved items ───────────────────────────────────────────
        Text {
            visible: items.length === 0
            Layout.fillWidth: true; wrapMode: Text.WordWrap; lineHeight: 1.3
            text: "Set a pose and ＋Pose, or ●Record and move the hand (real hand: torque off → shape it by hand). Click an entry to play it."
            color: Theme.text3; font.pixelSize: 12
        }
        // Drag the ⠿ handle to reorder. The dragged row lifts (reparents to the ListView) and
        // follows the cursor; each row's DropArea moves the model live as it passes; drop persists.
        ListView {
            id: poseList
            Layout.fillWidth: true
            Layout.preferredHeight: contentHeight
            visible: items.length > 0
            interactive: false
            spacing: 6
            model: poseModel

            displaced: Transition { NumberAnimation { property: "y"; duration: 140; easing.type: Easing.OutQuad } }

            delegate: Item {
                id: dele
                required property int index
                required property string pName
                required property string pKind
                required property real pSecs
                width: poseList.width
                height: 38
                readonly property bool held: gripMa.held
                readonly property bool isClip: pKind === "clip"

                DropArea {
                    anchors.fill: parent
                    onEntered: function(drag) {
                        var from = drag.source.index
                        if (from >= 0 && from !== dele.index) { poseModel.move(from, dele.index, 1); root.didReorder = true }
                    }
                }

                Rectangle {
                    id: content
                    width: dele.width; height: 38; radius: 8
                    anchors.horizontalCenter: parent.horizontalCenter
                    anchors.verticalCenter: parent.verticalCenter
                    color: (dele.held || rowHov.hovered) ? Theme.card2 : Theme.panel2
                    border.color: dele.held ? Theme.accent : Theme.line
                    Drag.active: dele.held
                    Drag.source: dele
                    Drag.hotSpot.x: width / 2
                    Drag.hotSpot.y: height / 2
                    HoverHandler { id: rowHov }

                    RowLayout {
                        anchors.fill: parent; anchors.leftMargin: 8; anchors.rightMargin: 8; spacing: 8

                        // drag handle
                        Item {
                            Layout.preferredWidth: 16; Layout.fillHeight: true
                            Text { anchors.centerIn: parent; text: "⠿"; color: dele.held ? Theme.accent : Theme.text3; font.pixelSize: 13 }
                            MouseArea {
                                id: gripMa
                                anchors.fill: parent
                                property bool held: false
                                enabled: !root.controller.isRecording && !root.controller.isPlaying
                                cursorShape: enabled ? Qt.SizeVerCursor : Qt.ArrowCursor
                                drag.target: held ? content : undefined
                                drag.axis: Drag.YAxis
                                onPressed: { root.reordering = true; held = true }
                                onReleased: { held = false; root.reordering = false
                                              if (root.didReorder) root.persistOrder(); root.didReorder = false }
                                onCanceled: { held = false; root.reordering = false
                                              if (root.didReorder) root.persistOrder(); root.didReorder = false }
                            }
                        }

                        Text { text: dele.isClip ? "↻" : "◉"; color: dele.isClip ? Theme.accent : Theme.green; font.pixelSize: 13 }

                        // name — click to rename (Enter commits, Esc cancels)
                        Item {
                            Layout.fillWidth: true; implicitHeight: 18
                            Text {
                                id: nameT; visible: !nameEdit.visible
                                anchors.left: parent.left; anchors.right: parent.right; anchors.verticalCenter: parent.verticalCenter
                                text: dele.pName; color: Theme.text; font.pixelSize: 13; elide: Text.ElideRight
                                HoverHandler { id: nmHov; cursorShape: Qt.IBeamCursor }
                                Rectangle { visible: nmHov.hovered; anchors.top: parent.bottom; width: Math.min(parent.width, nameT.contentWidth); height: 1; color: Theme.text3; opacity: 0.6 }
                                TapHandler {
                                    enabled: !root.controller.isRecording && !root.controller.isPlaying
                                    onTapped: { nameEdit.text = dele.pName; nameEdit.visible = true; nameEdit.forceActiveFocus(); nameEdit.selectAll() }
                                }
                            }
                            TextInput {
                                id: nameEdit; visible: false
                                anchors.left: parent.left; anchors.right: parent.right; anchors.verticalCenter: parent.verticalCenter
                                color: Theme.accent; font.pixelSize: 13; clip: true
                                function commit() {
                                    if (!visible) return
                                    var v = text.trim()
                                    if (root.live && v !== "" && v !== dele.pName) handlab.renamePose(dele.pName, v)
                                    visible = false
                                }
                                onAccepted: { commit(); focus = false }
                                onActiveFocusChanged: if (!activeFocus) commit()
                                Keys.onEscapePressed: { visible = false; focus = false }
                            }
                        }
                        Text { visible: dele.isClip; text: dele.pSecs + "s"; color: Theme.text3; font.family: Theme.mono; font.pixelSize: 10 }
                        Rectangle {
                            radius: 7; color: Theme.soft(Theme.accent, 0.15); border.color: Theme.accent
                            Layout.alignment: Qt.AlignVCenter
                            opacity: root.controller.eStopped || root.controller.isPlaying || root.controller.isRecording ? 0.4 : 1
                            implicitWidth: goT.implicitWidth + 18; implicitHeight: 24
                            Text { id: goT; anchors.centerIn: parent; text: dele.isClip ? "▶ play" : "▶ go"; color: Theme.accent; font.pixelSize: 11; font.weight: Font.DemiBold }
                            MouseArea {
                                anchors.fill: parent
                                enabled: !root.controller.eStopped && !root.controller.isPlaying && !root.controller.isRecording
                                cursorShape: enabled ? Qt.PointingHandCursor : Qt.ArrowCursor
                                onClicked: { if (!root.live) return; if (dele.isClip) handlab.playClip(dele.pName, root.loop); else handlab.applyPose(dele.pName) }
                            }
                        }
                        Rectangle {
                            width: 22; height: 22; radius: 6; color: "transparent"; border.color: Theme.line
                            Layout.alignment: Qt.AlignVCenter
                            Text { anchors.centerIn: parent; text: "×"; color: Theme.text3; font.pixelSize: 13 }
                            MouseArea { anchors.fill: parent; cursorShape: Qt.PointingHandCursor; onClicked: if (root.live) handlab.deletePose(dele.pName) }
                        }
                    }

                    states: State {
                        when: dele.held
                        ParentChange { target: content; parent: poseList }
                        AnchorChanges { target: content; anchors.horizontalCenter: undefined; anchors.verticalCenter: undefined }
                    }
                }
            }
        }
    }

}
