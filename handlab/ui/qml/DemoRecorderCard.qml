import QtCore
import QtQuick
// Controls.Basic, not Controls: the native macOS style refuses `background` customization on
// TextField (it warned on every instance), and this card styles its own field anyway.
import QtQuick.Controls.Basic
import QtQuick.Layouts
import "."

// Demo Recorder — one recording session, usable from wherever you actually record.
//
// Lives in BOTH the Teleop tab (where you drive the hand for demos) and the Agent tab (where Claude
// does): a 50-episode session cannot mean switching tabs per episode. Observe-only — this card
// never commands motion, it starts/stops `handlab`'s recorder, which samples the existing telemetry
// beat (bridge.py `_tele_tick_body` -> `_demo_step`).
//
// The readiness list is the point: an episode recorded with torque off, or with the tracker not
// running, looks identical while recording and is worthless afterwards. Better to be told than to
// find out from tools/inspect_episode.py after an afternoon.
Rectangle {
    id: root
    property var controller
    // Teleop tab: the operator drives, so teleop + a live tracker stream are preconditions.
    // Agent tab: Claude drives instead — those two rows don't apply.
    property bool requireTeleop: true

    readonly property bool bridgeOk: (typeof handlab !== "undefined" && handlab)
    readonly property bool recording: bridgeOk ? handlab.demoRecording : false
    readonly property int episodes:   bridgeOk ? handlab.demoEpisodeCount : 0
    readonly property string lastEpisode: bridgeOk ? handlab.lastEpisodePath : ""
    readonly property bool camOn:     bridgeOk ? handlab.cameraActive : false
    readonly property bool teleopOn:  bridgeOk ? handlab.teleopEnabled : false
    readonly property bool streamOk:  bridgeOk ? handlab.teleopStreamLive : false
    readonly property bool hwOn:      controller.driverName === "Dynamixel"
    readonly property bool torqueOn:  controller.anyTorqueOn()
    readonly property string task: taskField.text.trim()

    // every precondition, each bound to a real signal — `ok` false is what blocks Record
    readonly property var checks: {
        var c = [{ ok: camOn,    label: "Workspace camera", bad: "camera off — no observation to record" },
                 { ok: hwOn,     label: "Hardware connected", bad: "no hardware — the twin records nothing real" },
                 { ok: torqueOn, label: "Torque on", bad: "torque off — the hand will not move" }]
        if (requireTeleop) {
            c.push({ ok: teleopOn, label: "Teleop on", bad: "teleop off — you cannot drive a demo" })
            c.push({ ok: streamOk, label: "Tracker stream live", bad: "no tracker frames arriving" })
        }
        return c
    }
    readonly property bool ready: {
        for (var i = 0; i < checks.length; i++) if (!checks[i].ok) return false
        return bridgeOk && task.length > 0
    }

    // the task name survives restarts — you type it once per session, not once per episode
    Settings { id: demoStore; category: "DemoRecorder"; property string lastTask: "" }

    // elapsed time of the running episode: the cheapest way to keep episode lengths comparable
    property int elapsed: 0
    Timer {
        running: root.recording; repeat: true; interval: 1000
        onTriggered: root.elapsed += 1
    }
    onRecordingChanged: {
        if (recording) { elapsed = 0; demoStore.lastTask = task }
    }
    function mmss(s) {
        return (s < 600 ? "0" : "") + Math.floor(s / 60) + ":" + (s % 60 < 10 ? "0" : "") + (s % 60)
    }

    // ---- the actions, shared with Main.qml's keyboard shortcuts ----
    function startEpisode() { if (ready && !recording) handlab.startDemo(task) }
    function stopEpisode(outcome) { if (recording) handlab.stopDemo(outcome) }
    function discardLast() { if (bridgeOk && lastEpisode !== "" && !recording) handlab.discardLastDemo() }

    // The shortcuts in Main.qml drive whichever card is on screen; StackLayout flips `visible`
    // per tab, so registering here needs no tab bookkeeping.
    onVisibleChanged: if (visible) controller.demoCard = root
    Component.onCompleted: {                          // ONE handler — a second would replace this
        if (taskField.text === "")
            taskField.text = demoStore.lastTask
        if (visible)
            controller.demoCard = root
    }

    radius: Theme.rLg
    color: Theme.panel2
    border.color: recording ? Theme.red : Theme.line
    implicitHeight: col.implicitHeight + 24
    Behavior on border.color { ColorAnimation { duration: 150 } }

    ColumnLayout {
        id: col
        anchors.fill: parent; anchors.margins: 12; spacing: 8

        RowLayout {
            Layout.fillWidth: true; spacing: 8
            Text { text: "Demo Recorder"; color: Theme.text
                   font.pixelSize: 13; font.weight: Font.DemiBold }
            Rectangle {
                Layout.preferredWidth: 7; Layout.preferredHeight: 7; radius: 3.5
                Layout.alignment: Qt.AlignVCenter
                color: root.recording ? Theme.red : Theme.text3
                SequentialAnimation on opacity {          // a blink you notice from across the desk
                    running: root.recording; loops: Animation.Infinite
                    NumberAnimation { to: 0.25; duration: 550 }
                    NumberAnimation { to: 1.0;  duration: 550 }
                }
            }
            Text { visible: root.recording; text: "recording " + root.mmss(root.elapsed)
                   color: Theme.red; font.family: Theme.mono; font.pixelSize: 10 }
            Item { Layout.fillWidth: true }
            Text { text: root.episodes + " saved"; color: Theme.text3
                   font.family: Theme.mono; font.pixelSize: 10 }
        }

        Text {
            Layout.fillWidth: true; wrapMode: Text.WordWrap; lineHeight: 1.25
            text: "Records (cam frame · action · proprio · object-pose) at 10 Hz for imitation "
                + "learning. Observe-only — moves nothing."
            color: Theme.text2; font.pixelSize: 11
        }

        TextField {
            id: taskField
            Layout.fillWidth: true
            enabled: !root.recording
            placeholderText: "task name (e.g. grasp_prism)"
            color: Theme.text; font.pixelSize: 12
            background: Rectangle { radius: 7; color: Theme.card
                border.color: taskField.activeFocus ? Theme.accent : Theme.line }
        }

        // ── readiness ────────────────────────────────────────────
        // Shown while idle: this is what you check BEFORE the session, not during it.
        ColumnLayout {
            visible: !root.recording
            Layout.fillWidth: true; spacing: 3
            Repeater {
                model: root.checks
                delegate: RowLayout {
                    required property var modelData
                    Layout.fillWidth: true; spacing: 6
                    Text { text: modelData.ok ? "✓" : "✗"
                           color: modelData.ok ? Theme.green : Theme.red
                           font.family: Theme.mono; font.pixelSize: 11 }
                    Text {
                        Layout.fillWidth: true; elide: Text.ElideRight
                        text: modelData.ok ? modelData.label : modelData.bad
                        color: modelData.ok ? Theme.text3 : Theme.red
                        font.pixelSize: 10
                    }
                }
            }
        }

        RowLayout {
            Layout.fillWidth: true; spacing: 8
            Rectangle {                                  // idle: Record
                visible: !root.recording
                Layout.fillWidth: true; Layout.preferredHeight: 36; radius: 8
                color: root.ready ? Theme.soft(Theme.green, 0.16) : Theme.card
                border.color: root.ready ? Theme.green : Theme.line
                opacity: root.ready ? 1 : 0.5
                Text { anchors.centerIn: parent
                       text: root.task.length === 0 ? "● Record — name the task first" : "● Record  (space)"
                       color: root.ready ? Theme.green : Theme.text3
                       font.pixelSize: 12; font.weight: Font.DemiBold }
                MouseArea { anchors.fill: parent; enabled: root.ready
                    cursorShape: Qt.PointingHandCursor
                    onClicked: root.startEpisode() }
            }
            Rectangle {                                  // recording: save success
                visible: root.recording
                Layout.fillWidth: true; Layout.preferredHeight: 36; radius: 8
                color: Theme.soft(Theme.green, 0.16); border.color: Theme.green
                Text { anchors.centerIn: parent; text: "✓ save success  (space)"; color: Theme.green
                       font.pixelSize: 12; font.weight: Font.DemiBold }
                MouseArea { anchors.fill: parent; cursorShape: Qt.PointingHandCursor
                    enabled: root.bridgeOk
                    onClicked: root.stopEpisode("success") }
            }
            Rectangle {                                  // recording: save as fail
                visible: root.recording
                Layout.preferredWidth: 110; Layout.preferredHeight: 36; radius: 8
                color: Theme.soft(Theme.amber, 0.14); border.color: Theme.amber
                Text { anchors.centerIn: parent; text: "✗ fail  (f)"; color: Theme.amber
                       font.pixelSize: 12; font.weight: Font.DemiBold }
                MouseArea { anchors.fill: parent; cursorShape: Qt.PointingHandCursor
                    enabled: root.bridgeOk
                    onClicked: root.stopEpisode("fail") }
            }
        }

        RowLayout {
            Layout.fillWidth: true; spacing: 8
            Rectangle {
                Layout.preferredWidth: 150; Layout.preferredHeight: 30; radius: 7
                property bool can: root.lastEpisode !== "" && !root.recording && root.bridgeOk
                color: Theme.card; border.color: can ? Theme.line : Theme.line2
                opacity: can ? 1 : 0.5
                Text { anchors.centerIn: parent; text: "↩ discard last  (backspace)"
                       color: parent.can ? Theme.text2 : Theme.text3; font.pixelSize: 11 }
                MouseArea { anchors.fill: parent; enabled: parent.can
                    cursorShape: Qt.PointingHandCursor
                    onClicked: root.discardLast() }
            }
            Text { Layout.fillWidth: true; elide: Text.ElideLeft; text: root.lastEpisode
                   color: Theme.text3; font.family: Theme.mono; font.pixelSize: 9 }
        }
    }
}
