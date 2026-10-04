import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import "."

// A single mounted finger: status header, per-servo torque switches, and one
// labelled stepper (value field + −/+ · wheel · type) per DOF (set + units from the type model).
Rectangle {
    id: root
    property var controller
    property var finger

    readonly property var ftype: controller.typeById(finger.typeId)
    readonly property int socketIndex: controller.socketIndexOf(finger.socket)
    readonly property color idc: controller.idColorFor(socketIndex)
    readonly property var status: controller.fingerStatus(finger)

    radius: Theme.rLg
    color: Theme.card
    border.color: Theme.line
    implicitHeight: col.implicitHeight

    // Telemetry readouts update ~live, and a digit more (9mA -> 10mA, 999 -> 1000) used to widen
    // its chip and shove every neighbouring chip in the Flow sideways -- the row visibly twitched.
    // Reserve the width of the widest reading we expect instead, so the text changes but nothing
    // moves. Wider outliers still grow the chip (Math.max below) rather than being clipped.
    // advanceWidth, not width: that is the metric Text.implicitWidth uses (width = ink bounds,
    // ~1px narrower, which would let a 4-digit reading grow the slot after all).
    TextMetrics { id: maW; font.family: Theme.mono; font.pixelSize: 10; text: "1888mA" }
    TextMetrics { id: tempW; font.family: Theme.mono; font.pixelSize: 10; text: "188°" }

    // Compact − / + step button for the DOF stepper. Dumb: the delegate sets `armed` and reacts
    // to stepped(by). Click = ±amount°, Shift-click = ±5×, hold = auto-repeat.
    component StepButton: Rectangle {
        id: sb
        property int amount: 1
        property string glyph: "+"
        property bool armed: true
        property bool _big: false
        signal stepped(int by)
        implicitWidth: 24; implicitHeight: 22; radius: 6
        color: sbMa.pressed && armed ? Theme.line : Theme.card2
        border.color: armed ? Theme.line : Theme.line2
        opacity: armed ? 1 : 0.4
        Text { anchors.centerIn: parent; text: sb.glyph
               color: sbMa.pressed && armed ? Theme.text : Theme.text2
               font.family: Theme.mono; font.pixelSize: 15; font.weight: Font.DemiBold }
        MouseArea {
            id: sbMa; anchors.fill: parent; enabled: sb.armed
            cursorShape: Qt.PointingHandCursor
            onPressed: function(m) {
                sb._big = (m.modifiers & Qt.ShiftModifier) !== 0
                sb.stepped(sb._big ? sb.amount * 5 : sb.amount)   // first step immediately
                holdDelay.start()
            }
            onReleased: { holdDelay.stop(); holdRep.stop() }
            onCanceled: { holdDelay.stop(); holdRep.stop() }
        }
        Timer { id: holdDelay; interval: 350; onTriggered: holdRep.start() }   // hold delay
        Timer { id: holdRep; interval: 90; repeat: true
                onTriggered: sb.stepped(sb._big ? sb.amount * 5 : sb.amount) }
    }

    ColumnLayout {
        id: col
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.top: parent.top
        spacing: 0

        // ── header ────────────────────────────────────────────────
        Rectangle {
            Layout.fillWidth: true; implicitHeight: 54; color: "transparent"
            Rectangle { anchors.bottom: parent.bottom; width: parent.width; height: 1; color: Theme.line2 }
            RowLayout {
                anchors.fill: parent; anchors.leftMargin: 15; anchors.rightMargin: 15; spacing: 10
                Rectangle {
                    width: 28; height: 28; radius: 8
                    color: Theme.soft(root.idc, 0.16); border.color: root.idc
                    Text { anchors.centerIn: parent; text: controller.socketShort(socketIndex); color: root.idc
                           font.family: Theme.mono; font.pixelSize: 14; font.weight: Font.DemiBold }
                }
                ColumnLayout {
                    spacing: 1
                    Text { text: ftype ? ftype.name : ""; color: Theme.text; font.pixelSize: 14; font.weight: Font.DemiBold }
                    Text { text: controller.socketLabel(socketIndex) + " · " + (ftype ? ftype.motors : 0) + " motors"
                           color: Theme.text3; font.family: Theme.mono; font.pixelSize: 10 }
                }
                Item { Layout.fillWidth: true }
                Rectangle {
                    radius: 7; color: Theme.panel2
                    implicitWidth: stRow.implicitWidth + 18; implicitHeight: 24
                    Row {
                        id: stRow; anchors.centerIn: parent; spacing: 6
                        Rectangle {
                            width: 7; height: 7; radius: 3.5; color: root.status.color; anchors.verticalCenter: parent.verticalCenter
                            SequentialAnimation on opacity {
                                running: root.status.label === "moving"; loops: Animation.Infinite
                                NumberAnimation { to: 0.2; duration: 450 }
                                NumberAnimation { to: 1.0; duration: 450 }
                            }
                        }
                        Text { text: root.status.label; color: root.status.color; font.family: Theme.mono; font.pixelSize: 11 }
                    }
                }
            }
        }

        // ── servo chips (per-servo torque Switch) ─────────────────
        Flow {
            Layout.fillWidth: true; Layout.leftMargin: 15; Layout.rightMargin: 15; Layout.topMargin: 12
            spacing: 7
            Repeater {
                model: root.finger.servos
                delegate: Rectangle {
                    id: chipCard
                    required property var modelData
                    readonly property string st: controller.statusOf(root.finger.uid, modelData.id)
                    readonly property var tele: controller.telemetry[modelData.id]
                    // latched hardware error (overload/overheat) from telemetry — servo ignores
                    // goals until rebooted (Connect panel), then set home for its finger
                    readonly property int hwErr: (tele !== undefined && tele.err !== undefined) ? tele.err : 0
                    // live Dynamixel telemetry: grip current (|mA|) + temperature (°C), per motor
                    readonly property int curMa: (tele !== undefined && tele.cur !== undefined) ? Math.abs(tele.cur) : 0
                    readonly property int tempC: (tele !== undefined && tele.temp !== undefined) ? tele.temp : 0
                    readonly property bool showTelem: controller.driverName === "Dynamixel" && st !== "offline" && tele !== undefined
                    readonly property bool on: controller.isTorque(root.finger.uid, modelData.id) && !controller.eStopped && st !== "offline"
                    radius: 8; color: Theme.panel2
                    border.color: hwErr ? Theme.red : (on ? Theme.line : Theme.line2)
                    opacity: st === "offline" ? 0.6 : 1
                    implicitWidth: chip.implicitWidth + 18; implicitHeight: 30
                    ToolTip.visible: errMa.containsMouse && hwErr !== 0
                    ToolTip.text: "⚠ Hardware error (overload?). Connect panel → Reboot #" + modelData.id + ", then Set home for the finger"
                    MouseArea { id: errMa; anchors.fill: parent; hoverEnabled: true
                                acceptedButtons: Qt.NoButton }   // hover only — clicks pass through
                    Row {
                        id: chip; anchors.centerIn: parent; spacing: 7
                        Rectangle { width: 7; height: 7; radius: 2; color: Theme.dofColor(modelData.role); anchors.verticalCenter: parent.verticalCenter }
                        Text { text: (hwErr ? "⚠ " : "") + "#" + modelData.id
                               color: hwErr ? Theme.red : Theme.text
                               font.family: Theme.mono; font.pixelSize: 11; anchors.verticalCenter: parent.verticalCenter }
                        Rectangle { width: 5; height: 5; radius: 2.5
                            color: hwErr ? Theme.red : st === "online" ? Theme.green : st === "hot" ? Theme.amber : Theme.text3
                            anchors.verticalCenter: parent.verticalCenter }
                        // compact torque switch
                        Rectangle {
                            width: 28; height: 16; radius: 8; anchors.verticalCenter: parent.verticalCenter
                            color: on ? Theme.green : Theme.line
                            Behavior on color { ColorAnimation { duration: 130 } }
                            Rectangle {
                                width: 12; height: 12; radius: 6; color: "#fff"; y: 2
                                x: on ? 14 : 2
                                Behavior on x { NumberAnimation { duration: 130; easing.type: Easing.OutQuad } }
                            }
                            MouseArea { anchors.fill: parent; cursorShape: Qt.PointingHandCursor
                                onClicked: controller.toggleTorque(root.finger.uid, modelData.id) }
                        }
                        // live per-motor telemetry (Dynamixel only): grip current + temperature
                        Text {
                            visible: chipCard.showTelem
                            text: chipCard.curMa + "mA"
                            color: chipCard.curMa >= controller.curHotMa ? Theme.amber : Theme.text2
                            font.family: Theme.mono; font.pixelSize: 10
                            width: Math.max(maW.advanceWidth, implicitWidth)   // fixed slot -> no twitch
                            horizontalAlignment: Text.AlignRight
                            anchors.verticalCenter: parent.verticalCenter
                        }
                        Text {
                            visible: chipCard.showTelem
                            text: chipCard.tempC + "°"
                            color: chipCard.hwErr ? Theme.red
                                 : chipCard.tempC >= controller.tempHotC ? Theme.amber : Theme.text3
                            font.family: Theme.mono; font.pixelSize: 10
                            width: Math.max(tempW.advanceWidth, implicitWidth)
                            horizontalAlignment: Text.AlignRight
                            anchors.verticalCenter: parent.verticalCenter
                        }
                    }
                }
            }
        }

        // ── DOF steppers ──────────────────────────────────────────
        ColumnLayout {
            Layout.fillWidth: true; Layout.leftMargin: 15; Layout.rightMargin: 15
            Layout.topMargin: 8; Layout.bottomMargin: 15; spacing: 13
            Repeater {
                model: controller.dofsFor(root.finger)
                delegate: ColumnLayout {
                    required property var modelData
                    readonly property color dc: Theme.dofColor(modelData.group)
                    readonly property real val: controller.dofValue(root.finger.uid, modelData.id, modelData.home)
                    Layout.fillWidth: true; spacing: 7

                    // ±step° on this DOF (− / + button · mouse wheel). Drives the existing path
                    // setDof → pushSim → jogNorm; clamps to the range and only sends on a real
                    // change (no serial spam at the end stop).
                    function nudge(step) {
                        var cur = Math.round(val)
                        var nv = Math.max(modelData.min, Math.min(modelData.max, cur + step))
                        if (nv !== cur) controller.setDof(root.finger.uid, modelData.id, nv)
                    }

                    RowLayout {
                        Layout.fillWidth: true; spacing: 8
                        Rectangle { width: 8; height: 8; radius: 2; color: dc }
                        Text { text: modelData.label; color: Theme.text2; font.pixelSize: 12; Layout.fillWidth: true }
                        Rectangle {
                            visible: modelData.coupled === true
                            radius: 4; color: "transparent"; border.color: Theme.line
                            implicitWidth: cpl.implicitWidth + 10; implicitHeight: 16
                            Text { id: cpl; anchors.centerIn: parent; text: "coupled"; color: Theme.text3; font.family: Theme.mono; font.pixelSize: 9 }
                        }
                        // Stepper: a value field (click to type an exact angle) flanked by − / + .
                        // Click = ±1°, Shift-click = ±5°, hold = auto-repeat.
                        RowLayout {
                            spacing: 4
                            StepButton { amount: -1; glyph: "−"; armed: !controller.eStopped
                                         onStepped: function(by) { nudge(by) } }
                            // value field — click to type an exact angle (Enter commits, Esc cancels)
                            Item {
                                implicitWidth: 48; implicitHeight: 22
                                Rectangle {
                                    anchors.fill: parent; radius: 6
                                    color: valIn.visible ? Theme.panel : Theme.card2
                                    border.color: (valHov.hovered || valIn.visible) ? Theme.line : Theme.line2
                                }
                                Text {
                                    id: valT
                                    anchors.centerIn: parent
                                    visible: !valIn.visible
                                    text: (modelData.id === "spread" && val > 0 ? "+" : "") + Math.round(val) + modelData.unit
                                    color: Theme.text; font.family: Theme.mono; font.pixelSize: 14; font.weight: Font.DemiBold
                                    HoverHandler { id: valHov; cursorShape: Qt.IBeamCursor }
                                    TapHandler {
                                        enabled: !controller.eStopped
                                        onTapped: { valIn.text = "" + Math.round(val); valIn.visible = true
                                                    valIn.forceActiveFocus(); valIn.selectAll() }
                                    }
                                }
                                TextInput {
                                    id: valIn
                                    visible: false
                                    anchors.centerIn: parent
                                    width: 40; horizontalAlignment: TextInput.AlignHCenter
                                    color: Theme.accent; font.family: Theme.mono; font.pixelSize: 14; font.weight: Font.DemiBold
                                    validator: IntValidator { bottom: -360; top: 360 }
                                    function commitEdit() {
                                        if (!visible) return
                                        var v = parseInt(text)
                                        if (!isNaN(v)) {
                                            v = Math.max(modelData.min, Math.min(modelData.max, v))   // clamp to range
                                            controller.setDof(root.finger.uid, modelData.id, v)
                                        }
                                        visible = false
                                    }
                                    onAccepted: { commitEdit(); focus = false }
                                    onActiveFocusChanged: if (!activeFocus) commitEdit()
                                    Keys.onEscapePressed: { visible = false; focus = false }
                                }
                            }
                            StepButton { amount: 1; glyph: "+"; armed: !controller.eStopped
                                         onStepped: function(by) { nudge(by) } }

                            // Mouse wheel over the field = ±1° (Shift ±5°); consume the event, since
                            // the card sits in a ScrollView (else the page scrolls instead of nudging).
                            WheelHandler {
                                enabled: !controller.eStopped
                                onWheel: function(w) {
                                    var dy = (w.pixelDelta.y !== 0) ? w.pixelDelta.y : w.angleDelta.y
                                    if (dy === 0) return
                                    var big = (w.modifiers & Qt.ShiftModifier) ? 5 : 1
                                    nudge((dy > 0 ? 1 : -1) * big)
                                    w.accepted = true
                                }
                            }
                        }
                    }

                    // Position slider in our design language: a bold knob on a pill track, filled
                    // min→val in the DOF colour. Drag = position (the knob mirrors val → follows
                    // telemetry / return-home); the +/− steppers stay the fine control.
                    Item {
                        id: dial
                        Layout.fillWidth: true; implicitHeight: 24
                        readonly property real frac: (modelData.max > modelData.min)
                            ? Math.max(0, Math.min(1, (val - modelData.min) / (modelData.max - modelData.min))) : 0
                        Rectangle {          // groove
                            anchors.left: parent.left; anchors.right: parent.right
                            anchors.verticalCenter: parent.verticalCenter
                            height: 6; radius: 3; color: Theme.panel2; border.color: Theme.line2
                            Rectangle {      // fill min→val
                                anchors.left: parent.left; anchors.verticalCenter: parent.verticalCenter
                                width: dial.frac * parent.width; height: parent.height; radius: 3
                                color: dc; opacity: 0.55
                            }
                        }
                        Rectangle {          // knob
                            width: 20; height: 20; radius: 10
                            y: parent.height / 2 - height / 2
                            x: dial.frac * (dial.width - width)
                            color: dialMa.pressed ? Theme.soft(dc, 0.35) : Theme.card2
                            border.color: dc; border.width: 2
                            opacity: controller.eStopped ? 0.4 : 1
                            Rectangle { anchors.centerIn: parent; width: 6; height: 6; radius: 3; color: dc }
                            Behavior on x { enabled: !dialMa.pressed; NumberAnimation { duration: 90 } }
                        }
                        MouseArea {
                            id: dialMa; anchors.fill: parent; enabled: !controller.eStopped
                            cursorShape: Qt.PointingHandCursor
                            onPressed: function(m) { dialSet(m.x) }
                            onPositionChanged: function(m) { if (pressed) dialSet(m.x) }
                            function dialSet(mx) {
                                var f = Math.max(0, Math.min(1, mx / dial.width))
                                controller.setDof(root.finger.uid, modelData.id,
                                    Math.round(modelData.min + f * (modelData.max - modelData.min)))
                            }
                        }
                    }

                    // min / max range labels — tap to type an exact limit. This sets the
                    // EFFECTIVE DOF range: the value field + twin joint + commanded servo travel
                    // (⚠ on wired hardware this is a real mechanical limit). ↺ resets to base.
                    RowLayout {
                        Layout.fillWidth: true; spacing: 6

                        // ── min ──
                        Item {
                            implicitWidth: Math.max(minT.implicitWidth, minIn.visible ? 42 : 0); implicitHeight: 16
                            Text {
                                id: minT
                                anchors.left: parent.left; anchors.verticalCenter: parent.verticalCenter
                                visible: !minIn.visible
                                text: modelData.min + modelData.unit
                                color: Theme.text3; font.family: Theme.mono; font.pixelSize: 9
                                Rectangle { visible: minHov.hovered; anchors.top: parent.bottom
                                            width: parent.width; height: 1; color: Theme.text3; opacity: 0.5 }
                                HoverHandler { id: minHov; cursorShape: Qt.IBeamCursor }
                                TapHandler {
                                    enabled: !controller.eStopped
                                    onTapped: { minIn.text = "" + modelData.min; minIn.visible = true
                                                minIn.forceActiveFocus(); minIn.selectAll() }
                                }
                            }
                            TextInput {
                                id: minIn
                                visible: false
                                anchors.left: parent.left; anchors.verticalCenter: parent.verticalCenter
                                width: 42; horizontalAlignment: TextInput.AlignLeft
                                color: Theme.accent; font.family: Theme.mono; font.pixelSize: 9; font.weight: Font.DemiBold
                                validator: IntValidator { bottom: -360; top: 360 }
                                function commitEdit() {
                                    if (!visible) return
                                    visible = false                      // settle BEFORE the recompose (delegate may rebuild)
                                    var v = parseInt(text)
                                    if (!isNaN(v)) controller.setDofRange(root.finger.uid, modelData.id, v, modelData.max)
                                }
                                onAccepted: commitEdit()
                                onActiveFocusChanged: if (!activeFocus) commitEdit()
                                Keys.onEscapePressed: { visible = false; focus = false }
                                ToolTip.visible: activeFocus
                                ToolTip.text: "Min limit (also caps the real servo travel)"
                            }
                        }

                        // reset to base range — only shown once the user has edited this DOF
                        Item {
                            Layout.fillWidth: true; implicitHeight: 16
                            Rectangle {
                                visible: controller.dofRangeEdited(root.finger.uid, modelData.id)
                                anchors.centerIn: parent
                                implicitWidth: rst.implicitWidth + 10; implicitHeight: 14; radius: 4
                                color: "transparent"; border.color: Theme.line
                                Text { id: rst; anchors.centerIn: parent; text: "↺ base"; color: Theme.text3
                                       font.family: Theme.mono; font.pixelSize: 8 }
                                MouseArea {
                                    id: rstMa; anchors.fill: parent; hoverEnabled: true
                                    enabled: !controller.eStopped; cursorShape: Qt.PointingHandCursor
                                    onClicked: controller.resetDofRange(root.finger.uid, modelData.id)
                                }
                                ToolTip.visible: rstMa.containsMouse
                                ToolTip.text: "Reset limits to base values"
                            }
                        }

                        // ── max ──
                        Item {
                            implicitWidth: Math.max(maxT.implicitWidth, maxIn.visible ? 42 : 0); implicitHeight: 16
                            Text {
                                id: maxT
                                anchors.right: parent.right; anchors.verticalCenter: parent.verticalCenter
                                visible: !maxIn.visible
                                text: modelData.max + modelData.unit
                                color: Theme.text3; font.family: Theme.mono; font.pixelSize: 9
                                Rectangle { visible: maxHov.hovered; anchors.top: parent.bottom
                                            width: parent.width; height: 1; color: Theme.text3; opacity: 0.5 }
                                HoverHandler { id: maxHov; cursorShape: Qt.IBeamCursor }
                                TapHandler {
                                    enabled: !controller.eStopped
                                    onTapped: { maxIn.text = "" + modelData.max; maxIn.visible = true
                                                maxIn.forceActiveFocus(); maxIn.selectAll() }
                                }
                            }
                            TextInput {
                                id: maxIn
                                visible: false
                                anchors.right: parent.right; anchors.verticalCenter: parent.verticalCenter
                                width: 42; horizontalAlignment: TextInput.AlignRight
                                color: Theme.accent; font.family: Theme.mono; font.pixelSize: 9; font.weight: Font.DemiBold
                                validator: IntValidator { bottom: -360; top: 360 }
                                function commitEdit() {
                                    if (!visible) return
                                    visible = false
                                    var v = parseInt(text)
                                    if (!isNaN(v)) controller.setDofRange(root.finger.uid, modelData.id, modelData.min, v)
                                }
                                onAccepted: commitEdit()
                                onActiveFocusChanged: if (!activeFocus) commitEdit()
                                Keys.onEscapePressed: { visible = false; focus = false }
                                ToolTip.visible: activeFocus
                                ToolTip.text: "Max limit (also caps the real servo travel)"
                            }
                        }
                    }
                }
            }

            // per-finger return home (limp servos take no goals \u2014 disabled while e-stopped)
            Rectangle {
                Layout.topMargin: 2; radius: 8; color: "transparent"; border.color: Theme.line
                implicitWidth: rh.implicitWidth + 24; implicitHeight: 28
                opacity: controller.eStopped ? 0.4 : 1
                Text { id: rh; anchors.centerIn: parent; text: "\u21BA Return home"; color: Theme.text2; font.pixelSize: 12 }
                MouseArea { anchors.fill: parent; enabled: !controller.eStopped
                            cursorShape: enabled ? Qt.PointingHandCursor : Qt.ArrowCursor
                            onClicked: controller.returnHome(root.finger.uid) }
            }
        }
    }
}
