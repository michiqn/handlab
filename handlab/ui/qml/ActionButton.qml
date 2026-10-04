import QtQuick
import "."

// Shared pill action button (Control + Poses panels — was two near-identical inline components).
// `accent` = red/destructive styling, `active` = false dims + disables the click.
// baseColor/textColor cover the panels' surface deltas (card/text vs card2/text2).
Rectangle {
    id: root
    property alias text: bl.text
    property bool accent: false
    property bool active: true         // false = visibly disabled (e.g. while e-stopped)
    property color baseColor: Theme.card
    property color textColor: Theme.text
    signal clicked()
    implicitWidth: bl.implicitWidth + 26; implicitHeight: 32; radius: 9
    color: accent ? Theme.soft(Theme.red, 0.15) : baseColor
    border.color: accent ? Theme.red : Theme.line
    opacity: active ? 1 : 0.4
    Text { id: bl; anchors.centerIn: parent; color: root.accent ? Theme.red : root.textColor
           font.pixelSize: 12; font.weight: root.accent ? Font.DemiBold : Font.Medium }
    MouseArea { anchors.fill: parent; enabled: root.active
                cursorShape: enabled ? Qt.PointingHandCursor : Qt.ArrowCursor
                onClicked: root.clicked() }
}
