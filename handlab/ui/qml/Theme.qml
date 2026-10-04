pragma Singleton
import QtCore
import QtQuick

// ─────────────────────────────────────────────────────────────────────────────
//  Theme — single source of truth for colours, spacing, radii and per-DOF
//  identity colours. Dark / light are data here (see `mode`); views never
//  hardcode a colour. Per-DOF colours come from `dofColor(group)`, so a finger
//  type that introduces a new DOF group only needs an entry below.
// ─────────────────────────────────────────────────────────────────────────────
QtObject {
    id: theme

    // ── theme switch (persisted across restarts) ──────────────────
    property string mode: "dark"
    readonly property bool dark: mode === "dark"
    function toggle() { mode = dark ? "light" : "dark" }
    readonly property Settings _store: Settings {
        category: "Theme"
        property alias mode: theme.mode    // alias = auto-restore on load, auto-save on change
    }

    // ── type ──────────────────────────────────────────────────────
    readonly property string sans: Qt.application.font.family
    readonly property string mono: "monospace"

    // ── radii (px) ────────────────────────────────────────────────
    readonly property int rSm: 8
    readonly property int rMd: 12
    readonly property int rLg: 15

    // ── surfaces / text (theme-dependent) ─────────────────────────
    readonly property color bg:        dark ? "#0c0f13" : "#e8ecf1"
    readonly property color panel:     dark ? "#11161c" : "#f3f6f9"
    readonly property color panel2:    dark ? "#0e1318" : "#eaeef3"
    readonly property color card:      dark ? "#161c24" : "#ffffff"
    readonly property color card2:     dark ? "#1c232c" : "#f6f8fb"
    readonly property color line:      dark ? "#262e38" : "#dbe1e8"
    readonly property color line2:     dark ? "#1b222b" : "#e8ecf1"
    readonly property color text:      dark ? "#e8edf2" : "#19212b"
    readonly property color text2:     dark ? "#95a2ae" : "#5a6573"
    readonly property color text3:     dark ? "#5d6772" : "#8a95a1"
    readonly property color accent:    dark ? "#5aa0e0" : "#2f7fd1"
    readonly property color accentSoft: dark ? "#245aa0e0" : "#1f2f7fd1" // ARGB, low alpha
    readonly property color viewport:  dark ? "#0a0d11" : "#dde3ea"
    readonly property color viewport2: dark ? "#0e141b" : "#e7ecf2"

    // ── state colours (theme-independent) ─────────────────────────
    readonly property color green:  "#3fb883"
    readonly property color amber:  "#e7a93f"
    readonly property color red:    "#e5484d"

    // ── per-DOF identity colours (data) ───────────────────────────
    readonly property color cyan:   "#3fc4d0"   // flex
    readonly property color violet: "#9d82ec"   // spread
    readonly property color orange: "#ec9a55"   // curl
    readonly property color rose:   "#e879a6"   // opposition

    readonly property var dofColors: ({
        "spread": violet,
        "flex":   cyan,
        "curl":   orange,
        "bend":   violet,
        "curve":  cyan,
        "elong":  rose
    })
    function dofColor(group) {
        return dofColors[group] !== undefined ? dofColors[group] : accent
    }

    // soft tint of a colour for badge fills
    function soft(c, a) { return Qt.rgba(c.r, c.g, c.b, a === undefined ? 0.16 : a) }
}
