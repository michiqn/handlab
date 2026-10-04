pragma Singleton
import QtQuick

// Standalone fallback catalogs — used ONLY when Main.qml runs without the Python Bridge
// (`qml Main.qml` for pure-UI work). With the app running, the real on-disk library drives
// the catalogs via handlab.fingerCatalog/adapterCatalog. Keep the numbers in sync with the
// shipped library (handlab/library/*): ranges below mirror the corrected CAD export.
QtObject {

    // An adapter mates the base's center norm and carries 1..N typed finger slots.
    readonly property var adapterTypes: [
        { id: "palm", name: "Palm · 3 slots", fits: "center",
          blurb: "center norm · 3 fingers",
          ports: [ { id: "index", label: "Index slot", short: "P1", accepts: "finger" },
                   { id: "thumb", label: "Thumb slot", short: "P2", accepts: "thumb" },
                   { id: "mid",   label: "Mid slot",   short: "P3", accepts: "finger" } ] }
    ]

    // Each type declares its OWN DOF set — the UI is never hardcoded to a finger/joint count.
    readonly property var fingerTypes: [
        {
            id: "claw_finger", name: "Claw index", mech: "tendon finger", motors: 3,
            blurb: "claw index · 3 dof", slot: "finger",
            dofs: [
                { id: "spread", label: "Spread", group: "spread", unit: "°", min: -8,  max: 60,  home: 0 },
                { id: "flex",   label: "Flex",   group: "flex",   unit: "°", min: -70, max: 120, home: 0 },
                { id: "curl",   label: "Curl",   group: "curl",   unit: "°", min: -20, max: 110, home: 0, coupled: true }
            ]
        },
        {
            id: "claw_mid", name: "Claw mid", mech: "tendon finger", motors: 3,
            blurb: "claw mid · 3 dof", slot: "finger",
            dofs: [
                { id: "spread", label: "Spread", group: "spread", unit: "°", min: -45, max: 55,  home: 0 },
                { id: "flex",   label: "Flex",   group: "flex",   unit: "°", min: -70, max: 120, home: 0 },
                { id: "curl",   label: "Curl",   group: "curl",   unit: "°", min: -20, max: 110, home: 0, coupled: true }
            ]
        },
        {
            id: "claw_thumb", name: "Claw thumb", mech: "tendon thumb", motors: 3,
            blurb: "claw thumb · 3 dof", slot: "thumb",
            dofs: [
                { id: "spread", label: "Spread", group: "spread", unit: "°", min: -70, max: 65,  home: 0 },
                { id: "flex",   label: "Flex",   group: "flex",   unit: "°", min: -5,  max: 190, home: 0 },
                { id: "curl",   label: "Curl",   group: "curl",   unit: "°", min: -20, max: 110, home: 0, coupled: true }
            ]
        }
    ]

    // Finger sockets come from the ADAPTER's ports (Main.qml socketList) — a mount only
    // declares itself; the pre-adapter `sockets:` array is gone.
    readonly property var mounts: [
        { id: "palm_mount", name: "Palm base", blurb: "1 center norm · claw base" }
    ]
}
