import QtCore
import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Window
import "."

// ─────────────────────────────────────────────────────────────────────────────
//  handlab — control center + live digital twin for modular, tendon-driven
//  robotic hands. Driven by the Python Bridge (`handlab` context property);
//  runs standalone too (`qml Main.qml`) on the MockCatalog fallback.
//
//  Everything is data-driven: finger / DOF / servo counts come from `fingers`
//  + the type catalog, never from constants.
// ─────────────────────────────────────────────────────────────────────────────
ApplicationWindow {
    id: app
    visible: true
    width: 1280
    height: 820
    minimumWidth: 1040
    minimumHeight: 660
    title: "handlab"
    color: Theme.bg
    Behavior on color { ColorAnimation { duration: 180 } }

    // Part catalogs. Driven by the Python Bridge (the real on-disk library) when it is present;
    // the MockCatalog singleton is the fallback for running this QML standalone. The library is
    // fixed at boot (no runtime import), so these are read once.
    property var fingerTypes: refreshCatalog()
    property var adapterTypes: refreshAdapters()
    function refreshCatalog() {     // null check matters during app teardown
        return (typeof handlab !== "undefined" && handlab) ? JSON.parse(handlab.fingerCatalog) : MockCatalog.fingerTypes
    }
    function refreshAdapters() {
        return (typeof handlab !== "undefined" && handlab) ? JSON.parse(handlab.adapterCatalog) : MockCatalog.adapterTypes
    }
    // Base catalog (static — the palm base with its one center norm).
    readonly property var mounts: MockCatalog.mounts

    // Per-socket finger-identity colours (badges / socket nodes).
    readonly property var idColors: [Theme.cyan, Theme.violet, Theme.orange, Theme.rose]

    // ── live build state (mutated functionally so bindings refresh) ──────────
    property string mountId: "palm_mount"
    property string adapterId: ""        // center adapter (its ports host the "p*" fingers)
    property var    fingers: []          // [{ uid, typeId, socket, servos:[{id,dofId,role}] }]
    property var    dofValues: ({})      // "uid:dofId" -> degrees
    property var    torque: ({})         // "uid:servoId" -> bool
    property var    servoStatus: ({})    // "uid:servoId" -> "online"|"hot"|"offline"
    property bool   eStopped: false
    property int    tabIndex: 0
    // The demo recorder's card registers itself here when its tab becomes visible, so the keyboard
    // shortcuts below act on the card you are actually looking at (Teleop or Agent).
    property var    demoCard: null
    // A text field has focus -> those shortcuts stand down (a pose name may contain spaces!).
    // Duck-typed on activeFocusItem rather than hooking every field: TextField/TextInput/TextEdit
    // all expose `selectedText`, so a field added later is covered without touching this.
    readonly property bool textFocus: activeFocusItem !== null
                                      && activeFocusItem.selectedText !== undefined
    property int    uidSeq: 0
    property int    twinFrame: 0         // bumped by the Python Bridge each rendered MuJoCo frame

    // The assembled build survives restarts; first boot (or an invalid save, e.g. a
    // removed part type) falls back to the real-hardware preset.
    Settings { id: buildStore; category: "Main.build"; property string lastBuild: "" }
    // Fresh boot seeds the assembly from the YAML build (via the bridge's buildSpec) — the yaml
    // is the single source of truth for the servo mapping. Only when there's no live bridge
    // (standalone QML) do we fall back to a persisted UI assembly, then the hardcoded example.
    Component.onCompleted: if (!seedFromBuildSpec() && !restoreBuild()) loadPreset("claw")

    // seed per-finger UI state (dof homes, torque on, servos online) into the given maps
    function seedFinger(fg, dv, tq, ss) {
        var t = typeById(fg.typeId)
        for (var i = 0; i < t.dofs.length; i++) dv[fg.uid + ":" + t.dofs[i].id] = t.dofs[i].home
        for (var j = 0; j < fg.servos.length; j++) { tq[fg.uid + ":" + fg.servos[j].id] = true; ss[fg.uid + ":" + fg.servos[j].id] = "online" }
    }

    function restoreBuild() {
        try {
            if (buildStore.lastBuild === "") return false
            var s = JSON.parse(buildStore.lastBuild)
            if (!s || !mountById(s.mount)) return false
            if (s.adapter && !adapterById(s.adapter)) return false
            // valid finger sockets = the saved adapter's ports
            var ok = {}, ad = s.adapter ? adapterById(s.adapter) : null
            if (ad) for (var p = 0; p < ad.ports.length; p++) ok[ad.ports[p].id] = true
            var f = [], dv = {}, tq = {}, ss = {}, fl = s.fingers || []
            for (var i = 0; i < fl.length; i++) {
                var fg = fl[i]
                if (!typeById(fg.typeId) || !ok[fg.socket]) return false
                f.push(fg); seedFinger(fg, dv, tq, ss)
            }
            mountId = s.mount; adapterId = s.adapter || ""
            uidSeq = s.uidSeq || f.length
            fingers = f; dofValues = dv; torque = tq; servoStatus = ss
            pushBuild()
            return true
        } catch (e) { return false }
    }

    // Build one UI finger from a buildSpec entry — the servo IDs come VERBATIM from the yaml,
    // so the yaml mapping (builds/<HANDLAB_BUILD>.yaml) is honoured.
    function fingerFromSpec(fs) {
        var t = typeById(fs.type), servos = []
        if (!t) return null
        for (var i = 0; i < t.dofs.length; i++)
            servos.push({ id: fs.servos[i], dofId: t.dofs[i].id, role: t.dofs[i].group })
        uidSeq++
        return { uid: "f" + uidSeq, typeId: fs.type, socket: fs.socket, servos: servos }
    }

    // Seed the assembly from the bridge's loaded YAML build (buildSpec). Returns false when there
    // is no live bridge (standalone QML) so the caller can fall back.
    function seedFromBuildSpec() {
        if (typeof handlab === "undefined" || !handlab) return false
        try {
            var s = JSON.parse(handlab.buildSpec)
            if (!s || !mountById(s.mount)) return false
            if (s.adapter && !adapterById(s.adapter)) return false
            var ad = s.adapter ? adapterById(s.adapter) : null
            var ok = {}
            if (ad) for (var p = 0; p < ad.ports.length; p++) ok[ad.ports[p].id] = true
            uidSeq = 0
            var f = [], dv = {}, tq = {}, ss = {}, fl = s.fingers || []
            for (var i = 0; i < fl.length; i++) {
                var fg = fingerFromSpec(fl[i])
                if (!fg || (ad && !ok[fg.socket])) return false
                f.push(fg); seedFinger(fg, dv, tq, ss)
            }
            mountId = s.mount; adapterId = s.adapter || ""
            fingers = f; dofValues = dv; torque = tq; servoStatus = ss
            pushBuild()
            return true
        } catch (e) { return false }
    }

    // Live MuJoCo frames from the Python Bridge (null target when running pure-QML standalone).
    Connections {
        target: (typeof handlab !== "undefined") ? handlab : null
        function onFrameTick(n) { app.twinFrame = n; app._fpsCount++ }
        function onTelemetryChanged(t) { app.telemetry = JSON.parse(t) }
        function onPlaybackFrame(f) { app.applyPoseFrame(JSON.parse(f)) }
        function onTeleopFrame(f)   { app.syncPoseFrame(JSON.parse(f)) }    // teleop moved in PYTHON already — mirror only
    }

    // Live servo telemetry from the driver (10 Hz): id -> {pos,torque,moving,temp,volt,online}.
    // Empty when no bridge — the local mock state below takes over (standalone QML).
    property var telemetry: ({})
    readonly property string driverName: (typeof handlab !== "undefined" && handlab) ? handlab.driverName : "mock"
    // Telemetry display thresholds (single source; statusOf + the Control-tab per-motor readout share them).
    readonly property int tempHotC: 50    // °C — servo runs hot (was hardcoded in statusOf)
    readonly property int curHotMa: 900   // mA — sustained high force (firm grip / stall) → tint the readout

    // Poses & recordings (Bridge-owned, per build). The Bridge is the clock; we apply each
    // emitted frame through the normal DOF path so sliders animate + the twin moves safely.
    property var poses: (typeof handlab !== "undefined" && handlab) ? JSON.parse(handlab.posesJson) : []
    readonly property bool isRecording: (typeof handlab !== "undefined" && handlab) ? handlab.isRecording : false
    readonly property bool isPlaying: (typeof handlab !== "undefined" && handlab) ? handlab.isPlaying : false
    function applyPoseFrame(dofs) {
        if (eStopped) return                          // never move under e-stop
        for (var sock in dofs) {
            var fg = fingerOnSocket(sock)
            if (!fg) continue
            var t = typeById(fg.typeId), arr = dofs[sock]
            for (var j = 0; j < t.dofs.length && j < arr.length; j++)
                setDof(fg.uid, t.dofs[j].id, Math.round(arr[j]))
        }
    }

    // Teleop DISPLAY mirror: motion already happened in Python (direct clamped driver write) —
    // this only syncs sliders/labels. ONE dofValues clone per frame instead of 9 setDof clones,
    // and NO pushSim/jogNorm (which would re-issue 9 serial writes + 9 dofRanges YAML loads).
    function syncPoseFrame(dofs) {
        if (eStopped) return
        var dv = Object.assign({}, dofValues)
        for (var sock in dofs) {
            var fg = fingerOnSocket(sock)
            if (!fg) continue
            var t = typeById(fg.typeId), arr = dofs[sock]
            for (var j = 0; j < t.dofs.length && j < arr.length; j++)
                dv[fg.uid + ":" + t.dofs[j].id] = Math.round(arr[j])
        }
        dofValues = dv
    }

    // Real render rate of the twin (frames actually delivered in the last second).
    property int twinFps: 0
    property int _fpsCount: 0
    Timer {
        interval: 1000; running: true; repeat: true
        onTriggered: { app.twinFps = app._fpsCount; app._fpsCount = 0 }
    }

    // Switching tabs: frame the assembly centrally in the live twin (every tab shows the twin).
    onTabIndexChanged: if (typeof handlab !== "undefined" && handlab) handlab.resetView()

    // Pause the offscreen render when the window is minimized/hidden (perf — nothing to show).
    onVisibilityChanged: if (typeof handlab !== "undefined" && handlab)
        handlab.setWindowVisible(app.visibility !== Window.Hidden && app.visibility !== Window.Minimized)

    // Push a DOF to the real MuJoCo twin (no-op in standalone). Looks up finger + dof ordinals.
    // Normalize against the EFFECTIVE range (same source as the slider bounds, via dofsFor) —
    // NOT the base type range — so the norm matches jogNorm's effective range on the Python side
    // (else a min/max override compresses the usable travel to the base range).
    function pushSim(uid, dofId, val) {
        if (typeof handlab === "undefined") return
        var o = _dofOrdinals(uid, dofId)
        if (!o) return
        var d = dofsFor(fingers[o[0]])[o[1]]
        if (!d || d.max === d.min) return
        handlab.jogNorm(o[0], o[1], (val - d.min) / (d.max - d.min))
    }

    // Seed the twin to the current DOF homes. The Bridge already composed the build at boot from
    // builds/<HANDLAB_BUILD>.yaml (the single source of truth for the servo mapping), so there is no
    // structural push from the UI anymore — this only persists the UI assembly + syncs the homes.
    function pushBuild() {
        // persist the assembly (works standalone too); restored on next launch
        buildStore.lastBuild = JSON.stringify({ mount: mountId, adapter: adapterId, fingers: fingers, uidSeq: uidSeq })
        if (typeof handlab === "undefined") return
        syncAllDofs()
    }
    function syncAllDofs() {
        if (typeof handlab === "undefined") return
        for (var i = 0; i < fingers.length; i++) {
            var t = typeById(fingers[i].typeId)
            for (var j = 0; j < t.dofs.length; j++)
                pushSim(fingers[i].uid, t.dofs[j].id, dofValue(fingers[i].uid, t.dofs[j].id, t.dofs[j].home))
        }
    }
    // resolve (uid, dofId) -> (fingerOrd, dofOrd) like pushSim; returns [fo, di] or null
    function _dofOrdinals(uid, dofId) {
        var fo = -1; for (var i = 0; i < fingers.length; i++) if (fingers[i].uid === uid) { fo = i; break }
        if (fo < 0) return null
        var t = typeById(fingers[fo].typeId), di = -1
        for (var j = 0; j < t.dofs.length; j++) if (t.dofs[j].id === dofId) { di = j; break }
        return di < 0 ? null : [fo, di]
    }
    // set a DOF's effective min/max° (slider + twin + commanded servo travel; persists in
    // calibration.yaml). Recomposes the twin; keeps the current command inside the new bounds.
    function setDofRange(uid, dofId, minDeg, maxDeg) {
        if (typeof handlab === "undefined" || !handlab) return null
        var o = _dofOrdinals(uid, dofId); if (!o) return null
        var res = JSON.parse(handlab.setDofRange(o[0], o[1], Math.round(minDeg), Math.round(maxDeg)))
        if (res.ok) {
            var cur = dofValue(uid, dofId, 0)
            var cl = Math.max(res.range_deg[0], Math.min(res.range_deg[1], cur))
            if (cl !== cur) setDof(uid, dofId, cl)   // pull a now-out-of-range command back in
        }
        return res
    }
    function resetDofRange(uid, dofId) {
        if (typeof handlab === "undefined" || !handlab) return
        var o = _dofOrdinals(uid, dofId); if (o) handlab.resetDofRange(o[0], o[1])
    }
    // has the user range-edited this DOF? (→ show the reset affordance; port mirrors don't count)
    function dofRangeEdited(uid, dofId) {
        if (typeof handlab === "undefined" || !handlab) return false   // !handlab: null during teardown
        var o = _dofOrdinals(uid, dofId); if (!o) return false
        return (JSON.parse(handlab.dofUserRanges))[o[0] + ":" + o[1]] === true
    }

    // ── lookups ──────────────────────────────────────────────────────────────
    function typeById(id)  { for (var i = 0; i < fingerTypes.length; i++) if (fingerTypes[i].id === id) return fingerTypes[i]; return null }
    function adapterById(id) { for (var i = 0; i < adapterTypes.length; i++) if (adapterTypes[i].id === id) return adapterTypes[i]; return null }
    function mountById(id) { for (var i = 0; i < mounts.length; i++) if (mounts[i].id === id) return mounts[i]; return null }
    function curMount()    { return mountById(mountId) }
    function curAdapter()  { return adapterById(adapterId) }

    // The finger sockets = the selected adapter's ports. The base's center socket
    // itself is not a finger target — it hosts the adapter. Recomputed
    // automatically when the adapter or catalogs change.
    readonly property var socketList: {
        var l = [], ad = adapterById(adapterId)
        if (ad) for (var i = 0; i < ad.ports.length; i++)
            l.push({ id: ad.ports[i].id, label: ad.ports[i].label, short: ad.ports[i].short,
                     accepts: ad.ports[i].accepts || "",
                     ranges: ad.ports[i].ranges || ({}) })
        return l
    }
    function socketCount() { return socketList.length }
    function socketDef(i)  { return (i >= 0 && i < socketList.length) ? socketList[i] : null }
    function socketLabel(i){ var sd = socketDef(i); return sd ? sd.label : "" }
    function socketShort(i){ var sd = socketDef(i); return sd ? sd.short : "" }
    function socketIndexOf(id) { for (var i = 0; i < socketList.length; i++) if (socketList[i].id === id) return i; return -1 }
    function idColorFor(i) { var n = idColors.length; return idColors[((i % n) + n) % n] }   // safe for -1 during rebinds
    function fingerOnSocket(id) { for (var i = 0; i < fingers.length; i++) if (fingers[i].socket === id) return fingers[i]; return null }
    function fingerByUid(u){ for (var i = 0; i < fingers.length; i++) if (fingers[i].uid === u) return fingers[i]; return null }
    // the finger's DOF list with the slot's range overrides applied (e.g. the palm's
    // mid slot mirrors the spread travel of the same printed finger)
    function dofsFor(fg) {
        var t = fg ? typeById(fg.typeId) : null
        if (!t) return []
        // effective per-DOF [min,max]° bounds. The Bridge is the single source (merges the
        // adapter-port mirror AND the user's "edit limits"); fall back to the slot's port
        // ranges only when headless/mock (no Bridge). Keys are "<fingerOrd>:<dofOrd>".
        var ov = ({})
        if (typeof handlab !== "undefined" && handlab) {
            var fo = -1; for (var k = 0; k < fingers.length; k++) if (fingers[k].uid === fg.uid) { fo = k; break }
            var dr = JSON.parse(handlab.dofRanges)
            for (var m = 0; m < t.dofs.length; m++) { var r = dr[fo + ":" + m]; if (r) ov[t.dofs[m].id] = r }
        } else {
            var sd = socketDef(socketIndexOf(fg.socket))
            ov = (sd && sd.ranges) ? sd.ranges : ({})
        }
        var out = []
        for (var i = 0; i < t.dofs.length; i++) {
            var d = t.dofs[i]
            if (ov[d.id]) {
                d = JSON.parse(JSON.stringify(d))
                d.min = ov[d.id][0]; d.max = ov[d.id][1]
                d.home = Math.max(d.min, Math.min(d.max, d.home))
            }
            out.push(d)
        }
        return out
    }

    // ── build factory ────────────────────────────────────────────────────────
    function makeFinger(typeId, socketId, startId) {
        var t = typeById(typeId), servos = []
        for (var i = 0; i < t.dofs.length; i++)
            servos.push({ id: startId + i, dofId: t.dofs[i].id, role: t.dofs[i].group })
        uidSeq++
        return { uid: "f" + uidSeq, typeId: typeId, socket: socketId, servos: servos }
    }

    function loadPreset(name) {
        uidSeq = 0
        var f = [], dv = {}, tq = {}, ss = {}
        function seed(fg) { seedFinger(fg, dv, tq, ss) }
        if (name === "claw") {
            // the 3-finger claw hardware, MODULAR: palm_mount + palm adapter, then
            // claw_finger on index, claw_mid on mid, claw_thumb on thumb (typed slots).
            // index and mid are the same printed part, but v67 exported them with different
            // local frames — so each slot gets its own verbatim bundle (matches claw3f.yaml).
            // servo IDs: index 1-3, thumb 4-6, mid 7-9 (matches builds/claw3f.yaml).
            mountId = "palm_mount"
            adapterId = adapterById("palm") ? "palm"
                      : (adapterTypes.length ? adapterTypes[0].id : "")
            var clawPlan = [["claw_finger", "index", 1], ["claw_thumb", "thumb", 4], ["claw_mid", "mid", 7]]
            for (var ci = 0; ci < clawPlan.length; ci++) {
                var cfg = makeFinger(clawPlan[ci][0], clawPlan[ci][1], clawPlan[ci][2]); f.push(cfg); seed(cfg)
            }
        } else {
            adapterId = ""
        }
        fingers = f; dofValues = dv; torque = tq; servoStatus = ss
        pushBuild()
    }

    // ── control actions ──────────────────────────────────────────────────────
    function setDof(uid, dofId, val) { var dv = Object.assign({}, dofValues); dv[uid + ":" + dofId] = val; dofValues = dv; pushSim(uid, dofId, val) }
    function dofValue(uid, dofId, home) { var k = uid + ":" + dofId; return dofValues[k] !== undefined ? dofValues[k] : home }
    // torque + status come from the driver telemetry when the bridge is live; mock otherwise
    function isTorque(uid, id) {
        var t = telemetry[id]
        if (t !== undefined) return t.torque
        var k = uid + ":" + id; return torque[k] !== undefined ? torque[k] : true
    }
    function statusOf(uid, id) {
        var t = telemetry[id]
        if (t !== undefined) return !t.online ? "offline" : (t.temp >= tempHotC ? "hot" : "online")
        var k = uid + ":" + id; return servoStatus[k] !== undefined ? servoStatus[k] : "online"
    }
    function toggleTorque(uid, id) {
        if (typeof handlab !== "undefined" && telemetry[id] !== undefined) {
            var enabling = !isTorque(uid, id)
            handlab.setTorque(id, enabling)
            if (enabling) syncDofFromTelemetry(uid, id)   // hold-current: slider follows reality
            return
        }
        var tq = Object.assign({}, torque), k = uid + ":" + id; tq[k] = !(tq[k] !== undefined ? tq[k] : true); torque = tq
    }
    // Re-engaged servos HOLD their current position (never jump) — snap the slider to it.
    // UI degrees are true joint degrees, so ticks convert directly.
    function syncDofFromTelemetry(uid, servoId) {
        var f = fingerByUid(uid), t = telemetry[servoId]
        if (!f || t === undefined) return
        for (var j = 0; j < f.servos.length; j++)
            if (f.servos[j].id === servoId) {
                var d = dofsFor(f)[j]        // effective range (user/port override), not the base type range
                if (!d) return
                var deg = Math.max(d.min, Math.min(d.max, Math.round((t.pos - 2048) * 360 / 4096)))
                var dv = Object.assign({}, dofValues); dv[uid + ":" + d.id] = deg; dofValues = dv
                return
            }
    }
    function anyTorqueOn() {
        for (var i = 0; i < fingers.length; i++)
            for (var j = 0; j < fingers[i].servos.length; j++)
                if (isTorque(fingers[i].uid, fingers[i].servos[j].id)) return true
        return false
    }
    function torqueOnAll() {
        if (typeof handlab !== "undefined") handlab.setTorqueAll(true)
        var tq = Object.assign({}, torque)
        for (var i = 0; i < fingers.length; i++)
            for (var j = 0; j < fingers[i].servos.length; j++) {
                tq[fingers[i].uid + ":" + fingers[i].servos[j].id] = true
                syncDofFromTelemetry(fingers[i].uid, fingers[i].servos[j].id)
            }
        torque = tq
    }
    // Return home = PHASED safe-home via the Bridge sequencer (curl → flex → spread with
    // settle waits — opening the curls first pulls the fingertips apart, so fingers coming
    // back from a grasp don't collide). Standalone (no bridge): immediate jump as before.
    function returnHome(uid) {
        if (typeof handlab !== "undefined" && handlab) {
            for (var k = 0; k < fingers.length; k++)
                if (fingers[k].uid === uid) { handlab.safeHome(k); return }
            return
        }
        var dv = Object.assign({}, dofValues), t = typeById(fingerByUid(uid).typeId)
        for (var i = 0; i < t.dofs.length; i++) { dv[uid + ":" + t.dofs[i].id] = t.dofs[i].home; pushSim(uid, t.dofs[i].id, t.dofs[i].home) }
        dofValues = dv
    }
    function returnHomeAll() {
        if (typeof handlab !== "undefined" && handlab) { handlab.safeHome(-1); return }
        var dv = Object.assign({}, dofValues)
        for (var i = 0; i < fingers.length; i++) { var t = typeById(fingers[i].typeId); for (var j = 0; j < t.dofs.length; j++) { dv[fingers[i].uid + ":" + t.dofs[j].id] = t.dofs[j].home; pushSim(fingers[i].uid, t.dofs[j].id, t.dofs[j].home) } }
        dofValues = dv
    }
    function torqueOffAll() {
        if (typeof handlab !== "undefined") handlab.setTorqueAll(false)
        var tq = Object.assign({}, torque)
        for (var i = 0; i < fingers.length; i++) for (var j = 0; j < fingers[i].servos.length; j++) tq[fingers[i].uid + ":" + fingers[i].servos[j].id] = false
        torque = tq
    }
    function toggleStop() {
        eStopped = !eStopped
        // FIRST: mirror the flag to Python (gates the direct teleop write path) — both directions.
        if (typeof handlab !== "undefined" && handlab) handlab.setEstopped(eStopped)
        // E-stop: everything limp. Release: torque STAYS OFF — releasing a stop must
        // never cause motion; the user re-engages explicitly ("Torque all on" / chips).
        if (eStopped) {
            if (typeof handlab !== "undefined" && handlab) handlab.stopHoming()   // abort the home sequence
            if (typeof handlab !== "undefined" && handlab) handlab.disableTeleop() // stop the hand-teleop loop (motion is gated in Python via setEstopped; display frames drop in syncPoseFrame)
            if (typeof handlab !== "undefined" && handlab) handlab.setArmed(false) // cut agent authority: a still-armed MCP client must not re-torque/drive under E-STOP
            torqueOffAll()
        }
    }

    // ── derived totals ───────────────────────────────────────────────────────
    // typeById can be null transiently (catalog falls back to the mock at teardown), so
    // every finger×type iteration guards it.
    function motorsUsed()   { var n = 0; for (var i = 0; i < fingers.length; i++) { var t = typeById(fingers[i].typeId); if (t) n += t.motors } return n }

    // Per-finger status, derived from servo + dof + e-stop state.
    function fingerStatus(f) {
        var t = typeById(f.typeId), offline = false, hot = false, anyTq = false, moving = false
        if (!t) return { label: "holding", color: Theme.green }
        for (var j = 0; j < f.servos.length; j++) {
            var st = statusOf(f.uid, f.servos[j].id)
            if (st === "offline") offline = true
            if (st === "hot") hot = true
            if (isTorque(f.uid, f.servos[j].id)) anyTq = true
        }
        for (var i = 0; i < t.dofs.length; i++)
            if (Math.round(dofValue(f.uid, t.dofs[i].id, t.dofs[i].home)) !== t.dofs[i].home) moving = true
        if (eStopped)        return { label: "e-stopped",     color: Theme.red }
        if (offline)         return { label: "servo offline", color: Theme.text3 }
        if (hot)             return { label: "hot",           color: Theme.amber }
        if (anyTq && moving) return { label: "moving",        color: Theme.accent }
        if (!anyTq)          return { label: "torque off",    color: Theme.text3 }
        return { label: "holding", color: Theme.green }
    }

    // Parts list aggregated by name (data-driven over the build).
    function partsList() {
        var out = [], m = curMount()
        if (m) out.push({ kind: "mount", name: m.name, qty: 1 })
        var ca = curAdapter()
        if (ca) out.push({ kind: "adapter", name: ca.name, qty: 1 })
        var typeCounts = {}, adapterCounts = {}
        for (var i = 0; i < fingers.length; i++) {
            var t = typeById(fingers[i].typeId)
            if (!t) continue
            typeCounts[t.name] = (typeCounts[t.name] || 0) + 1
            adapterCounts[t.adapter] = (adapterCounts[t.adapter] || 0) + 1
        }
        for (var n in typeCounts) out.push({ kind: "finger", name: n, qty: typeCounts[n] })
        for (var a in adapterCounts) out.push({ kind: "adapter", name: a, qty: adapterCounts[a] })
        if (fingers.length) out.push({ kind: "servo", name: "XL330 servo", qty: motorsUsed() })
        return out
    }

    // ════════════════════════════ LAYOUT ════════════════════════════════════
    ColumnLayout {
        anchors.fill: parent
        spacing: 0

        TopBar { Layout.fillWidth: true; controller: app }

        SplitView {
            Layout.fillWidth: true
            Layout.fillHeight: true
            orientation: Qt.Horizontal

            // draggable divider; right panel gets a generous default, viewport keeps a min width
            ViewportPane {
                controller: app
                SplitView.fillWidth: true
                SplitView.minimumWidth: 320
            }

            Rectangle {
                SplitView.preferredWidth: 560
                SplitView.minimumWidth: 420
                color: Theme.panel
                Behavior on color { ColorAnimation { duration: 180 } }

                StackLayout {
                    anchors.fill: parent
                    currentIndex: app.tabIndex
                    ControlPanel   { controller: app }   // tabIndex 0 — joint control
                    AgentPanel     { controller: app }   // tabIndex 1 — Claude's cockpit (MCP)
                    TeleopPanel    { controller: app }   // tabIndex 2 — WiLoR hand-tracking teleop
                }
            }
        }
    }

    // ── hardware connection panel (opened from the status chip) ──────────────
    function openConnectPanel() { connectPopup.open() }
    ConnectPopup { id: connectPopup; controller: app }

    // ── demo-recorder shortcuts ──────────────────────────────────────────────
    // Recording demos means one hand is in front of the tracking camera; reaching for the mouse
    // breaks the take. The free hand runs the cycle from the keyboard instead. These are the app's
    // only shortcuts, and deliberately so: E-STOP gets none, because a key pressed by reflex must
    // never have two meanings.
    Shortcut {
        sequence: "Space"
        enabled: !app.textFocus && app.demoCard !== null
        onActivated: app.demoCard.recording ? app.demoCard.stopEpisode("success")
                                            : app.demoCard.startEpisode()   // no-op unless ready
    }
    Shortcut {
        sequence: "F"
        enabled: !app.textFocus && app.demoCard !== null && app.demoCard.recording
        onActivated: app.demoCard.stopEpisode("fail")
    }
    Shortcut {
        sequence: "Backspace"
        enabled: !app.textFocus && app.demoCard !== null && !app.demoCard.recording
        onActivated: app.demoCard.discardLast()
    }

    // ── floating Agent bubble (Claude/MCP cockpit — moved off the top bar) ────
    // A deliberate, occasional action, so it launches from a floating bubble instead of a tab.
    // Bottom CENTRE, not the bottom-right corner: the right side is the panel stack, where it sat
    // on top of the poses list's own row actions (deleting a pose meant aiming around it). The
    // window centre falls in the viewport, which has nothing clickable down there.
    // The armed dot tells you at a glance whether Claude currently has motion authority.
    Rectangle {
        id: agentBubble
        z: 100
        width: 54; height: 54; radius: 27
        anchors.horizontalCenter: parent.horizontalCenter; anchors.bottom: parent.bottom
        anchors.bottomMargin: 22
        readonly property bool onAgent: app.tabIndex === 1
        readonly property bool armed: (typeof handlab !== "undefined" && handlab) ? handlab.armed : false
        color: onAgent ? Theme.accent : Theme.card
        border.color: onAgent ? Theme.accent : (bubbleHov.hovered ? Theme.accent : Theme.line)
        border.width: 1.5
        Behavior on color { ColorAnimation { duration: 150 } }

        // hover halo (soft elevation cue)
        Rectangle {
            anchors.centerIn: parent; width: parent.width + 12; height: parent.height + 12
            radius: (parent.width + 12) / 2; z: -1
            color: "transparent"; border.color: Theme.accent; border.width: 1
            opacity: bubbleHov.hovered ? 0.25 : 0.0
            Behavior on opacity { NumberAnimation { duration: 150 } }
        }

        // agent mark (Claude-style starburst)
        Text {
            anchors.centerIn: parent; text: "✳"
            color: agentBubble.onAgent ? "#ffffff" : Theme.accent
            font.pixelSize: 23; font.weight: Font.DemiBold
        }

        // armed indicator — green dot when Claude can drive
        Rectangle {
            visible: agentBubble.armed
            width: 14; height: 14; radius: 7
            anchors.right: parent.right; anchors.top: parent.top
            color: Theme.green; border.color: Theme.panel; border.width: 2
        }

        HoverHandler { id: bubbleHov; cursorShape: Qt.PointingHandCursor }
        ToolTip.visible: bubbleHov.hovered
        ToolTip.text: agentBubble.armed ? "Agent — ARMED (Claude can drive)"
                                        : "Agent — Claude drives via MCP"
        TapHandler { onTapped: app.tabIndex = 1 }

        // gentle pulse while armed (but not while you're already watching the Agent tab)
        SequentialAnimation on scale {
            running: agentBubble.armed && !agentBubble.onAgent; loops: Animation.Infinite
            NumberAnimation { to: 1.06; duration: 750; easing.type: Easing.InOutQuad }
            NumberAnimation { to: 1.0;  duration: 750; easing.type: Easing.InOutQuad }
        }
    }

}
