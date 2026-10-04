"""Bridge mixin: hardware connection, calibration, torque, recovery.

Slot/helper mixin for the Bridge (moved verbatim out of bridge.py). ONLY @Slot methods and
plain helpers live here — @Property/@Signal stay in bridge.py: PySide6 does not wire
Property(notify=...) across mixin boundaries (verified empirically), and signals must be
declared on the QObject subclass. Methods here use self.<signal>/self.<state> at runtime.
"""
from __future__ import annotations

import json
import os

from PySide6.QtCore import Slot

from . import library_io
from .hal.mock_driver import MockDriver
from .hal.mujoco_driver import MuJoCoDriver


class BridgeHardwareMixin:

    def _make_driver(self) -> None:
        # a live hardware connection survives structural rebuilds — swap the map, keep the port
        from .hal.dynamixel_driver import DynamixelDriver
        if isinstance(self._driver, DynamixelDriver) and self._driver._port is not None:
            self._driver._map = self._servo_map
            self._driver.set_calibration(library_io.load_calibration())
            self._online = set(self._driver.ping_all())
            return

        choice = os.environ.get("HANDLAB_DRIVER", "auto")
        driver = None
        if choice == "dynamixel":
            try:
                driver = self._new_dynamixel()
                driver.connect(self._build.hardware.port, self._build.hardware.baud)
                driver.home_at_connect()            # absolute home where stored, else delta-home
            except Exception as e:
                print(f"[handlab] dynamixel driver failed ({e}); falling back to sim.")
                driver = None
        if driver is None and choice != "mock" and self._twin is not None:
            driver = MuJoCoDriver(self._servo_map, self._twin)
            driver.connect("", 0)
        if driver is None:
            driver = MockDriver(self._servo_map)
            driver.connect("", 0)
        self._driver = driver
        self._online = set(driver.ping_all())
        if not isinstance(driver, DynamixelDriver):
            for s in self._servo_map.servos:    # sim servos boot holding; hardware stays limp
                driver.set_torque(s.id, True)

    def _new_dynamixel(self):
        from .hal.dynamixel_driver import DynamixelDriver
        hw = self._build.hardware
        return DynamixelDriver(self._servo_map, calibration=library_io.load_calibration(),
                               profile_velocity=hw.profile_velocity,
                               profile_acceleration=hw.profile_acceleration,
                               current_limit=hw.current_limit, goal_current=hw.goal_current,
                               hold_current=hw.hold_current,
                               pos_p=hw.pos_p, pos_i=hw.pos_i, pos_d=hw.pos_d)

    @Slot(result=str)
    def scanPorts(self) -> str:
        import glob
        return json.dumps(sorted(glob.glob("/dev/cu.usbserial*") + glob.glob("/dev/cu.usbmodem*")))

    @Slot(str, int, result=str)
    def connectHardware(self, port: str, baud: int) -> str:
        try:
            drv = self._new_dynamixel()
            drv.connect(port, int(baud))
            # Per-servo home: stored absolute home_shaft where confident, delta-home the rest
            # (spread+flex have no return spring, so delta-home alone drifts; absolute recovers it).
            home = drv.home_at_connect()
        except Exception as e:
            return json.dumps({"ok": False, "error": str(e)})
        self._driver = drv                      # servos stay torque OFF until the user engages
        self._online = set(drv.ping_all())
        self.structureChanged.emit()            # driverName updates
        out = {"ok": True, "online": sorted(self._online), "home": home}
        if home.get("near_flip"):               # a stored-home joint was too near the ±180° flip
            out["warn"] = ("Home uncertain (near ±180°) for servo " + str(home["near_flip"]) +
                           " — straighten the joint and press 'Set home' again.")
        return json.dumps(out)

    @Slot(result=str)
    def disconnectHardware(self) -> str:
        from .hal.dynamixel_driver import DynamixelDriver
        if isinstance(self._driver, DynamixelDriver):
            try:
                self._driver.disconnect()       # leaves the hand limp
            except Exception:
                pass
            self._driver = None
            self._make_driver()                 # back to the sim driver
            self.structureChanged.emit()
        return json.dumps({"ok": True})

    @Slot(result=str)
    def setAbsoluteHome(self) -> str:
        """Hand held at the true home pose (stretched): store each servo's ABSOLUTE encoder angle as
        the PERSISTENT home reference. Thereafter every connect recovers home from it — no matter
        where the hand was left (spread+flex have no return spring, so delta-home alone drifts).
        Human-only (Connect panel); reads only, moves nothing."""
        from .hal.dynamixel_driver import DynamixelDriver
        if not isinstance(self._driver, DynamixelDriver):
            return json.dumps({"ok": False, "error": "connect the hardware first"})
        try:
            home = self._driver.capture_home_shaft()
            library_io.save_calibration(self._driver.calibration())   # persists sign + home_shaft
        except Exception as e:
            return json.dumps({"ok": False, "error": str(e)})
        return json.dumps({"ok": True, "home_shaft": {str(k): v for k, v in home.items()}})

    @Slot(int, result=str)
    def setAbsoluteHomeFinger(self, finger_ord: int) -> str:
        """Set the absolute home for ONE finger only (the others keep theirs) — e.g. anchor a
        freshly-strung thumb without re-homing the calibrated index."""
        from .hal.dynamixel_driver import DynamixelDriver
        if not isinstance(self._driver, DynamixelDriver):
            return json.dumps({"ok": False, "error": "connect the hardware first"})
        if not (0 <= finger_ord < len(self._build.fingers)):
            return json.dumps({"ok": False, "error": "bad finger"})
        ids = {int(s) for s in self._build.fingers[finger_ord].servos}
        try:
            home = self._driver.capture_home_shaft(only_ids=ids)
            library_io.save_calibration(self._driver.calibration())
        except Exception as e:
            return json.dumps({"ok": False, "error": str(e)})
        return json.dumps({"ok": True, "servos": sorted(ids),
                           "home_shaft": {str(k): v for k, v in home.items() if k in ids}})

    @Slot(int, result=str)
    def rebootServo(self, servo_id: int) -> str:
        """Revive a servo after a latched hardware error (⚠ overload/overheat): in-band REBOOT
        clears reg 70, then the driver re-applies mode + caps. Torque stays OFF and the servo's
        home offset is stale afterwards (position re-wraps) → set the finger's home next."""
        from .hal.dynamixel_driver import DynamixelDriver
        if not isinstance(self._driver, DynamixelDriver):
            return json.dumps({"ok": False, "error": "connect the hardware first"})
        try:
            self._driver.reboot(int(servo_id))
        except Exception as e:
            return json.dumps({"ok": False, "error": str(e)})
        return json.dumps({"ok": True,
                           "hint": "Servo rebooted (torque off). Re-anchor the finger with Set home, then turn torque back on."})

    def _set_signs(self, ids: list[int], sign: int) -> str:
        """Shared body of the three sign slots: guard for real hardware, flip every id, persist
        to calibration.yaml (a hardware fact that survives restarts)."""
        from .hal.dynamixel_driver import DynamixelDriver
        if not isinstance(self._driver, DynamixelDriver):
            return json.dumps({"ok": False, "error": "connect the hardware first"})
        try:
            for sid in ids:
                self._driver.set_sign(sid, int(sign))
            library_io.save_calibration(self._driver.calibration())
        except Exception as e:
            return json.dumps({"ok": False, "error": str(e)})
        return json.dumps({"ok": True, "servos": ids, "sign": int(sign)})

    @Slot(int, int, result=str)
    def setServoSign(self, servo_id: int, sign: int) -> str:
        """Invert a servo's direction (discovered on day 1 if an axis runs backwards)."""
        return self._set_signs([int(servo_id)], sign)

    @Slot(int, int, result=str)
    def setFingerSign(self, finger_ord: int, sign: int) -> str:
        """Set the HARDWARE direction for a WHOLE finger at once (all its servos → sign). The
        common case: the real finger runs mirror-inverted vs the twin, so flip every servo of it
        in one click. (Hardware direction only — never touches the twin; the twin's direction is
        the CAD/Fusion export.)"""
        if not (0 <= finger_ord < len(self._build.fingers)):
            return json.dumps({"ok": False, "error": "bad finger"})
        return self._set_signs([int(s) for s in self._build.fingers[finger_ord].servos], sign)

    @Slot(int, result=str)
    def setAllSigns(self, sign: int) -> str:
        """Set the HARDWARE direction for the WHOLE hand (every servo → sign) — one click when
        the entire build is wired the same way vs the twin."""
        return self._set_signs([int(s.id) for s in self._servo_map.servos], sign)

    @Slot(int, bool)
    def setTorque(self, servo_id: int, enable: bool) -> None:
        if self._driver is not None:
            self._driver.set_torque(servo_id, enable)

    @Slot(bool)
    def setTorqueAll(self, enable: bool) -> None:
        # per-servo guard: one broken/latched servo must NEVER abort the loop for the rest —
        # this is also the E-STOP path (torque off all)
        if self._driver is not None:
            for s in self._servo_map.servos:
                try:
                    self._driver.set_torque(s.id, enable)
                except Exception:
                    pass
