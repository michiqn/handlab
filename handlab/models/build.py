"""Build model: one machine configuration = a mount + mounted fingers (the source of truth)."""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class Hardware:
    model: str = "XL330-M288-T"
    ticks_per_rev: int = 4096
    port: str = "/dev/cu.usbserial-FILL_ME"
    baud: int = 57600
    # Motion caps written to every servo (XL330 unit: vel 0.229 rev/min, acc 214.577 rev/min^2).
    # Default deliberately SLOW — the first real moves on day 1 must be gentle on the tendons.
    profile_velocity: int = 60     # ~14 rev/min
    profile_acceleration: int = 20
    # Force caps (~mA): current_limit = EEPROM ceiling written at connect; goal_current = the
    # compliant press force in current-based position mode (pressing an obstacle saturates here
    # instead of stalling into an overload shutdown). Conservative defaults; tune per build.
    current_limit: int = 600
    goal_current: int = 350         # MOVE-level force cap
    hold_current: int | None = None  # if set (< goal_current): drop the cap to this once a servo
    #                                  settles ("grasp firm, hold gentle" — heat + 5V supply budget)
    # Position PID gains (RAM), pinned at every connect so tracking is deterministic. A weak Pos_P
    # (e.g. a Wizard-leftover 250 vs the 900 default) starves the position loop → fine moves fall
    # under the tendon break-loose force (dead zone). Pos_I=0: integral winds up on the elastic load.
    pos_p: int = 900
    pos_i: int = 0
    pos_d: int = 0


@dataclass
class FingerInstance:
    """A finger mounted on the machine: a type placed on a socket, driven by specific servos."""
    id: str                        # e.g. "A"
    type: str                      # finger-type name, e.g. "claw_finger"
    socket: str                    # arm socket id ("arm1"..) or adapter port id ("p1"..)
    servos: list[int]              # dynamixel ids, one per DOF (in the type's dof order)


@dataclass
class Build:
    name: str
    mount: str                     # mount-type name, e.g. "palm_mount"
    fingers: list[FingerInstance]
    adapter: str | None = None     # center adapter-type name (its ports host "p*" fingers)
    hardware: Hardware = field(default_factory=Hardware)

    @property
    def motor_count(self) -> int:
        return sum(len(f.servos) for f in self.fingers)
