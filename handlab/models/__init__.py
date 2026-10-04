"""Plain-dataclass domain models (stdlib only — import-safe without any third-party deps)."""

from .library import Socket, Coupling, Dof, FingerType, MountType, AdapterType
from .build import Build, FingerInstance, Hardware
from .servo_map import Servo, ServoMap

__all__ = [
    "Socket", "Coupling", "Dof", "FingerType", "MountType", "AdapterType",
    "Build", "FingerInstance", "Hardware",
    "Servo", "ServoMap",
]
