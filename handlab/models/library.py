"""Library models: mount types (chassis) and finger types (reusable appendage modules)."""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class Dof:
    """One actuated degree of freedom of a finger type."""
    key: str                       # e.g. "curl"
    label: str                     # e.g. "Curl"
    joint: str                     # MuJoCo joint name inside the type's model.xml
    range: tuple[float, float] = (0.0, 1.0)   # rad, for the position actuator ctrlrange
    coupled: bool = False          # informational (curl drives a follower via a coupling)


@dataclass
class Coupling:
    """Mechanical joint coupling (e.g. one curl tendon drives PIP and DIP 1:1)."""
    driver: str                    # joint name driven by the actuator
    follower: str                  # joint that follows
    ratio: float = 1.0             # -> <equality polycoef="0 ratio 0 0 0">


@dataclass
class FingerType:
    """A reusable, tendon-driven appendage (CRAFT finger, SpiRob arm, ...)."""
    name: str
    label: str
    motors: int
    dofs: list[Dof]
    model: str                     # path to the ACDC4Robot fragment (model.xml), relative to the type dir
    couplings: list[Coupling] = field(default_factory=list)
    tuning: dict = field(default_factory=dict)
    ui: dict = field(default_factory=dict)   # optional display hints for the QML catalog
    slot: str = ""                 # which port slot type this finger fits ("" = any port)
    dir: str = ""                  # absolute dir of this type in the library (filled by loader)


@dataclass
class Socket:
    """A standardized mounting socket on the mount (kind 'center'/'arm') or a finger
    port on an adapter (kind 'port'). `accepts` optionally restricts which finger-type
    slot fits here (e.g. the palm's thumb port only takes slot 'thumb'); empty = any."""
    id: str
    kind: str = "arm"                                     # "center" | "arm" | "port"
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)     # metres
    euler: tuple[float, float, float] = (0.0, 0.0, 0.0)   # radians
    accepts: str = ""                                     # finger slot type; "" = any
    # per-slot joint-range overrides in DEGREES {dof key: (lo, hi)} — e.g. the palm's mid
    # slot mirrors the index slot's spread travel (same printed finger, opposite direction)
    dof_ranges_deg: dict = field(default_factory=dict)


@dataclass
class MountType:
    """The base chassis (the palm base). Declares standardized sockets; the center
    socket takes an ADAPTER (see AdapterType)."""
    name: str
    label: str
    sockets: list[Socket]
    model: str | None = None       # static base geometry (STL), relative to the mount dir
    model_pos: tuple[float, float, float] = (0.0, 0.0, 0.0)     # world frame of the model
    model_euler: tuple[float, float, float] = (0.0, 0.0, 0.0)   # (mesh is in link coords)
    dir: str = ""


@dataclass
class AdapterType:
    """A swappable coupler that mates the mount's standardized norm (center for now) and
    carries 1..N finger ports. The user designs adapters per finger count (1-finger today,
    2-finger for finger+thumb next, ...). Geometry only — control stays per-finger."""
    name: str
    label: str
    ports: list[Socket]            # finger ports (kind="port"), frames relative to the socket
    fits: str = "center"           # which mount socket norm this adapter plugs into
    mesh: str | None = None        # STL rendered as static geom, relative to the adapter dir
    mesh_scale: float = 0.001      # mm -> m (matches the finger mesh convention)
    mesh_pos: tuple[float, float, float] = (0.0, 0.0, 0.0)     # tweak after visual check
    mesh_euler: tuple[float, float, float] = (0.0, 0.0, 0.0)
    dir: str = ""
