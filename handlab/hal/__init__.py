"""Hardware Abstraction Layer — interchangeable drivers behind one interface.

mock      — no physics, instant (default; zero setup)
mujoco    — the composed MuJoCo twin, driven headless (real geometry, real coupling)
dynamixel — real servos over USB (pending hardware)

Ported in spirit from craft-control; the mujoco driver maps each servo to its PREFIXED
actuator ("A_curl") so EVERY configured finger drives the sim (no "first finger wins").
"""

from .base import HandDriver

__all__ = ["HandDriver"]
