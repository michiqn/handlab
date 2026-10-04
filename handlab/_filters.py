"""Small, dependency-free signal filters shared by the control paths.

`OneEuro` was previously defined in the (now removed) MediaPipe `landmarks` module; the WiLoR
teleop path still needs it to smooth per-DOF norms, so it lives here as a standalone util with no
numpy/Qt/mediapipe dependency.
"""
from __future__ import annotations

import math


class OneEuro:
    """1€ filter — low lag at speed, low jitter at rest. Per-signal; feed dt in seconds."""
    def __init__(self, min_cutoff=1.0, beta=0.02, dcutoff=1.0):
        self.min_cutoff, self.beta, self.dcutoff = min_cutoff, beta, dcutoff
        self._x = None
        self._dx = 0.0

    @staticmethod
    def _alpha(cutoff, dt):
        tau = 1.0 / (2 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / max(dt, 1e-6))

    def __call__(self, x: float, dt: float) -> float:
        if self._x is None:
            self._x = x
            return x
        dx = (x - self._x) / max(dt, 1e-6)
        a_d = self._alpha(self.dcutoff, dt)
        self._dx = a_d * dx + (1 - a_d) * self._dx
        cutoff = self.min_cutoff + self.beta * abs(self._dx)
        a = self._alpha(cutoff, dt)
        self._x = a * x + (1 - a) * self._x
        return self._x
