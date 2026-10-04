"""Bridge mixin: viewport camera + sim-playground (physics/drop).

Slot/helper mixin for the Bridge (moved verbatim out of bridge.py). ONLY @Slot methods and
plain helpers live here — @Property/@Signal stay in bridge.py: PySide6 does not wire
Property(notify=...) across mixin boundaries (verified empirically), and signals must be
declared on the QObject subclass. Methods here use self.<signal>/self.<state> at runtime.
"""
from __future__ import annotations

from PySide6.QtCore import Slot

from . import compose


class BridgeViewMixin:

    @Slot(float)
    def zoom(self, factor: float) -> None:
        if self._twin is not None:
            self._twin.zoom(factor)

    @Slot(float, float)
    def orbit(self, d_az: float, d_el: float) -> None:
        if self._twin is not None:
            self._twin.orbit(d_az, d_el)
            self._cam_dirty = True          # gizmo re-reads az/el on the next frame

    @Slot(float, float)
    def pan(self, dx: float, dy: float) -> None:
        if self._twin is not None:
            self._twin.pan(dx, dy)

    @Slot(float)
    def panX(self, delta: float) -> None:
        """Pan locked to the world X axis (⌥/Option from the viewport)."""
        if self._twin is not None:
            self._twin.pan_x(delta)

    @Slot()
    def resetView(self) -> None:
        if self._twin is not None:
            self._twin.reset_view()
            self._cam_dirty = True

    def _cam_orientation(self) -> tuple:
        """Cached az/el — the gizmo polls per rendered frame; without the cache that was 2 extra
        `twin._lock` hits/frame (contending with the physics thread's up-to-50 ms step batches)."""
        if self._cam_dirty and self._twin is not None:
            self._cam_cache = tuple(self._twin.camera_orientation())
            self._cam_dirty = False
        return self._cam_cache

    @Slot(result=float)
    def camAzimuth(self) -> float:
        return self._cam_orientation()[0] if self._twin is not None else 0.0

    @Slot(result=float)
    def camElevation(self) -> float:
        return self._cam_orientation()[1] if self._twin is not None else 0.0

    @Slot(bool)
    def setPhysics(self, on: bool) -> None:
        if bool(on) == self._physics:
            return
        self._physics = bool(on)
        self._rebuild()                 # recompose scene (gravity/collisions/object) + reload twin
        self.structureChanged.emit()

    @Slot()
    def dropObject(self) -> None:
        if self._twin is not None and self._physics:
            self._twin.set_free_joint("playobj_free", compose.PLAY_SPAWN)
