"""Opt-in perf probes (HANDLAB_PERF=1) — Qt-free, zero overhead when disabled.

Two primitives: `PERF.lap(name)` times a with-block; `PERF.mark(name)` records the INTER-ARRIVAL
gap since the last mark of the same name (the "freeze" metric for camera frames). Aggregates
(count / avg / max ms) flush to stdout ~1×/s, piggybacked on a lap() exit — no own timer, no Qt.
Start the .app from a terminal to see the lines: `HANDLAB_PERF=1 dist/handlab.app/Contents/MacOS/handlab`.
"""
from __future__ import annotations

import os
import time
from contextlib import contextmanager


class _Perf:
    def __init__(self):
        self.enabled = os.environ.get("HANDLAB_PERF", "") not in ("", "0")
        self._stats: dict[str, list[float]] = {}       # name -> [count, sum_ms, max_ms]
        self._last_mark: dict[str, float] = {}
        self._last_flush = time.monotonic()

    def _add(self, name: str, ms: float) -> None:
        s = self._stats.get(name)
        if s is None:
            self._stats[name] = [1, ms, ms]
        else:
            s[0] += 1
            s[1] += ms
            if ms > s[2]:
                s[2] = ms

    @contextmanager
    def lap(self, name: str):
        if not self.enabled:
            yield
            return
        t0 = time.monotonic()
        try:
            yield
        finally:
            now = time.monotonic()
            self._add(name, (now - t0) * 1000.0)
            if now - self._last_flush >= 1.0:          # flush piggybacks on a lap exit
                self._flush(now)

    def mark(self, name: str) -> None:
        if not self.enabled:
            return
        now = time.monotonic()
        prev = self._last_mark.get(name)
        self._last_mark[name] = now
        if prev is not None:
            self._add(name, (now - prev) * 1000.0)

    def _flush(self, now: float) -> None:
        self._last_flush = now
        if not self._stats:
            return
        parts = [f"{n} n={int(c)} avg={s / c:.1f}ms max={mx:.0f}ms"
                 for n, (c, s, mx) in sorted(self._stats.items())]
        print("[perf] " + " · ".join(parts), flush=True)
        self._stats.clear()


PERF = _Perf()
