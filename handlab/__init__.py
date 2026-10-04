"""handlab — modular, config-driven control center + digital twin for tendon-driven hands/arms.

Native desktop app: Python logic + PySide6/QML UI in one process; the 3D twin is MuJoCo
rendered offscreen and shown in QML. A machine is assembled from a *mount* (chassis) and
*fingers* (appendage types from a library), composed into a MuJoCo scene + a servo map.

Heavy deps (PySide6, mujoco, yaml) are imported lazily so `import handlab` always works.
"""

__version__ = "0.0.1"
