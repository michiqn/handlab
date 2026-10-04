"""Session-wide offscreen Qt app for the suite.

handlab is a GUI app: QImage JPEG encoding and the QPainter ArUco overlay reach the font database,
which requires a QGuiApplication (a bare QCoreApplication aborts on drawText). Create ONE offscreen
QGuiApplication before any test, so the existing `QCoreApplication.instance() or QCoreApplication([])`
idiom in the Qt tests finds it and every QImage/QPainter path behaves as it does in the real app.
"""
import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")   # headless Qt platform (same as the QML load)


@pytest.fixture(scope="session", autouse=True)
def _qt_app():
    from PySide6.QtGui import QGuiApplication
    app = QGuiApplication.instance() or QGuiApplication([])
    yield app
