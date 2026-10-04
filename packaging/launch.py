"""PyInstaller entry point — boots the handlab app (see packaging/handlab.spec)."""

import sys

from handlab.app import main

if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
