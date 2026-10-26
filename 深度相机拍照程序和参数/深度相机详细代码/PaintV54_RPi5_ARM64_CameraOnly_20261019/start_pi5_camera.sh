#!/usr/bin/env bash
# Convenience wrapper: does not activate conda or install anything.
# Set PI_ORBBEC_SDK_ROOT to your Pi-specific verified aarch64 legacy SDK root.
set -euo pipefail
HERE="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
PY39="${PI_CAMERA_PYTHON:-python3.9}"
exec "$PY39" -B "$HERE/pi5_capture_only.py" capture "$@"
