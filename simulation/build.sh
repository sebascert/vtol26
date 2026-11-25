#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
BUILD_DIR="$PROJECT_DIR/build"
/usr/bin/python3 -m venv --system-site-packages "$BUILD_DIR/venv"
"$BUILD_DIR/venv/bin/python3" -m pip install --disable-pip-version-check -r "$PROJECT_DIR/requirements.txt"
"$BUILD_DIR/venv/bin/python3" -c 'from gz.transport13 import Node; from gz.msgs10.imu_pb2 import IMU'
cmake -S "$PROJECT_DIR" -B "$BUILD_DIR" \
	-DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DBUILD_TESTING=ON "$@"
cmake --build "$BUILD_DIR" --parallel 2
"$BUILD_DIR/vtol_prepare" "$HOME/.local/share/vtol26/config/vehicle.json" "$PROJECT_DIR" "$BUILD_DIR"
