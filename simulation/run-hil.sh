#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
if [[ ! -x "$PROJECT_DIR/build/venv/bin/python3" || ! -x "$PROJECT_DIR/build/vtol_prepare" ]]; then
	echo 'Run simulation/build.sh first.' >&2
	exit 1
fi
exec "$PROJECT_DIR/build/venv/bin/python3" "$PROJECT_DIR/hil_bridge.py" "$@"
