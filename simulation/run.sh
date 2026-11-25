#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
BUILD_DIR="$PROJECT_DIR/build"
if [[ ! -x "$BUILD_DIR/vtol_prepare" ]]; then
	echo 'Run simulation/build.sh first.' >&2
	exit 1
fi
"$BUILD_DIR/vtol_prepare" "$HOME/.local/share/vtol26/config/vehicle.json" "$PROJECT_DIR" "$BUILD_DIR"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$BUILD_DIR:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$BUILD_DIR/models:$PROJECT_DIR/models:${GZ_SIM_RESOURCE_PATH:-}"
exec gz sim -r "$@" "$BUILD_DIR/pvtol.sdf"
