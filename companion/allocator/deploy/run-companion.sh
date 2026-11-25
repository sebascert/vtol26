#!/usr/bin/env bash
set -euo pipefail

source_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
exec "$source_dir/build/vtol_allocator" \
	--port /dev/serial/by-id/usb-Auterion_PX4_FMU_v6C.x_0-if00 --baud 115200 \
	--pso-config "$source_dir/src/pso/config.json" "$@"
