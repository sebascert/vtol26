#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
POKY_DIR=${POKY_DIR:-"$PROJECT_DIR/build/sources/poky-scarthgap"}
BSP_DIR=${BSP_DIR:-"$PROJECT_DIR/build/sources/meta-raspberrypi-scarthgap"}

key_args=()
if [[ -f "$PROJECT_DIR/build/provision/authorized_keys" ]]; then
    key_args=(--ssh-public-key "$PROJECT_DIR/build/provision/authorized_keys")
fi
python3 "$PROJECT_DIR/configure.py" "$POKY_DIR" "$BSP_DIR" "${key_args[@]}"
# The upstream environment script reads unset variables during initialization.
set +u
# shellcheck source=/dev/null
source "$POKY_DIR/oe-init-build-env" "$PROJECT_DIR/build/yocto" >/dev/null
set -u
bitbake vtol-companion-image "$@"
