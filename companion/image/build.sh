#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
SOURCES_DIR="$PROJECT_DIR/build/sources"
POKY_DIR=${POKY_DIR:-"$SOURCES_DIR/poky-scarthgap"}
BSP_DIR=${BSP_DIR:-"$SOURCES_DIR/meta-raspberrypi-scarthgap"}

if ((EUID == 0)); then
	echo "Run this script as your normal user; BitBake must not run as root." >&2
	exit 1
fi
mkdir -p "$PROJECT_DIR/build"
if ! python3 -c 'import locale; locale.setlocale(locale.LC_ALL, "en_US.UTF-8")' 2>/dev/null; then
	echo "Generating the en_US.UTF-8 host locale (requires sudo)." >&2
	sudo locale-gen en_US.UTF-8
fi
export LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

if ! unshare -Ur true 2>/dev/null; then
	if [[ ${VTOL_IMAGE_USERNS:-0} == 1 ]] ||
		[[ $(sysctl -n kernel.apparmor_restrict_unprivileged_userns 2>/dev/null || true) != 1 ]]; then
		echo "User namespaces are unavailable; check host policy before running BitBake." >&2
		exit 1
	fi
	profile_name="vtol26-bitbake-$UID-$$"
	profile_file="$PROJECT_DIR/build/$profile_name.apparmor"
	cat >"$profile_file" <<PROFILE
abi <abi/4.0>,
include <tunables/global>
profile $profile_name flags=(unconfined) {
    userns,
}
PROFILE
	echo "Loading a temporary AppArmor profile for this build (requires sudo)." >&2
	sudo apparmor_parser --skip-cache -r "$profile_file"
	trap '
        if ! sudo apparmor_parser --skip-cache -R "$profile_file"; then
            echo "Could not unload temporary profile: $profile_name" >&2
        fi
    ' EXIT
	aa-exec -p "$profile_name" -- env VTOL_IMAGE_USERNS=1 bash "$PROJECT_DIR/build.sh" "$@"
	exit 0
fi

python3 - "$PROJECT_DIR/build" <<'PY'
import shutil, sys
free = shutil.disk_usage(sys.argv[1]).free
if free < 100_000_000_000:
    print(f"Warning: {free / 1e9:.1f} GB free; 100 GB is recommended for a full Yocto build.", file=sys.stderr)
PY

fetch_layer() {
	local name=$1 url=$2 destination="$SOURCES_DIR/$1"
	if [[ -e "$destination" || -L "$destination" ]]; then return; fi
	mkdir -p "$SOURCES_DIR"
	local archive="$SOURCES_DIR/$name.tar.gz" unpack
	if [[ ! -f "$archive" ]]; then
		wget -O "$archive.part" "$url"
		mv "$archive.part" "$archive"
	fi
	(
		cd "$SOURCES_DIR"
		if [[ -f "$name.tar.gz.sha256" ]]; then
			sha256sum -c "$name.tar.gz.sha256"
		else
			sha256sum "$name.tar.gz" >"$name.tar.gz.sha256"
		fi
	)
	unpack=$(mktemp -d "$SOURCES_DIR/.unpack.XXXXXX")
	tar -xzf "$archive" -C "$unpack"
	mv -T "$unpack/$name" "$destination"
	rmdir "$unpack"
}
if [[ "$POKY_DIR" == "$SOURCES_DIR/poky-scarthgap" ]]; then
	fetch_layer poky-scarthgap https://github.com/yoctoproject/poky/archive/refs/heads/scarthgap.tar.gz
fi
if [[ "$BSP_DIR" == "$SOURCES_DIR/meta-raspberrypi-scarthgap" ]]; then
	fetch_layer meta-raspberrypi-scarthgap https://github.com/agherzan/meta-raspberrypi/archive/refs/heads/scarthgap.tar.gz
fi

python3 "$PROJECT_DIR/configure.py" "$POKY_DIR" "$BSP_DIR"
# The upstream environment script reads unset variables during initialization.
set +u
# shellcheck source=/dev/null
source "$POKY_DIR/oe-init-build-env" "$PROJECT_DIR/build/yocto" >/dev/null
set -u
bitbake vtol-companion-image "$@"
