#!/usr/bin/env bash
set -euo pipefail

launcher=$1
fixture=$(mktemp -d "$2/run-companion-test.XXXXXX")
trap 'rm -rf -- "$fixture"' EXIT
source_dir="$fixture/project with spaces"
mkdir -p "$source_dir/deploy" "$source_dir/build"
cp -- "$launcher" "$source_dir/deploy/run-companion.sh"
cat >"$source_dir/build/vtol_allocator" <<'MOCK'
#!/usr/bin/env bash
printf '%s\n' "$@"
MOCK
chmod +x "$source_dir/build/vtol_allocator"

mapfile -t args < <(cd / && bash "$source_dir/deploy/run-companion.sh")
[[ ${#args[@]} == 6 ]]
[[ ${args[0]} == --port && ${args[1]} == /dev/serial/by-id/usb-Auterion_PX4_FMU_v6C.x_0-if00 ]]
[[ ${args[2]} == --baud && ${args[3]} == 115200 ]]
[[ ${args[4]} == --pso-config && ${args[5]} == "$source_dir/src/pso/config.json" ]]

mapfile -t args < <(cd / && bash "$source_dir/deploy/run-companion.sh" \
	--port /dev/serial0 --baud 57600 --pso-config "/config with spaces.json")
[[ ${#args[@]} == 12 ]]
[[ ${args[6]} == --port && ${args[7]} == /dev/serial0 ]]
[[ ${args[8]} == --baud && ${args[9]} == 57600 ]]
[[ ${args[10]} == --pso-config && ${args[11]} == "/config with spaces.json" ]]
