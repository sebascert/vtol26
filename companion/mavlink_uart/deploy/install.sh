#!/usr/bin/env bash
set -euo pipefail

if [[ ${1:-} == --help ]]; then
    echo "Usage: $0"
    echo 'Run as your normal Raspberry Pi OS user; sudo is used for system installation.'
    echo 'Installs dependencies, builds/tests, preserves existing PSO config and enables autostart.'
    exit 0
fi
if (( $# != 0 )); then
    echo "Unknown argument: $1 (use --help)" >&2
    exit 1
fi
if (( EUID == 0 )); then
    echo 'Run this script as your normal user, without sudo.' >&2
    exit 1
fi
for command in sudo apt-get systemctl; do
    if ! command -v "$command" >/dev/null; then
        echo "Required command missing: $command (Raspberry Pi OS with systemd is required)." >&2
        exit 1
    fi
done

source_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
build_dir="$source_dir/build"
service="mavlink-uart@$(id -un).service"
config_dir=/usr/local/share/vtol26/companion/mavlink_uart/src/pso

sudo -v
sudo apt-get update
sudo apt-get install -y --no-install-recommends build-essential cmake ca-certificates

cmake -S "$source_dir" -B "$build_dir" -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build "$build_dir" --parallel 2
(cd -- "$build_dir" && ctest --output-on-failure)

sudo install -d /usr/local/bin /usr/local/lib/systemd/system "$config_dir"
sudo install -m 0755 "$build_dir/mavlink_uart" /usr/local/bin/mavlink_uart
if ! sudo test -e "$config_dir/config.json"; then
    sudo install -m 0644 "$source_dir/src/pso/config.json" "$config_dir/config.json"
else
    echo "Preserving existing config: $config_dir/config.json"
fi
sudo install -m 0644 "$source_dir/deploy/mavlink-uart@.service" /usr/local/lib/systemd/system/mavlink-uart@.service
sudo usermod --append --groups dialout "$(id -un)"
sudo systemctl daemon-reload
sudo systemctl enable "$service"

if [[ -c /dev/serial0 ]]; then
    sudo systemctl restart "$service"
    sudo systemctl is-active --quiet "$service"
else
    echo 'Service enabled. Enable UART and disable its serial login console using sudo raspi-config, then reboot.'
fi
printf 'Installed. Logs: journalctl -u %s -f\n' "$service"
echo 'The service reports model allocations; aircraft calibration and physical output mapping are still required.'
