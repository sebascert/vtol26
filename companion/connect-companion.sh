#!/usr/bin/env bash
set -euo pipefail

if (($# > 1)); then
	echo "Usage: $0 [ethernet-interface]" >&2
	exit 2
fi

interface=${1:-enp46s0}
profile=vtol-companion

if [[ $(nmcli -g GENERAL.TYPE device show "$interface") != ethernet ]]; then
	echo "Expected an Ethernet interface: $interface" >&2
	exit 1
fi

settings=(
	connection.interface-name "$interface"
	connection.autoconnect yes connection.autoconnect-priority 100
	ipv4.method manual ipv4.addresses 192.168.76.1/24
	ipv4.gateway "" ipv4.dns "" ipv4.routes "" ipv4.never-default yes
	ipv6.method disabled
)

if nmcli connection show "$profile" >/dev/null 2>&1; then
	sudo nmcli connection modify "$profile" "${settings[@]}"
elif nmcli connection show vtol-direct >/dev/null 2>&1; then
	sudo nmcli connection modify vtol-direct connection.id "$profile" "${settings[@]}"
else
	sudo nmcli connection add type ethernet con-name "$profile" "${settings[@]}"
fi

sudo nmcli connection up "$profile" ifname "$interface"
echo "Ethernet configured: laptop 192.168.76.1/24, Pi 192.168.76.2."
destination=vtol@192.168.76.2
ssh_options=(-T -o ConnectTimeout=10 -o PreferredAuthentications=password
	-o PasswordAuthentication=yes -o PubkeyAuthentication=no)

if read -r -p "Set up SSH key login 'vtol-companion' in ~/.ssh/config? [y/N] " answer &&
	[[ ${answer,,} == y || ${answer,,} == yes ]]; then
	identity="$HOME/.ssh/vtol-companion_ed25519"
	if [[ ! -f "$identity" ]]; then
		(
			umask 077
			mkdir -p "$HOME/.ssh"
		)
		ssh-keygen -q -t ed25519 -N '' -C vtol-companion -f "$identity"
	fi
	key_options=(-T -o ConnectTimeout=10 -o IdentitiesOnly=yes -i "$identity"
		-o PreferredAuthentications=publickey -o PubkeyAuthentication=yes
		-o PasswordAuthentication=no -o BatchMode=yes)
	if ! ssh "${key_options[@]}" "$destination" true </dev/null 2>/dev/null; then
		echo "Installing the laptop's public key. Enter the Pi password: vtol"
		public_key=$(ssh-keygen -y -f "$identity")
		printf '%s\n' "$public_key" | ssh "${ssh_options[@]}" "$destination" 'set -eu
key=$(cat)
umask 077
mkdir -p "$HOME/.ssh"
touch "$HOME/.ssh/authorized_keys"
grep -qxF "$key" "$HOME/.ssh/authorized_keys" || printf "%s\n" "$key" >> "$HOME/.ssh/authorized_keys"
chmod 700 "$HOME/.ssh"
chmod 600 "$HOME/.ssh/authorized_keys"
# Yocto uses a new sshd process for each socket-activated connection.
sudo -n sed -i "s/^PubkeyAuthentication no$/PubkeyAuthentication yes/" /etc/ssh/sshd_config
sudo -n /usr/sbin/sshd -t
'
	fi
	python3 - "$HOME/.ssh/config" "$identity" <<'PYTHON'
from pathlib import Path
import json
import os
import shlex
import sys
import tempfile

config = Path(sys.argv[1]).resolve()
lines = config.read_text().splitlines(keepends=True) if config.exists() else []
kept = []
skip = False
for line in lines:
    fields = shlex.split(line, comments=True)
    if fields and fields[0].lower() in ("host", "match"):
        skip = False
        if fields[0].lower() == "host" and any(name.lower() in ("rasp", "vtol-companion") for name in fields[1:]):
            aliases = [name for name in fields[1:] if name.lower() not in ("rasp", "vtol-companion")]
            if not aliases:
                skip = True
                continue
            line = "Host " + " ".join(aliases) + "\n"
    if not skip:
        kept.append(line)
profile = f"""Host vtol-companion
    HostName 192.168.76.2
    User vtol
    IdentityFile {json.dumps(sys.argv[2], ensure_ascii=False)}
    IdentitiesOnly yes
    PreferredAuthentications publickey
    PubkeyAuthentication yes
    PasswordAuthentication no
    ConnectTimeout 10
"""
config.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
with tempfile.NamedTemporaryFile(mode="w", dir=config.parent, delete=False) as output:
    temporary = Path(output.name)
    try:
        output.write("".join(kept).rstrip() + "\n\n" + profile)
        output.close()
        os.replace(temporary, config)
    finally:
        temporary.unlink(missing_ok=True)
PYTHON
	destination=vtol-companion
	ssh_options=("${key_options[@]}")
fi

echo "Testing SSH and project installation. Username: vtol; password: vtol"
ssh "${ssh_options[@]}" "$destination" 'sh -se' <<'REMOTE'
check() {
    label=$1
    shift
    if "$@"; then
        printf 'PASS: %s\n' "$label"
    else
        printf 'FAIL: %s\n' "$label" >&2
        exit 1
    fi
}

check "SSH login as vtol" test "$(id -un)" = vtol
check "Allocator executable" test -x /usr/bin/vtol_allocator
check "PSO configuration readable" test -r /etc/vtol/pso.json
check "PSO configuration nonempty" test -s /etc/vtol/pso.json
check "UART character device" test -c /dev/serial0
check "UART readable" test -r /dev/serial0
check "UART writable" test -w /dev/serial0
check "Allocator service enabled" systemctl is-enabled --quiet vtol_allocator@vtol.service
check "Allocator service running" systemctl is-active --quiet vtol_allocator@vtol.service

ls -l /usr/bin/vtol_allocator /etc/vtol/pso.json /dev/serial0
echo "Recent allocator logs:"
sudo -n journalctl -u vtol_allocator@vtol.service -n 10 --no-pager
REMOTE
echo "SSH and project validation passed."
echo "Connect: ssh $destination"
