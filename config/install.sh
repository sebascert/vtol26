#!/usr/bin/env bash
set -euo pipefail

if (($# > 1)) || [[ $# -eq 1 && ${1:-} != -f ]]; then
	echo "Usage: $0 [-f]" >&2
	exit 2
fi
source_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
: "${HOME:?HOME must identify the current home directory}"
config_dir="$HOME/.local/share/vtol26/config"
files=()
for name in vehicle.json vehicle.schema.json validate_vehicle.py requirements.txt; do
	destination="$config_dir/$name"
	if [[ ${1:-} == -f || (! -e "$destination" && ! -L "$destination") ]]; then
		files+=("$name")
	fi
done
python="$config_dir/venv/bin/python3"
if ((${#files[@]} == 0)) && [[ -x "$python" ]] &&
	"$python" -I -c 'import jsonschema' >/dev/null 2>&1; then
	echo "Configuration already installed"
	exit 0
fi

python3 -I -m venv "$config_dir/venv"
"$python" -I -m pip install --quiet --disable-pip-version-check -r "$source_dir/requirements.txt"
"$python" -I "$source_dir/validate_vehicle.py" "$source_dir/vehicle.json" >/dev/null
for name in "${files[@]}"; do
	install -m 0644 -T -- "$source_dir/$name" "$config_dir/$name"
done
echo "Configuration installed"
