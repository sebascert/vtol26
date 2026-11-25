#!/usr/bin/env bash
set -euo pipefail

# Mock privileged commands so installation behavior can be checked on a host.
installer=$1
fixture=$(mktemp -d "$2/install-test.XXXXXX")
trap 'rm -rf -- "$fixture"' EXIT
mkdir -p "$fixture/bin" "$fixture/project/companion/allocator/deploy" "$fixture/project/companion/allocator/build" "$fixture/project/config"
cp -- "$installer" "$fixture/project/companion/allocator/deploy/install.sh"
export INSTALL_TEST_LOG="$fixture/commands.log"
cat >"$fixture/project/config/install.sh" <<'MOCK'
#!/usr/bin/env bash
printf '%s\n' 'user config/install.sh' >> "$INSTALL_TEST_LOG"
MOCK
chmod +x "$fixture/project/config/install.sh"

export INSTALL_TEST_CONFIG=missing
export INSTALL_TEST_FAIL=0
export INSTALL_TEST_LEGACY_SERVICE=absent
cat >"$fixture/bin/sudo" <<'MOCK'
#!/usr/bin/env bash
printf '%s\n' "sudo $*" >> "$INSTALL_TEST_LOG"
if [[ $1 == test ]]; then [[ $INSTALL_TEST_CONFIG == existing ]]; else exit 0; fi
MOCK
cat >"$fixture/bin/id" <<'MOCK'
#!/usr/bin/env bash
echo pi
MOCK
cat >"$fixture/bin/cmake" <<'MOCK'
#!/usr/bin/env bash
printf '%s\n' "cmake $*" >> "$INSTALL_TEST_LOG"
MOCK
cat >"$fixture/bin/ctest" <<'MOCK'
#!/usr/bin/env bash
printf '%s\n' ctest >> "$INSTALL_TEST_LOG"
exit "$INSTALL_TEST_FAIL"
MOCK
cat >"$fixture/bin/systemctl" <<'MOCK'
#!/usr/bin/env bash
[[ $2 == mavlink-uart@pi.service ]] || exit 1
case "$1:$INSTALL_TEST_LEGACY_SERVICE" in
    is-enabled:enabled|is-active:active) exit 0 ;;
    *) exit 1 ;;
esac
MOCK
chmod +x "$fixture/bin/"*
export PATH="$fixture/bin:$PATH"
# The test may run as root in CI; only the child installer's UID check is replaced.
sed -i 's/(( EUID == 0 ))/(( 0 ))/' "$fixture/project/companion/allocator/deploy/install.sh"

bash "$fixture/project/companion/allocator/deploy/install.sh" >"$fixture/output.log"
if grep -Eq '^sudo apt(-get)? ' "$INSTALL_TEST_LOG"; then
	echo 'Installer tried to manage system packages' >&2
	exit 1
fi
grep -Fq 'sudo install -m 0644' "$INSTALL_TEST_LOG"
grep -Fq '/src/pso/config.json /usr/local/share/' "$INSTALL_TEST_LOG"
grep -Fq 'user config/install.sh' "$INSTALL_TEST_LOG"
grep -Fq 'sudo systemctl enable vtol_allocator@pi.service' "$INSTALL_TEST_LOG"
grep -Fq '/build/vtol_allocator /usr/local/bin/vtol_allocator' "$INSTALL_TEST_LOG"
grep -Fq '/deploy/vtol_allocator@.service /usr/local/lib/systemd/system/vtol_allocator@.service' "$INSTALL_TEST_LOG"
if grep -Fq 'disable --now' "$INSTALL_TEST_LOG"; then
	echo 'Installer tried to disable an absent legacy service' >&2
	exit 1
fi

export INSTALL_TEST_CONFIG=existing
: >"$INSTALL_TEST_LOG"
bash "$fixture/project/companion/allocator/deploy/install.sh" >"$fixture/output.log"
if grep -Fq '/src/pso/config.json /usr/local/share/' "$INSTALL_TEST_LOG"; then
	echo 'Installer overwrote existing calibration config' >&2
	exit 1
fi
grep -Fq 'Preserving existing config' "$fixture/output.log"

for state in enabled active; do
	export INSTALL_TEST_LEGACY_SERVICE=$state
	: >"$INSTALL_TEST_LOG"
	bash "$fixture/project/companion/allocator/deploy/install.sh" >"$fixture/output.log"
	grep -Fq 'sudo systemctl disable --now mavlink-uart@pi.service' "$INSTALL_TEST_LOG"
	awk '/sudo systemctl disable --now/ { stopped=1 }
         /sudo systemctl enable vtol_allocator@pi.service/ { if (!stopped) exit 1; enabled=1 }
         END { if (!enabled) exit 1 }' "$INSTALL_TEST_LOG"
done

export INSTALL_TEST_FAIL=1
: >"$INSTALL_TEST_LOG"
if bash "$fixture/project/companion/allocator/deploy/install.sh" >"$fixture/output.log"; then
	echo 'Installer ignored failing tests' >&2
	exit 1
fi
if grep -Eq '^sudo (install|usermod|systemctl)' "$INSTALL_TEST_LOG"; then
	echo 'Installer deployed despite failing tests' >&2
	exit 1
fi
