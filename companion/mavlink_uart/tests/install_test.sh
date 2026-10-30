#!/usr/bin/env bash
set -euo pipefail

# Mock privileged commands so installation behavior can be checked on a host.
installer=$1
fixture=$(mktemp -d "$2/install-test.XXXXXX")
trap 'rm -rf -- "$fixture"' EXIT
mkdir -p "$fixture/bin" "$fixture/project/deploy" "$fixture/project/build"
cp -- "$installer" "$fixture/project/deploy/install.sh"
export INSTALL_TEST_LOG="$fixture/commands.log"
export INSTALL_TEST_CONFIG=missing
export INSTALL_TEST_FAIL=0
cat > "$fixture/bin/sudo" <<'MOCK'
#!/usr/bin/env bash
printf '%s\n' "sudo $*" >> "$INSTALL_TEST_LOG"
if [[ $1 == test ]]; then [[ $INSTALL_TEST_CONFIG == existing ]]; else exit 0; fi
MOCK
cat > "$fixture/bin/id" <<'MOCK'
#!/usr/bin/env bash
echo pi
MOCK
cat > "$fixture/bin/cmake" <<'MOCK'
#!/usr/bin/env bash
printf '%s\n' "cmake $*" >> "$INSTALL_TEST_LOG"
MOCK
cat > "$fixture/bin/ctest" <<'MOCK'
#!/usr/bin/env bash
printf '%s\n' ctest >> "$INSTALL_TEST_LOG"
exit "$INSTALL_TEST_FAIL"
MOCK
cp "$fixture/bin/cmake" "$fixture/bin/apt-get"
cp "$fixture/bin/cmake" "$fixture/bin/systemctl"
chmod +x "$fixture/bin/"*
export PATH="$fixture/bin:$PATH"
# The test may run as root in CI; only the child installer's UID check is replaced.
sed -i 's/(( EUID == 0 ))/(( 0 ))/' "$fixture/project/deploy/install.sh"

bash "$fixture/project/deploy/install.sh" > "$fixture/output.log"
grep -Fq 'sudo install -m 0644' "$INSTALL_TEST_LOG"
grep -Fq '/src/pso/config.json /usr/local/share/' "$INSTALL_TEST_LOG"
grep -Fq 'sudo systemctl enable mavlink-uart@pi.service' "$INSTALL_TEST_LOG"

export INSTALL_TEST_CONFIG=existing
: > "$INSTALL_TEST_LOG"
bash "$fixture/project/deploy/install.sh" > "$fixture/output.log"
if grep -Fq '/src/pso/config.json /usr/local/share/' "$INSTALL_TEST_LOG"; then
    echo 'Installer overwrote existing calibration config' >&2; exit 1
fi
grep -Fq 'Preserving existing config' "$fixture/output.log"

export INSTALL_TEST_FAIL=1
: > "$INSTALL_TEST_LOG"
if bash "$fixture/project/deploy/install.sh" > "$fixture/output.log"; then
    echo 'Installer ignored failing tests' >&2; exit 1
fi
if grep -Eq '^sudo (install|usermod|systemctl)' "$INSTALL_TEST_LOG"; then
    echo 'Installer deployed despite failing tests' >&2; exit 1
fi
