"""Check laptop network setup using mocked commands; never change networking."""

import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "connect-companion.sh"


class NetworkTest(unittest.TestCase):
    def run_setup(
        self,
        *args,
        existing=False,
        device_type="ethernet",
        fail_modify=False,
        answer="",
        initial_config=None,
        ssh_status=0,
        missing=None,
        service_active=True,
        service_enabled=True,
        legacy_profile=False,
        existing_identity=False,
        key_installed=False,
        bootstrap_status=0,
    ):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            log = root / "calls.jsonl"
            script = root / "project with spaces/companion/connect-companion.sh"
            script.parent.mkdir(parents=True)
            shutil.copy2(SCRIPT, script)
            home = root / "home with spaces"
            home.mkdir()
            config = home / ".ssh/config"
            identity = home / ".ssh/vtol-companion_ed25519"
            if existing_identity or key_installed:
                identity.parent.mkdir(mode=0o700)
                subprocess.run(
                    [
                        "ssh-keygen",
                        "-q",
                        "-t",
                        "ed25519",
                        "-N",
                        "",
                        "-f",
                        str(identity),
                    ],
                    check=True,
                )
            previous_identity = identity.read_bytes() if identity.exists() else None
            if initial_config is not None:
                config.parent.mkdir(mode=0o700, exist_ok=True)
                config.write_text(initial_config)
            remote = root / "remote"
            remote.mkdir()
            for name in ("allocator", "config", "uart"):
                if name == missing:
                    continue
                path = remote / name
                if name == "uart":
                    path.symlink_to("/dev/null")
                else:
                    path.write_text("{}\n")
                    path.chmod(0o755 if name == "allocator" else 0o644)
            remote_home = remote / "home"
            remote_home.mkdir()
            server_config = remote / "sshd_config"
            server_config.write_text(
                "PubkeyAuthentication no\nPasswordAuthentication yes\n"
            )
            if key_installed:
                authorized = remote_home / ".ssh/authorized_keys"
                authorized.parent.mkdir(mode=0o700)
                authorized.write_text(identity.with_suffix(".pub").read_text())
                server_config.write_text(
                    "PubkeyAuthentication yes\nPasswordAuthentication yes\n"
                )
            connection_log = root / "ssh.jsonl"
            commands = {
                "sudo": """import os, sys
args = sys.argv[1:]
if args[0] == '-n':
    args = args[1:]
os.execvp(args[0], args)
""",
                "id": "print('vtol')\n",
                "journalctl": "print('Waiting for fresh PX4 state and pose targets')\n",
                "systemctl": """import os, sys
key = 'NETWORK_TEST_ACTIVE' if sys.argv[1] == 'is-active' else 'NETWORK_TEST_ENABLED'
sys.exit(0 if os.environ[key] == '1' else 1)
""",
                "sshd": """import os, sys
from pathlib import Path
assert sys.argv[1:] == ['-t']
assert 'PubkeyAuthentication yes' in Path(os.environ['NETWORK_TEST_SERVER_CONFIG']).read_text()
""",
                "ssh": """import json, os, subprocess, sys
from pathlib import Path
args = sys.argv[1:]
body = sys.stdin.read()
with open(os.environ['NETWORK_TEST_SSH_LOG'], 'a') as log:
    log.write(json.dumps(dict(args=args, script=body)) + '\\n')
status = int(os.environ['NETWORK_TEST_SSH_STATUS'])
if status:
    sys.exit(status)
root = Path(os.environ['NETWORK_TEST_REMOTE'])
remote_home = root / 'home'
if args[-1] == 'true' or 'BatchMode=yes' in args:
    identity = args[args.index('-i') + 1]
    public = subprocess.run(['ssh-keygen', '-y', '-f', identity], check=True, capture_output=True, text=True).stdout.strip()
    authorized = remote_home / '.ssh/authorized_keys'
    if not authorized.exists() or not any(line.startswith(public) for line in authorized.read_text().splitlines()):
        sys.exit(255)
    if 'PubkeyAuthentication yes' not in (root / 'sshd_config').read_text():
        sys.exit(255)
if args[-1] == 'true':
    sys.exit(0)
bootstrap = args[-1] != 'sh -se'
if bootstrap and int(os.environ['NETWORK_TEST_BOOTSTRAP_STATUS']):
    sys.exit(int(os.environ['NETWORK_TEST_BOOTSTRAP_STATUS']))
script = args[-1] if bootstrap else body
for original, name in [('/usr/bin/vtol_allocator', 'allocator'), ('/etc/vtol/pso.json', 'config'), ('/dev/serial0', 'uart'), ('/etc/ssh/sshd_config', 'sshd_config')]:
    script = script.replace(original, str(root / name))
script = script.replace('/usr/sbin/sshd', os.environ['NETWORK_TEST_SSHD'])
remote_env = dict(os.environ, HOME=str(remote_home))
command = ['sh', '-c', script] if bootstrap else ['sh', '-se']
sys.exit(subprocess.run(command, input=body if bootstrap else script,
                        text=True, env=remote_env).returncode)
""",
                "nmcli": """import json, os, sys
args = sys.argv[1:]
with open(os.environ['NETWORK_TEST_LOG'], 'a') as log:
    log.write(json.dumps(args) + '\\n')
if args[:3] == ['-g', 'GENERAL.TYPE', 'device']:
    print(os.environ['NETWORK_TEST_TYPE'])
elif args[:2] == ['connection', 'show']:
    key = 'NETWORK_TEST_EXISTING' if args[2] == 'vtol-companion' else 'NETWORK_TEST_LEGACY'
    sys.exit(0 if os.environ[key] == '1' else 10)
elif args[:2] == ['connection', 'modify'] and os.environ['NETWORK_TEST_FAIL'] == '1':
    sys.exit(5)
""",
            }
            for name, source in commands.items():
                command = root / name
                command.write_text(f"#!{sys.executable}\n{source}")
                command.chmod(0o755)
            env = dict(
                os.environ,
                PATH=f"{root}:{os.environ['PATH']}",
                HOME=str(home),
                NETWORK_TEST_LOG=str(log),
                NETWORK_TEST_TYPE=device_type,
                NETWORK_TEST_EXISTING=str(int(existing)),
                NETWORK_TEST_LEGACY=str(int(legacy_profile)),
                NETWORK_TEST_FAIL=str(int(fail_modify)),
                NETWORK_TEST_SSH_LOG=str(connection_log),
                NETWORK_TEST_REMOTE=str(remote),
                NETWORK_TEST_SSH_STATUS=str(ssh_status),
                NETWORK_TEST_ACTIVE=str(int(service_active)),
                NETWORK_TEST_ENABLED=str(int(service_enabled)),
                NETWORK_TEST_SERVER_CONFIG=str(server_config),
                NETWORK_TEST_SSHD=str(root / "sshd"),
                NETWORK_TEST_BOOTSTRAP_STATUS=str(bootstrap_status),
            )
            result = subprocess.run(
                [str(script), *args],
                env=env,
                input=answer,
                capture_output=True,
                text=True,
            )
            calls = (
                [json.loads(line) for line in log.read_text().splitlines()]
                if log.exists()
                else []
            )
            self.connections = (
                [json.loads(line) for line in connection_log.read_text().splitlines()]
                if connection_log.exists()
                else []
            )
            self.connection = self.connections[-1] if self.connections else None
            self.identity = dict(
                exists=identity.exists(),
                path=str(identity),
                mode=identity.stat().st_mode & 0o777 if identity.exists() else None,
                reused=previous_identity is not None
                and identity.read_bytes() == previous_identity,
            )
            self.server_config = server_config.read_text()
            authorized = remote_home / ".ssh/authorized_keys"
            self.authorized = authorized.read_text() if authorized.exists() else None
            self.authorized_mode = (
                authorized.stat().st_mode & 0o777 if authorized.exists() else None
            )
            self.remote_ssh_mode = (
                authorized.parent.stat().st_mode & 0o777
                if authorized.exists()
                else None
            )
            ssh = None
            if config.exists():
                parsed = subprocess.run(
                    ["ssh", "-G", "-F", str(config), "vtol-companion"],
                    capture_output=True,
                    text=True,
                    check=True,
                )
                ssh = dict(
                    text=config.read_text(),
                    mode=config.stat().st_mode & 0o777,
                    parsed=parsed.stdout,
                )
            return result, calls, ssh

    def test_create_and_reuse(self):
        for existing in (False, True):
            with self.subTest(existing=existing):
                result, calls, ssh = self.run_setup(existing=existing)
                self.assertEqual(result.returncode, 0, result.stderr)
                setup = calls[-2]
                self.assertEqual(
                    setup[:2], ["connection", "modify" if existing else "add"]
                )
                for key, value in [
                    ("connection.interface-name", "enp46s0"),
                    ("ipv4.addresses", "192.168.76.1/24"),
                    ("ipv4.never-default", "yes"),
                    ("ipv6.method", "disabled"),
                    ("ipv4.gateway", ""),
                    ("ipv4.dns", ""),
                ]:
                    self.assertEqual(setup[setup.index(key) + 1], value)
                self.assertEqual(
                    calls[-1],
                    ["connection", "up", "vtol-companion", "ifname", "enp46s0"],
                )
                self.assertIsNone(
                    ssh
                )  # EOF never authorizes changing SSH configuration.
                self.assertFalse(self.identity["exists"])

    def test_legacy_network_profile_is_renamed(self):
        result, calls, _ = self.run_setup(legacy_profile=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(
            calls[-2][:5],
            ["connection", "modify", "vtol-direct", "connection.id", "vtol-companion"],
        )
        self.assertEqual(
            calls[-1], ["connection", "up", "vtol-companion", "ifname", "enp46s0"]
        )
        self.assertFalse(any(call[:2] == ["connection", "add"] for call in calls))

    def test_current_ssh_alias_is_replaced_without_duplicates(self):
        original = "Host vtol-companion\n    HostName old.example\n"
        result, _, ssh = self.run_setup(answer="y\n", initial_config=original)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(ssh["text"].count("Host vtol-companion\n"), 1)
        self.assertIn("hostname 192.168.76.2\n", ssh["parsed"])

    def test_custom_interface(self):
        result, calls, _ = self.run_setup("enx1234")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(calls[-1][-1], "enx1234")

    def test_reject_non_ethernet(self):
        result, calls, _ = self.run_setup("wlo1", device_type="wifi")
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(len(calls), 1)

    def test_failed_configuration_does_not_activate(self):
        result, calls, _ = self.run_setup(existing=True, fail_modify=True)
        self.assertEqual(result.returncode, 5)
        self.assertFalse(any(call[:2] == ["connection", "up"] for call in calls))
        self.assertIsNone(self.connection)

    def test_append_ssh_profile_on_consent(self):
        original = "Host work\n    HostName work.example\n"
        result, _, ssh = self.run_setup(answer="yes\n", initial_config=original)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertTrue(ssh["text"].startswith(original))
        self.assertEqual(ssh["mode"], 0o600)
        for setting in (
            "hostname 192.168.76.2\n",
            "user vtol\n",
            "preferredauthentications publickey\n",
            "connecttimeout 10\n",
        ):
            self.assertIn(setting, ssh["parsed"])
        self.assertRegex(ssh["parsed"], r"(?m)^pubkeyauthentication (yes|true)$")
        self.assertIn("Connect: ssh vtol-companion", result.stdout)
        self.assertIn("Username: vtol; password: vtol", result.stdout)

    def test_create_ssh_profile(self):
        result, _, ssh = self.run_setup(answer="y\n")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("Host vtol-companion\n", ssh["text"])
        self.assertEqual(ssh["mode"], 0o600)

    def test_decline_keeps_config_unchanged(self):
        original = "Host work\n    HostName work.example\n"
        for initial in (None, original):
            result, _, ssh = self.run_setup(answer="n\n", initial_config=initial)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(ssh["text"] if ssh else None, initial)
            self.assertNotIn("Connect: ssh vtol-companion", result.stdout)

    def test_existing_alias_is_updated_without_changing_other_hosts(self):
        original = "Host other rasp\n    HostName old.example\n    User olduser\n"
        result, _, ssh = self.run_setup(answer="y\n", initial_config=original)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertTrue(
            ssh["text"].startswith(
                "Host other\n    HostName old.example\n    User olduser\n"
            )
        )
        self.assertIn("hostname 192.168.76.2\n", ssh["parsed"])
        self.assertIn("user vtol\n", ssh["parsed"])
        self.assertEqual(ssh["text"].count("Host vtol-companion\n"), 1)

    def test_previous_key_profile_is_replaced(self):
        original = "Host rasp\n    HostName old.example\n    IdentityFile /old/image/key\n    IdentitiesOnly yes\n"
        original += "Host work\n    HostName work.example\n"
        result, _, ssh = self.run_setup(answer="y\n", initial_config=original)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertNotIn("/old/image/key", ssh["text"])
        self.assertIn(f"identityfile {self.identity['path']}\n", ssh["parsed"])
        self.assertIn("Host work\n    HostName work.example\n", ssh["text"])
        self.assertIn("preferredauthentications publickey\n", ssh["parsed"])
        self.assertEqual(ssh["text"].count("Host vtol-companion\n"), 1)

    def test_remote_validation_without_saved_profile(self):
        result, _, _ = self.run_setup(answer="n\n")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.connection["args"][-2:], ["vtol@192.168.76.2", "sh -se"])
        for message in (
            "PASS: SSH login as vtol",
            "PASS: Allocator executable",
            "PASS: PSO configuration nonempty",
            "PASS: UART writable",
            "PASS: Allocator service enabled",
            "PASS: Allocator service running",
            "Waiting for fresh PX4 state and pose targets",
            "SSH and project validation passed.",
        ):
            self.assertIn(message, result.stdout)

    def test_remote_validation_uses_saved_profile(self):
        result, _, _ = self.run_setup(answer="y\n")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.connection["args"][-2:], ["vtol-companion", "sh -se"])

    def test_key_bootstrap_and_key_only_validation(self):
        result, _, ssh = self.run_setup(answer="y\n")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertTrue(self.identity["exists"])
        self.assertEqual(self.identity["mode"], 0o600)
        self.assertIn("PubkeyAuthentication yes", self.server_config)
        self.assertTrue(self.authorized.startswith("ssh-ed25519 "))
        self.assertEqual(self.authorized_mode, 0o600)
        self.assertEqual(self.remote_ssh_mode, 0o700)
        self.assertEqual(
            len(self.connections), 3
        )  # Probe, password bootstrap, key validation.
        self.assertIn("PreferredAuthentications=password", self.connections[1]["args"])
        self.assertIn("BatchMode=yes", self.connection["args"])
        self.assertIn("PasswordAuthentication=no", self.connection["args"])
        self.assertIn(f"identityfile {self.identity['path']}\n", ssh["parsed"])
        self.assertIn("SSH and project validation passed.", result.stdout)

    def test_existing_key_is_reused(self):
        result, _, _ = self.run_setup(answer="y\n", existing_identity=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertTrue(self.identity["reused"])

    def test_installed_key_skips_password_bootstrap(self):
        result, _, _ = self.run_setup(answer="y\n", key_installed=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertTrue(self.identity["reused"])
        self.assertEqual(len(self.connections), 2)
        self.assertEqual(len(self.authorized.splitlines()), 1)
        self.assertFalse(
            any(
                "PreferredAuthentications=password" in call["args"]
                for call in self.connections
            )
        )

    def test_bootstrap_failure_does_not_replace_config(self):
        original = "Host work\n    HostName work.example\n"
        result, _, ssh = self.run_setup(
            answer="y\n", initial_config=original, bootstrap_status=1
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(ssh["text"], original)
        self.assertNotIn("SSH and project validation passed.", result.stdout)

    def test_ssh_failure_propagates(self):
        result, _, _ = self.run_setup(ssh_status=255)
        self.assertEqual(result.returncode, 255)
        self.assertNotIn("SSH and project validation passed.", result.stdout)

    def test_missing_remote_files_fail_validation(self):
        for missing, message in [
            ("allocator", "Allocator executable"),
            ("config", "PSO configuration readable"),
            ("uart", "UART character device"),
        ]:
            with self.subTest(missing=missing):
                result, _, _ = self.run_setup(missing=missing)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("FAIL: " + message, result.stderr)
                self.assertNotIn("SSH and project validation passed.", result.stdout)

    def test_inactive_or_disabled_service_fails_validation(self):
        for options, message in [
            ({"service_active": False}, "Allocator service running"),
            ({"service_enabled": False}, "Allocator service enabled"),
        ]:
            with self.subTest(options=options):
                result, _, _ = self.run_setup(**options)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("FAIL: " + message, result.stderr)
                self.assertNotIn("SSH and project validation passed.", result.stdout)


if __name__ == "__main__":
    unittest.main()
