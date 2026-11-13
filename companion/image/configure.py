#!/usr/bin/env python3
"""Prepare a local Yocto build and source snapshot; never fetch or alter Git trees."""
import argparse
import subprocess
import tarfile
from pathlib import Path


def bitbake_path(value):
    path = Path(value).expanduser().resolve()
    if any(c.isspace() or c in '"$\\' for c in str(path)):
        raise ValueError(f"Yocto paths must not contain whitespace, quotes, dollars or backslashes: {path}")
    return path


def snapshot(repo, destination):
    paths = [repo / "companion/allocator/CMakeLists.txt"]
    for directory in ("companion/allocator/src", "companion/allocator/deploy"):
        paths.extend(p for p in (repo / directory).rglob("*") if p.is_file() and
                     p.suffix in (".cpp", ".hpp", ".json", ".service") and "build" not in p.relative_to(repo).parts and
                     "examples" not in p.relative_to(repo).parts)
    with tarfile.open(destination, "w:gz") as archive:
        for path in sorted(paths):
            archive.add(path, arcname="vtol-source/" + str(path.relative_to(repo)))


def prepare(poky, bsp, public_key=None):
    root = Path(__file__).resolve().parent
    repo = root.parent.parent
    poky, bsp = bitbake_path(poky), bitbake_path(bsp)
    if not (poky / "oe-init-build-env").is_file() or not (poky / "meta/conf/distro/include/yocto-uninative.inc").is_file():
        raise ValueError("Provide an existing Poky source directory")
    if not (bsp / "conf/machine/raspberrypi4-64.conf").is_file():
        raise ValueError("Provide an existing meta-raspberrypi source directory")
    if 'scarthgap' not in (bsp / "conf/layer.conf").read_text():
        raise ValueError("meta-raspberrypi must support scarthgap")
    if 'scarthgap' not in (poky / "meta/conf/layer.conf").read_text():
        raise ValueError("Poky must support scarthgap")
    build = bitbake_path(root / "build")
    if public_key is None:
        key = build / "ssh/id_ed25519"
        key.parent.mkdir(parents=True, exist_ok=True)
        if not key.exists():
            subprocess.run(["ssh-keygen", "-q", "-t", "ed25519", "-N", "", "-C", "vtol-companion", "-f", str(key)], check=True)
        public_key = Path(str(key) + ".pub")
    public_key = Path(public_key).expanduser().resolve()
    text = public_key.read_text().strip()
    if len(text.splitlines()) != 1 or not text.startswith(("ssh-ed25519 ", "ssh-rsa ", "ecdsa-sha2-", "sk-")):
        raise ValueError("Supply one OpenSSH public key, never a private key")
    subprocess.run(["ssh-keygen", "-lf", str(public_key)], check=True, stdout=subprocess.DEVNULL)
    provision = build / "provision"
    provision.mkdir(parents=True, exist_ok=True)
    (provision / "authorized_keys").write_text(text + "\n")
    snapshot(repo, provision / "vtol-source.tar.gz")
    conf = build / "yocto/conf"
    conf.mkdir(parents=True, exist_ok=True)
    replacements = {"@POKY@": poky, "@BSP@": bsp, "@LAYER@": root / "meta-vtol", "@PROVISION@": provision}
    for filename in ("local.conf", "bblayers.conf"):
        content = (root / (filename + ".in")).read_text()
        for token, path in replacements.items():
            content = content.replace(token, str(path))
        (conf / filename).write_text(content)
    print(f"Prepared {conf.parent}")
    print(f"Authorized public key: {public_key}")
    print(f"Build: source {poky}/oe-init-build-env {conf.parent} && bitbake vtol-companion-image")
    print("Re-run configure.py after changing project source or the authorized SSH key.")
    return conf.parent


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("poky", help="existing Poky scarthgap source directory")
    parser.add_argument("bsp", help="existing meta-raspberrypi scarthgap source directory")
    parser.add_argument("--ssh-public-key", type=Path, help="authorize this public key; otherwise create a dedicated key in ignored build/ssh")
    args = parser.parse_args()
    try:
        prepare(args.poky, args.bsp, args.ssh_public_key)
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        parser.exit(1, f"Configuration failed: {error}\n")
