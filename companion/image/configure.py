#!/usr/bin/env python3
"""Prepare a local Yocto build and source snapshot; never fetch or alter Git trees."""

import argparse
import io
import json
import runpy
import subprocess
import tarfile
from pathlib import Path


def bitbake_path(value):
    path = Path(value).expanduser().resolve()
    if any(c.isspace() or c in '"$\\' for c in str(path)):
        raise ValueError(
            f"Yocto paths must not contain whitespace, quotes, dollars or backslashes: {path}"
        )
    return path


def snapshot(repo, destination):
    vehicle_bytes = subprocess.check_output(
        [
            str(Path.home() / ".local/share/vtol26/config/venv/bin/python3"),
            "-I",
            str(Path.home() / ".local/share/vtol26/config/validate_vehicle.py"),
            "--print-json",
        ]
    )
    tuning_path = repo / "companion/allocator/src/pso/config.json"
    hil_config = runpy.run_path(str(repo / "companion/allocator/prepare_hil.py"))[
        "configuration"
    ]
    tuning = hil_config(
        json.loads(vehicle_bytes),
        json.loads(tuning_path.read_text()),
    )
    tuning_bytes = (json.dumps(tuning, indent=2, allow_nan=False) + "\n").encode()
    paths = [repo / "companion/allocator/CMakeLists.txt"]
    for directory in ("companion/allocator/src", "companion/allocator/deploy"):
        paths.extend(
            p
            for p in (repo / directory).rglob("*")
            if p.is_file()
            and p.suffix in (".cpp", ".hpp", ".json", ".service")
            and "build" not in p.relative_to(repo).parts
            and "examples" not in p.relative_to(repo).parts
        )
    with tarfile.open(destination, "w:gz") as archive:
        vehicle_info = tarfile.TarInfo("vtol-source/config/vehicle.json")
        vehicle_info.size = len(vehicle_bytes)
        vehicle_info.mode = 0o644
        archive.addfile(vehicle_info, io.BytesIO(vehicle_bytes))
        for name in ("validate_vehicle.py", "vehicle.schema.json", "requirements.txt"):
            archive.add(
                (Path.home() / ".local/share/vtol26/config") / name,
                arcname="vtol-source/config/" + name,
            )
        for path in sorted(paths):
            name = "vtol-source/" + str(path.relative_to(repo))
            if path == tuning_path:
                info = archive.gettarinfo(str(path), arcname=name)
                info.size = len(tuning_bytes)
                archive.addfile(info, io.BytesIO(tuning_bytes))
            else:
                archive.add(path, arcname=name)


def prepare(poky, bsp):
    root = Path(__file__).resolve().parent
    repo = root.parent.parent
    poky, bsp = bitbake_path(poky), bitbake_path(bsp)
    if (
        not (poky / "oe-init-build-env").is_file()
        or not (poky / "meta/conf/distro/include/yocto-uninative.inc").is_file()
    ):
        raise ValueError("Provide an existing Poky source directory")
    if not (bsp / "conf/machine/raspberrypi4-64.conf").is_file():
        raise ValueError("Provide an existing meta-raspberrypi source directory")
    if "scarthgap" not in (bsp / "conf/layer.conf").read_text():
        raise ValueError("meta-raspberrypi must support scarthgap")
    if "scarthgap" not in (poky / "meta/conf/layer.conf").read_text():
        raise ValueError("Poky must support scarthgap")
    build = bitbake_path(root / "build")
    provision = build / "provision"
    provision.mkdir(parents=True, exist_ok=True)
    snapshot(repo, provision / "vtol-source.tar.gz")
    conf = build / "yocto/conf"
    conf.mkdir(parents=True, exist_ok=True)
    replacements = {
        "@POKY@": poky,
        "@BSP@": bsp,
        "@LAYER@": root / "meta-vtol",
        "@PROVISION@": provision,
    }
    for filename in ("local.conf", "bblayers.conf"):
        content = (root / (filename + ".in")).read_text()
        for token, path in replacements.items():
            content = content.replace(token, str(path))
        (conf / filename).write_text(content)
    print(f"Prepared {conf.parent}")
    print("SSH login: vtol (password: vtol)")
    print(
        f"Build: source {poky}/oe-init-build-env {conf.parent} && bitbake vtol-companion-image"
    )
    print("Re-run configure.py after changing project source.")
    return conf.parent


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("poky", help="existing Poky scarthgap source directory")
    parser.add_argument(
        "bsp", help="existing meta-raspberrypi scarthgap source directory"
    )
    args = parser.parse_args()
    try:
        prepare(args.poky, args.bsp)
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        parser.exit(1, f"Configuration failed: {error}\n")
