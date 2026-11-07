"""Build a target profile in this workspace without flashing or deleting files."""

import argparse
import os
from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
PROFILES = {
    "pre_v3": ("esp32p4", Path("config") / "esp32p4_rev_pre_v3.defaults", "CONFIG_ESP32P4_REV_MIN_1=y"),
    "v3_0": ("esp32p4", Path("config") / "esp32p4_rev_v3_0.defaults", "CONFIG_ESP32P4_REV_MIN_300=y"),
    "v3_1": ("esp32p4", Path("config") / "esp32p4_rev_v3_1.defaults", "CONFIG_ESP32P4_REV_MIN_301=y"),
    "s31": ("esp32s31", Path("config") / "esp32s31_function_coreboard_1.defaults", "CONFIG_ESP32S31_REV_MIN_0=y"),
}
PREVIEW_TARGETS = {"esp32s31"}
PROTECTED_DIRECTORIES = {
    ".devcontainer", ".git", ".github", ".vscode", ".zencoder", ".zenflow",
    "3d printable case", "3rdparty", "components", "config", "homeassistant",
    "main", "managed_components", "misc", "pcb", "release", "testing", "tools",
    "v3upgrade",
}
PROTECTED_DEFINITIONS = {"IDF_TARGET", "SDKCONFIG", "SDKCONFIG_DEFAULTS"}
DEFINITION_PATTERN = re.compile(r"^([A-Za-z_][A-Za-z0-9_]*)(?::[A-Za-z_][A-Za-z0-9_]*)?=.*$")


def build_command(profile, build_dir, idf_path, python, definitions, reconfigure):
    target, profile_defaults, _ = PROFILES[profile]
    defaults = f"{ROOT / 'sdkconfig.defaults'};{ROOT / profile_defaults}"
    command = [str(python), str(idf_path / "tools" / "idf.py")]
    if target in PREVIEW_TARGETS:
        command.append("--preview")
    command.extend([
        "-C", str(ROOT),
        "-B", str(build_dir),
        f"-DIDF_TARGET={target}",
        f"-DSDKCONFIG={build_dir / 'sdkconfig'}",
        f"-DSDKCONFIG_DEFAULTS={defaults}",
    ])
    command.extend(f"-D{definition}" for definition in definitions)
    if reconfigure:
        command.append("reconfigure")
    command.append("build")
    return command


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", choices=PROFILES, default="v3_1")
    parser.add_argument("--build-dir", help="workspace-local output directory")
    parser.add_argument("--define", action="append", default=[], metavar="NAME=VALUE")
    parser.add_argument("--reconfigure", action="store_true", help="reconfigure before building")
    args = parser.parse_args(argv)

    root = ROOT.resolve()
    build_dir = (root / (args.build_dir or f"build_{args.profile}")).resolve()
    if not build_dir.is_relative_to(root) or build_dir == root:
        parser.error("the build directory must be inside the workspace and must not be its root")
    if build_dir.relative_to(root).parts[0].lower() in PROTECTED_DIRECTORIES:
        parser.error("the build directory cannot be inside a source or dependency directory")
    if build_dir.exists() and not build_dir.is_dir():
        parser.error("the build path exists and is not a directory")

    for definition in args.define:
        match = DEFINITION_PATTERN.fullmatch(definition)
        if match is None or match.group(1).upper() in PROTECTED_DEFINITIONS:
            parser.error("definitions must be NAME=VALUE and cannot override target or sdkconfig paths")

    target, profile_defaults, expected_revision = PROFILES[args.profile]
    defaults_files = [root / "sdkconfig.defaults", root / profile_defaults]
    if target == "esp32p4":
        defaults_files.append(root / "sdkconfig.defaults.esp32p4")
    for path in defaults_files:
        if not path.is_file():
            parser.error(f"required defaults file is missing: {path}")

    config = build_dir / "sdkconfig"
    if config.is_symlink():
        parser.error("the build sdkconfig must not be a symlink")
    if config.exists():
        if not config.is_file():
            parser.error(f"the build sdkconfig is not a file: {config}")
        lines = set(config.read_text(encoding="utf-8").splitlines())
        if f'CONFIG_IDF_TARGET="{target}"' not in lines or expected_revision not in lines:
            parser.error("the existing sdkconfig does not match this profile; use another build directory")
    elif (build_dir / "CMakeCache.txt").exists():
        parser.error("the existing build cache has no isolated sdkconfig; use another build directory")

    idf_value = os.environ.get("IDF_PATH")
    python_env = os.environ.get("IDF_PYTHON_ENV_PATH")
    if not idf_value or not python_env:
        parser.error("load the selected ESP-IDF environment before running this helper")
    idf_path = Path(idf_value).resolve()
    python = Path(python_env) / ("Scripts" if os.name == "nt" else "bin") / ("python.exe" if os.name == "nt" else "python")
    if not (idf_path / "tools" / "idf.py").is_file() or not python.is_file():
        parser.error("the selected ESP-IDF installation or Python environment is incomplete")

    command = build_command(args.profile, build_dir, idf_path, python, args.define, args.reconfigure)
    try:
        return subprocess.run(command, cwd=root, check=False).returncode
    except OSError as error:
        print(f"Unable to run the ESP-IDF build: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
