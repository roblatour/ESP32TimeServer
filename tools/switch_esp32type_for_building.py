## ESP32 Time Server v4.0.1
## Copyright Rob Latour, 2026
## License: MIT
## Website: https://github.com/roblatour/ESP32TimeServer


import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile

from build import PROFILES, PROTECTED_DEFINITIONS, PROTECTED_DIRECTORIES


ROOT = Path(__file__).resolve().parents[1]
PROFILE_LABELS = {
    "pre_v3": "ESP32-P4 before revision 3.0",
    "v3_0": "ESP32-P4 revision 3.0",
    "v3_1": "ESP32-P4 revision 3.1 or later",
    "s31": "ESP32-S31 Function CoreBoard 1",
}
JSONC_TOKENS = re.compile(r'"(?:[^"\\]|\\.)*"|//[^\r\n]*|/\*[\s\S]*?\*/')
JSONC_COMMAS = re.compile(r'"(?:[^"\\]|\\.)*"|,(?=\s*[}\]])')


def unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate settings key: {key}")
        result[key] = value
    return result


def strip_comments(text):
    return JSONC_TOKENS.sub(
        lambda match: re.sub(r"[^\r\n]", " ", match.group())
        if match.group().startswith("/") else match.group(), text)


def parse_settings(text):
    cleaned = JSONC_COMMAS.sub(
        lambda match: " " if match.group() == "," else match.group(), strip_comments(text))
    settings = json.loads(cleaned, object_pairs_hook=unique_object)
    if not isinstance(settings, dict):
        raise ValueError("VS Code settings must contain a JSON object")
    return settings, cleaned


def update_settings(text, updates):
    settings, cleaned = parse_settings(text)
    decoder = json.JSONDecoder()
    pending = dict(updates)
    replacements = []
    cursor = cleaned.index("{") + 1
    last_value_end = None
    while True:
        cursor += len(cleaned[cursor:]) - len(cleaned[cursor:].lstrip())
        if cleaned[cursor] == "}":
            break
        key, cursor = decoder.raw_decode(cleaned, cursor)
        cursor = cleaned.index(":", cursor) + 1
        cursor += len(cleaned[cursor:]) - len(cleaned[cursor:].lstrip())
        value_start = cursor
        _, cursor = decoder.raw_decode(cleaned, cursor)
        last_value_end = cursor
        if key in pending:
            value = json.dumps(pending.pop(key), ensure_ascii=False, indent=2)
            replacements.append((value_start, cursor, value.replace("\n", "\n  ")))
        cursor += len(cleaned[cursor:]) - len(cleaned[cursor:].lstrip())
        if cleaned[cursor] == ",":
            cursor += 1
    if pending:
        members = []
        for key, value in pending.items():
            formatted = json.dumps(value, ensure_ascii=False, indent=2).replace("\n", "\n  ")
            members.append(f"  {json.dumps(key)}: {formatted}")
        replacements.append((cursor, cursor, "\n" + ",\n".join(members) + "\n"))
        if settings and "," not in strip_comments(text[last_value_end:cursor]):
            replacements.append((last_value_end, last_value_end, ","))
    for start, end, replacement in sorted(replacements, key=lambda item: (item[0], item[1]), reverse=True):
        text = text[:start] + replacement + text[end:]
    parse_settings(text)
    return text


def string_list(settings, key):
    values = settings.get(key, [])
    if not isinstance(values, list) or not all(isinstance(value, str) for value in values):
        raise ValueError(f"{key} must be a list of strings")
    return values


def cmake_arguments(arguments):
    result = []
    cursor = 0
    while cursor < len(arguments):
        argument = arguments[cursor]
        definition = argument[2:] if argument.startswith("-D") else ""
        count = 1
        if argument == "-D" and cursor + 1 < len(arguments):
            definition = arguments[cursor + 1]
            count = 2
        name = definition.split("=", 1)[0].split(":", 1)[0]
        if "=" not in definition or name not in PROTECTED_DEFINITIONS:
            result.extend(arguments[cursor:cursor + count])
        cursor += count
    return result


def validate_build_directory(root, profile, directory):
    target, defaults, revision = PROFILES[profile]
    required_defaults = [root / "sdkconfig.defaults", root / defaults]
    if target == "esp32p4":
        required_defaults.append(root / "sdkconfig.defaults.esp32p4")
    for path in required_defaults:
        if not path.is_file():
            raise ValueError(f"required defaults file is missing: {path}")
    if not directory.is_relative_to(root) or directory == root:
        raise ValueError("the build directory must be inside the workspace, not its root")
    if directory.relative_to(root).parts[0].lower() in PROTECTED_DIRECTORIES:
        raise ValueError("the build directory cannot be inside a source or dependency directory")
    if directory.exists() and not directory.is_dir():
        raise ValueError("the build path is not a directory")
    config = directory / "sdkconfig"
    if config.is_symlink():
        raise ValueError("the build sdkconfig must not be a symlink")
    if config.exists():
        if not config.is_file():
            raise ValueError("the build sdkconfig is not a file")
        lines = set(config.read_text(encoding="utf-8").splitlines())
        flash_size = "16MB" if target == "esp32s31" else "32MB"
        required = {f'CONFIG_IDF_TARGET="{target}"', revision,
                    f'CONFIG_ESPTOOLPY_FLASHSIZE="{flash_size}"',
                    "CONFIG_COMPILER_OPTIMIZATION_PERF=y"}
        if not required.issubset(lines):
            raise ValueError("existing sdkconfig has a different target, revision, flash size, or optimization; "
                             "preserve it and use --build-dir with a new directory")
    cache = directory / "CMakeCache.txt"
    if cache.exists():
        if not config.is_file():
            raise ValueError("existing build cache has no isolated sdkconfig; use --build-dir with a new directory")
        entries = dict(re.findall(r"^(IDF_TARGET|SDKCONFIG):[^=]*=(.*)$",
                                  cache.read_text(encoding="utf-8"), re.MULTILINE))
        cached_config = entries.get("SDKCONFIG", "")
        if entries.get("IDF_TARGET") != target or not cached_config or (
                root / cached_config).resolve() != config:
            raise ValueError("existing build cache uses a different target or sdkconfig; "
                             "use --build-dir with a new directory")


def profile_from_chip_id(output):
    output = re.sub(r"\x1b\[[0-?]*[ -/]*[@-~]", "", output)
    descriptions = re.findall(r"^[ \t]*Chip(?:[ \t]+is[ \t]+|[ \t]+type:[ \t]*)([^\r\n]+)",
                              output, re.MULTILINE | re.IGNORECASE)
    if len(descriptions) != 1:
        raise ValueError("esptool did not report one unambiguous chip identity; settings were not changed")
    description = descriptions[0].strip()
    model = re.match(r"ESP32-[A-Z0-9]+\b", description, re.IGNORECASE)
    if model is None or model.group().upper() not in {"ESP32-P4", "ESP32-S31"}:
        raise ValueError(f"unsupported chip reported by esptool: {description}")
    revision = re.search(r"\(revision[ \t]+v?(\d+)\.(\d+)\)", description, re.IGNORECASE)
    if revision is None:
        raise ValueError(f"esptool did not report a usable chip revision: {description}")
    if model.group().upper() == "ESP32-S31":
        return "s31", description
    version = (int(revision.group(1)), int(revision.group(2)))
    if version < (3, 0):
        return "pre_v3", description
    if version == (3, 0):
        return "v3_0", description
    return "v3_1", description


def eim_config_paths():
    directories = [Path.home() / ".espressif", Path.home() / ".espressif" / "tools"]
    tools_path = os.environ.get("IDF_TOOLS_PATH")
    if tools_path:
        directories.insert(0, Path(tools_path).expanduser())
    if os.name == "nt":
        directories.append(Path(os.environ.get("SystemDrive", "C:") + "\\") / "Espressif" / "tools")
    return list(dict.fromkeys(directory / "eim_idf.json" for directory in directories))


def esptool_python():
    python_env = os.environ.get("IDF_PYTHON_ENV_PATH")
    if python_env:
        candidate = Path(python_env) / ("Scripts" if os.name == "nt" else "bin")
        candidate /= "python.exe" if os.name == "nt" else "python"
        if not candidate.is_file():
            raise ValueError("the ESP-IDF Python environment is unavailable; reopen an ESP-IDF 6.1 terminal")
        return str(candidate)
    root = ROOT.resolve()
    settings_path = root / ".vscode" / "settings.json"
    text = settings_path.read_text(encoding="utf-8-sig") if settings_path.exists() else "{}"
    settings, _ = parse_settings(text)
    setup = settings.get("idf.currentSetup") or os.environ.get("IDF_PATH")
    if not setup:
        return sys.executable
    if not isinstance(setup, str):
        raise ValueError("idf.currentSetup must be an ESP-IDF installation path")
    idf_path = (root / Path(os.path.expandvars(setup)).expanduser()).resolve()
    candidates = set()
    for config_path in eim_config_paths():
        if not config_path.is_file():
            continue
        try:
            config = json.loads(config_path.read_text(encoding="utf-8-sig"), object_pairs_hook=unique_object)
        except ValueError as error:
            raise ValueError(f"invalid ESP-IDF installer configuration {config_path}: {error}") from error
        installed = config.get("idfInstalled") if isinstance(config, dict) else None
        if not isinstance(installed, list):
            raise ValueError(f"invalid ESP-IDF installer configuration: {config_path}")
        for entry in installed:
            if not isinstance(entry, dict) or not isinstance(entry.get("path"), str) or not entry["path"]:
                continue
            if Path(entry["path"]).expanduser().resolve() != idf_path:
                continue
            python = entry.get("python")
            if not isinstance(python, str) or not python or not Path(python).is_absolute():
                raise ValueError(f"ESP-IDF installer has no absolute Python path for {idf_path}")
            candidates.add(Path(python).resolve())
    if len(candidates) > 1:
        raise ValueError("multiple Python environments are registered for the selected ESP-IDF installation; "
                         "open its ESP-IDF 6.1 terminal and retry")
    if not candidates:
        return sys.executable
    candidate = candidates.pop()
    if not candidate.is_file():
        raise ValueError(f"the registered ESP-IDF Python interpreter is unavailable: {candidate}; "
                         "repair the selected installation or open an ESP-IDF 6.1 terminal")
    return str(candidate)


def detect_profile(port):
    python = esptool_python()
    print(f"Using Python for esptool: {python}")
    print(f"Reading chip identity on {port}; esptool may reset the connected board.")
    try:
        result = subprocess.run([python, "-m", "esptool", "-p", port, "chip-id"],
                                cwd=ROOT.resolve(), stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                text=True, encoding="utf-8", errors="replace", timeout=45, check=False)
    except subprocess.TimeoutExpired as error:
        raise ValueError("esptool chip-id timed out; check the serial port, stop any active monitor, "
                         "and retry") from error
    if result.returncode != 0:
        raise ValueError(f"esptool chip-id failed on {port} (exit {result.returncode}):\n"
                         f"{result.stdout.strip()}\n"
                         "Use an ESP-IDF 6.1 terminal with esptool installed, check the port, "
                         "and stop any active monitor before retrying.")
    return profile_from_chip_id(result.stdout)


def switch_profile(profile, build_dir=None, port=None):
    root = ROOT.resolve()
    directory = (root / (build_dir or f"build_{profile}")).resolve()
    validate_build_directory(root, profile, directory)
    settings_path = root / ".vscode" / "settings.json"
    if settings_path.is_symlink():
        raise ValueError("VS Code settings must not be a symlink")
    text = settings_path.read_text(encoding="utf-8-sig") if settings_path.exists() else "{}\n"
    settings, _ = parse_settings(text)
    target, defaults, _ = PROFILES[profile]
    extra_vars = settings.get("idf.customExtraVars", {})
    if not isinstance(extra_vars, dict):
        raise ValueError("idf.customExtraVars must be an object")
    extra_vars = dict(extra_vars)
    extra_vars["IDF_TARGET"] = target
    extra_vars.pop("SDKCONFIG", None)
    extra_vars.pop("SDKCONFIG_DEFAULTS", None)
    updates = {
        "idf.customExtraVars": extra_vars,
        "idf.buildPath": str(directory),
        "idf.sdkconfigFilePath": str(directory / "sdkconfig"),
        "idf.sdkconfigDefaults": [str(root / "sdkconfig.defaults"), str(root / defaults)],
        "idf.openOcdConfigs": [f"board/{target}-builtin.cfg"],
    }
    if port is not None:
        updates["idf.port"] = port
    if "idf.cmakeCompilerArgs" in settings:
        updates["idf.cmakeCompilerArgs"] = cmake_arguments(string_list(settings, "idf.cmakeCompilerArgs"))
    if "clangd.arguments" in settings:
        arguments = string_list(settings, "clangd.arguments")
        filtered = []
        cursor = 0
        while cursor < len(arguments):
            argument = arguments[cursor]
            if argument == "--compile-commands-dir":
                cursor += 2
                continue
            if not argument.startswith("--compile-commands-dir="):
                filtered.append(argument)
            cursor += 1
        updates["clangd.arguments"] = filtered + [f"--compile-commands-dir={directory}"]
    updated = update_settings(text, updates)
    settings_path.parent.mkdir(parents=True, exist_ok=True)
    directory.mkdir(parents=True, exist_ok=True)
    temporary_path = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", newline="\n",
                                         dir=settings_path.parent, delete=False) as temporary:
            temporary_path = Path(temporary.name)
            temporary.write(updated)
        os.replace(temporary_path, settings_path)
    finally:
        if temporary_path is not None and temporary_path.exists():
            temporary_path.unlink()
    return directory


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Automatically detect the connected ESP32 chip and revision and select its VS Code ESP-IDF build profile. "
                    "Does not build, flash, or delete existing configurations.")
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument("--profile", choices=PROFILES, help="select explicitly without contacting a board")
    selection.add_argument("--manual", action="store_true", help="choose interactively without contacting a board")
    parser.add_argument("--port", "-p", help="serial port for automatic chip detection; defaults to VS Code idf.port")
    parser.add_argument("--build-dir", help="workspace-local build directory; defaults to build_<profile>")
    args = parser.parse_args(argv)
    profile = args.profile
    port = args.port
    try:
        if args.manual:
            choices = list(PROFILES)
            for number, key in enumerate(choices, 1):
                print(f"{number}. {PROFILE_LABELS[key]} ({key})")
            while profile is None:
                answer = input("Select profile [1-4], or q to cancel: ").strip()
                if answer.lower() == "q":
                    return 0
                if answer in choices:
                    profile = answer
                elif answer in {"1", "2", "3", "4"}:
                    profile = choices[int(answer) - 1]
                else:
                    print("Enter 1, 2, 3, 4, a profile name, or q.")
        elif profile is None:
            if port is None:
                settings_path = ROOT.resolve() / ".vscode" / "settings.json"
                text = settings_path.read_text(encoding="utf-8-sig") if settings_path.exists() else "{}"
                settings, _ = parse_settings(text)
                port = settings.get("idf.port")
                if port is None or port == "":
                    port = input("Serial port for chip detection (for example COM12): ").strip()
            if not isinstance(port, str) or not port.strip():
                raise ValueError("a serial port is required; select one in VS Code or supply --port")
            port = port.strip()
            profile, description = detect_profile(port)
            print(f"Detected {description}; selecting profile {profile}.")
        if port is not None:
            if not isinstance(port, str) or not port.strip():
                raise ValueError("--port must not be empty")
            port = port.strip()
        directory = switch_profile(profile, args.build_dir, port)
    except (EOFError, KeyboardInterrupt):
        print("\nProfile selection cancelled; settings were not changed.")
        return 130
    except (OSError, ValueError) as error:
        print(f"Unable to switch profile: {error}", file=sys.stderr)
        return 1
    print(f"Selected {PROFILE_LABELS[profile]} ({PROFILES[profile][0]}).")
    print(f"Build directory: {directory}")
    print(f"Configuration: {directory / 'sdkconfig'}")
    print("Now use ESP-IDF Build, Flash and Monitor in VS Code. No build or flash was performed.")
    print("Use this script again to switch profiles; do not run Set Target afterwards.")
    print("If VS Code still shows the old configuration, run Developer: Reload Window.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
