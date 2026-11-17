# Switch ESP32 build profiles

`.\tools\switch_esp32type_for_building.py` configures this repository for VS
Code's ESP-IDF **Build, Flash and Monitor** command. By default, it detects the
connected chip and revision using esptool and selects the matching build
profile. It does not build, flash, or delete existing configurations.

All repository paths and command examples below are relative to the repository
root. The script locates the repository from its own location, not the
terminal's current directory.

## Quick start

1. Connect the supported board over USB.
2. Stop any active serial monitor or other program using its serial port.
3. Open a Command Prompt at the repository root and run:

   ```cmd
   call C:\esp\v6.1\esp-idf\export.bat
   python .\tools\switch_esp32type_for_building.py
   ```

4. Check the reported chip, revision, profile, and build directory.
5. Use the ESP-IDF **Build, Flash and Monitor** command in VS Code.

If VS Code still displays the previous configuration, run **Developer: Reload
Window** from the Command Palette.

**Use this script to switch profiles. Do not run ESP-IDF Set Target afterward.**
Changing only the ESP-IDF chip selector does not select this project's
revision-specific P4 defaults. Set Target can also clean the selected build
configuration.

## Automatic detection

The script chooses the serial port in this order:

1. The `--port` or `-p` argument.
2. The `idf.port` setting in `.\.vscode\settings.json`.
3. A prompt, if no port is configured.

To specify a port explicitly:

```cmd
python .\tools\switch_esp32type_for_building.py --port COM12
```

The detection command is equivalent to:

```cmd
python -m esptool -p COM12 chip-id
```

When `IDF_PYTHON_ENV_PATH` is set, the script uses that ESP-IDF environment's
Python interpreter for esptool. Otherwise, it uses the interpreter running the
script. Automatic detection requires esptool with support for the connected
chip; the ESP-IDF 6.1 environment is recommended.

**Esptool may reset the connected board.** Detection times out after 45 seconds.
The script does not scan other ports or guess a profile if detection fails.
After successful detection and configuration, it saves the selected serial port
in `idf.port`.

### Profile selection

| Detected hardware             | Profile  | Default build directory |
| ----------------------------- | -------- | ----------------------- |
| P4 before revision 3.0        | `pre_v3` | `.\build_pre_v3`        |
| P4 revision 3.0               | `v3_0`   | `.\build_v3_0`          |
| P4 revision 3.1 or later      | `v3_1`   | `.\build_v3_1`          |
| S31, with a reported revision | `s31`    | `.\build_s31`           |

The chip family is checked before comparing revisions. An S31 revision is never
interpreted as a P4 revision. P4 major and minor revisions are compared as
integers, not floating-point numbers.

Detection identifies the chip and revision, not the board's wiring, peripherals,
or installed flash size. The supported P4 profiles expect 32 MB flash; the S31
Function CoreBoard 1 profile expects 16 MB. Hardware and network settings in
`.\main\ESP32TimeServerSettings.h` remain your responsibility and are not
changed.

## Manual and offline selection

To choose one of the four profiles interactively without contacting a board:

```cmd
python .\tools\switch_esp32type_for_building.py --manual
```

Enter a menu number, a profile name, or `q` to cancel without changing settings.

To select a profile directly without contacting a board:

```cmd
python .\tools\switch_esp32type_for_building.py --profile v3_1
```

Valid profile names are `pre_v3`, `v3_0`, `v3_1`, and `s31`. The `--manual` and
`--profile` options cannot be combined. Offline selection needs Python 3.9 or
later but does not need esptool or an activated ESP-IDF environment.

In manual or explicit-profile mode, the existing serial port is preserved unless
`--port` is supplied. A supplied port is saved without probing the board.

## Build directories and existing configurations

Each profile uses a separate build directory with its generated configuration
inside it. For example, S31 uses `.\build_s31\sdkconfig`, which ESP-IDF
generates when first configuring the build. The switcher creates the build
directory but does not generate the configuration itself or migrate the root
`.\sdkconfig`.

To use a different directory:

```cmd
python .\tools\switch_esp32type_for_building.py ^
  --port COM12 --build-dir .\build_board
```

`--build-dir` also works with `--manual` and `--profile`. Relative
build-directory arguments are resolved from the repository root. The directory
must remain inside the repository and cannot be its root or a protected source
directory.

The script rejects an existing configuration whose target, revision, flash size,
or performance optimization does not match the selected profile. It also rejects
an existing CMake cache that uses a different target or configuration path, or
has no isolated configuration. It does not overwrite or delete those files.
Preserve them and select a fresh directory with `--build-dir` instead.

**Back up important menuconfig customizations before using Full Clean or Set
Target.** Generated configurations are inside build directories and can be
removed by cleaning operations.

P4 and S31 builds must run sequentially in one checkout because the Component
Manager shares `.\managed_components` between them.

## Settings changed

The script updates `.\.vscode\settings.json` with:

- **Target**: `IDF_TARGET` in `idf.customExtraVars`.
- **Build location**: `idf.buildPath` and `idf.sdkconfigFilePath`.
- **Defaults**: `idf.sdkconfigDefaults`, using `.\sdkconfig.defaults` and the
  selected board's defaults under `.\config`.
- **USB-JTAG configuration**: `idf.openOcdConfigs` for the selected chip family.
- **Serial port**: `idf.port`, when a port was selected or explicitly supplied.
- **clangd**: The compilation-database directory, if `clangd.arguments` exists.

Conflicting `SDKCONFIG` and `SDKCONFIG_DEFAULTS` environment overrides are
removed from `idf.customExtraVars`. If `idf.cmakeCompilerArgs` exists, duplicate
target and configuration definitions are removed while unrelated arguments are
kept.

Unrelated settings, including the ESP-IDF installation and flash method, are
preserved. VS Code JSON with comments and trailing commas is accepted; invalid
settings or duplicate keys are rejected. Settings are replaced atomically using
a temporary file in the same directory.

The generated paths are absolute. Run the switcher again after moving the
repository to another location.

## Troubleshooting

- **Port unavailable or detection times out**: Check the USB connection and
  port, stop the serial monitor, and retry with an explicit `--port`.
- **Esptool missing or S31 unsupported**: Load the ESP-IDF 6.1 environment and
  run the script again. An invalid `IDF_PYTHON_ENV_PATH` requires reopening or
  reloading the ESP-IDF environment.
- **Unsupported chip or missing revision**: No profile is guessed and settings
  are not changed. Check the connected board and esptool version. Use manual
  selection only if you know the correct profile.
- **Existing configuration rejected**: Use `--build-dir` with a fresh directory
  rather than deleting customized configurations.
- **Wrong hardware profile after manual selection**: Rerun the script with
  automatic detection before building.

## Help and exit codes

```cmd
python .\tools\switch_esp32type_for_building.py --help
```

- **0**: Configuration succeeded, or the manual menu was cancelled with `q`.
- **1**: Detection, configuration validation, or a filesystem operation failed.
- **2**: Invalid command-line arguments.
- **130**: Selection was interrupted with Ctrl+C or end-of-input.
