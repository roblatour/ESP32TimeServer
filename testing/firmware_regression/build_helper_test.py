import contextlib
import importlib.util
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest import mock


PROJECT_ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("time_server_build", PROJECT_ROOT / "tools" / "build.py")
if SPEC is None or SPEC.loader is None:
    raise RuntimeError("Unable to load the build helper")
build = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(build)
SWITCH_SPEC = importlib.util.spec_from_file_location(
    "time_server_switch", PROJECT_ROOT / "tools" / "switch_esp32type_for_building.py")
if SWITCH_SPEC is None or SWITCH_SPEC.loader is None:
    raise RuntimeError("Unable to load the profile switcher")
switcher = importlib.util.module_from_spec(SWITCH_SPEC)
with mock.patch.object(sys, "path", [str(PROJECT_ROOT / "tools"), *sys.path]):
    SWITCH_SPEC.loader.exec_module(switcher)


class BuildHelperTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="time-server-build-test-")
        self.addCleanup(self.temporary.cleanup)
        parent = Path(self.temporary.name)
        self.root = (parent / "workspace").resolve()
        self.root.mkdir()
        (self.root / "config").mkdir()
        (self.root / "sdkconfig.defaults").write_text("", encoding="utf-8")
        (self.root / "sdkconfig.defaults.esp32p4").write_text("", encoding="utf-8")
        for _, defaults, _ in build.PROFILES.values():
            (self.root / defaults).write_text("", encoding="utf-8")
        self.idf = parent / "idf"
        (self.idf / "tools").mkdir(parents=True)
        (self.idf / "tools" / "idf.py").touch()
        self.python_env = parent / "python-env"
        python_dir = self.python_env / ("Scripts" if os.name == "nt" else "bin")
        python_dir.mkdir(parents=True)
        self.python = python_dir / ("python.exe" if os.name == "nt" else "python")
        self.python.touch()
        root_patch = mock.patch.object(build, "ROOT", self.root)
        root_patch.start()
        self.addCleanup(root_patch.stop)
        env_patch = mock.patch.dict(os.environ, {
            "IDF_PATH": str(self.idf),
            "IDF_PYTHON_ENV_PATH": str(self.python_env),
        })
        env_patch.start()
        self.addCleanup(env_patch.stop)

    def assert_rejected(self, arguments, message):
        stderr = io.StringIO()
        with contextlib.redirect_stderr(stderr), self.assertRaises(SystemExit) as raised:
            build.main(arguments)
        self.assertEqual(raised.exception.code, 2)
        self.assertIn(message, stderr.getvalue())

    def test_profile_commands_and_preview(self):
        for profile, (target, defaults, _) in build.PROFILES.items():
            with self.subTest(profile=profile):
                directory = self.root / f"build_{profile}"
                command = build.build_command(profile, directory, self.idf, self.python, [], False)
                self.assertIn(f"-DIDF_TARGET={target}", command)
                self.assertIn(f"-DSDKCONFIG={directory / 'sdkconfig'}", command)
                self.assertIn(f"-DSDKCONFIG_DEFAULTS={self.root / 'sdkconfig.defaults'};{self.root / defaults}", command)
                self.assertEqual("--preview" in command, profile == "s31")
                self.assertEqual(command[-1], "build")
                self.assertNotIn("set-target", command)

    def test_fresh_and_incremental_builds(self):
        with mock.patch.object(build.subprocess, "run",
                               return_value=subprocess.CompletedProcess([], 0)) as run:
            self.assertEqual(build.main(["--profile", "v3_0", "--define", "FOO:BOOL=ON"]), 0)
            command = run.call_args.args[0]
            self.assertIn("-DFOO:BOOL=ON", command)
            self.assertEqual(run.call_args.kwargs["cwd"], self.root)
            self.assertEqual(command[-1], "build")
            directory = self.root / "custom_build"
            directory.mkdir()
            config = directory / "sdkconfig"
            contents = 'CONFIG_IDF_TARGET="esp32p4"\nCONFIG_ESP32P4_REV_MIN_300=y\n'
            config.write_text(contents, encoding="utf-8")
            self.assertEqual(build.main(["--profile", "v3_0", "--build-dir", str(directory),
                                         "--reconfigure"]), 0)
            command = run.call_args.args[0]
            self.assertEqual(command[-2:], ["reconfigure", "build"])
            self.assertNotIn("set-target", command)
            self.assertEqual(config.read_text(encoding="utf-8"), contents)

    def test_s31_fresh_and_incremental_builds(self):
        with mock.patch.object(build.subprocess, "run",
                               return_value=subprocess.CompletedProcess([], 0)) as run:
            self.assertEqual(build.main(["--profile", "s31"]), 0)
            self.assertIn("--preview", run.call_args.args[0])
            directory = self.root / "build_s31"
            directory.mkdir()
            config = directory / "sdkconfig"
            contents = 'CONFIG_IDF_TARGET="esp32s31"\nCONFIG_ESP32S31_REV_MIN_0=y\n'
            config.write_text(contents, encoding="utf-8")
            self.assertEqual(build.main(["--profile", "s31", "--reconfigure"]), 0)
            self.assertEqual(run.call_args.args[0][-2:], ["reconfigure", "build"])
            self.assertEqual(config.read_text(encoding="utf-8"), contents)

    def test_invalid_paths_and_definitions_are_rejected(self):
        for directory in (str(self.root.parent / "outside"), str(self.root), "main",
                          str(self.root / "config" / "build")):
            with self.subTest(directory=directory):
                self.assert_rejected(["--build-dir", directory], "build directory")
        for definition in ("IDF_TARGET=esp32s31", "SDKCONFIG=outside", "SDKCONFIG_DEFAULTS=other",
                           "invalid name=1", "-DVALUE=1"):
            with self.subTest(definition=definition):
                self.assert_rejected([f"--define={definition}"], "definitions must")

    def test_existing_config_and_cache_are_preserved(self):
        directory = self.root / "build_existing"
        directory.mkdir()
        config = directory / "sdkconfig"
        for contents in ('CONFIG_IDF_TARGET="esp32s31"\nCONFIG_ESP32P4_REV_MIN_301=y\n',
                         'CONFIG_IDF_TARGET="esp32p4"\nCONFIG_ESP32P4_REV_MIN_300=y\n'):
            with self.subTest(contents=contents):
                config.write_text(contents, encoding="utf-8")
                self.assert_rejected(["--build-dir", str(directory), "--reconfigure"], "does not match")
                self.assertEqual(config.read_text(encoding="utf-8"), contents)
        config.unlink()
        (directory / "CMakeCache.txt").touch()
        self.assert_rejected(["--build-dir", str(directory)], "has no isolated sdkconfig")
        with mock.patch.object(Path, "is_symlink", return_value=True):
            self.assert_rejected(["--build-dir", str(directory)], "must not be a symlink")

    def test_missing_environment_and_subprocess_failures(self):
        with mock.patch.dict(os.environ, {}, clear=True):
            self.assert_rejected([], "load the selected ESP-IDF environment")
        with mock.patch.object(build.subprocess, "run",
                               return_value=subprocess.CompletedProcess([], 23)):
            self.assertEqual(build.main([]), 23)
        stderr = io.StringIO()
        with mock.patch.object(build.subprocess, "run", side_effect=OSError("cannot start")), \
                contextlib.redirect_stderr(stderr):
            self.assertEqual(build.main([]), 1)
        self.assertIn("cannot start", stderr.getvalue())


class ProfileSwitcherTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="time-server-switch-test-")
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name).resolve() / "workspace with spaces"
        self.root.mkdir()
        (self.root / "config").mkdir()
        (self.root / ".vscode").mkdir()
        (self.root / "sdkconfig.defaults").write_text("", encoding="utf-8")
        (self.root / "sdkconfig.defaults.esp32p4").write_text("", encoding="utf-8")
        for _, defaults, _ in build.PROFILES.values():
            (self.root / defaults).write_text("", encoding="utf-8")
        self.settings = self.root / ".vscode" / "settings.json"
        self.original = {
            "idf.currentSetup": "C:/esp/v6.1/esp-idf",
            "idf.port": "COM12",
            "idf.flashType": "JTAG",
            "idf.customExtraVars": {"IDF_TARGET": "esp32s31", "USER_VARIABLE": "keep"},
            "clangd.arguments": ["--background-index", "--compile-commands-dir=old"],
            "cSpell.words": ["blox"],
        }
        self.settings.write_text(json.dumps(self.original, indent=2) + "\n", encoding="utf-8")
        root_patch = mock.patch.object(switcher, "ROOT", self.root)
        root_patch.start()
        self.addCleanup(root_patch.stop)
        self.eim_config = self.root.parent / "eim_idf.json"
        self.discover_eim_paths = switcher.eim_config_paths
        eim_patch = mock.patch.object(switcher, "eim_config_paths", return_value=[self.eim_config])
        eim_patch.start()
        self.addCleanup(eim_patch.stop)

    def write_config(self, profile, directory=None):
        directory = directory or self.root / f"build_{profile}"
        directory.mkdir(exist_ok=True)
        target, _, revision = build.PROFILES[profile]
        flash = "16MB" if profile == "s31" else "32MB"
        contents = (f'CONFIG_IDF_TARGET="{target}"\n{revision}\n'
                    f'CONFIG_ESPTOOLPY_FLASHSIZE="{flash}"\n'
                    "CONFIG_COMPILER_OPTIMIZATION_PERF=y\nCONFIG_USER_SETTING=42\n")
        (directory / "sdkconfig").write_text(contents, encoding="utf-8")
        return contents

    def test_all_profiles_update_matching_settings_without_building(self):
        root_config = self.root / "sdkconfig"
        root_config.write_text("existing root configuration", encoding="utf-8")
        for profile, (target, defaults, _) in build.PROFILES.items():
            with self.subTest(profile=profile), mock.patch.dict(os.environ, {}, clear=True), \
                    mock.patch.object(subprocess, "run") as run:
                directory = switcher.switch_profile(profile)
                run.assert_not_called()
                settings = json.loads(self.settings.read_text(encoding="utf-8"))
                self.assertEqual(directory, self.root / f"build_{profile}")
                self.assertEqual(settings["idf.customExtraVars"],
                                 {"IDF_TARGET": target, "USER_VARIABLE": "keep"})
                self.assertEqual(settings["idf.buildPath"], str(directory))
                self.assertEqual(settings["idf.sdkconfigFilePath"], str(directory / "sdkconfig"))
                self.assertEqual(settings["idf.sdkconfigDefaults"],
                                 [str(self.root / "sdkconfig.defaults"), str(self.root / defaults)])
                self.assertEqual(settings["idf.openOcdConfigs"], [f"board/{target}-builtin.cfg"])
                self.assertEqual(settings["clangd.arguments"],
                                 ["--background-index", f"--compile-commands-dir={directory}"])
                for key in ("idf.port", "idf.flashType", "idf.currentSetup", "cSpell.words"):
                    self.assertEqual(settings[key], self.original[key])
                self.assertFalse((directory / "sdkconfig").exists())
        self.assertEqual(root_config.read_text(encoding="utf-8"), "existing root configuration")

    def test_existing_matching_config_and_cache_are_preserved(self):
        for profile, (target, _, _) in build.PROFILES.items():
            with self.subTest(profile=profile):
                directory = self.root / f"build_{profile}"
                contents = self.write_config(profile)
                cache = (f"IDF_TARGET:STRING={target}\n"
                         f"SDKCONFIG:FILEPATH={(directory / 'sdkconfig').as_posix()}\n")
                (directory / "CMakeCache.txt").write_text(cache, encoding="utf-8")
                switcher.switch_profile(profile)
                switcher.switch_profile(profile)
                self.assertEqual((directory / "sdkconfig").read_text(encoding="utf-8"), contents)
                self.assertEqual((directory / "CMakeCache.txt").read_text(encoding="utf-8"), cache)

    def test_jsonc_comments_trailing_commas_and_strings_are_preserved(self):
        for text in ('{}\n', '{"keep": 1}\n', '{"keep": 1,}\n',
                     '{\n// preserved comment, with a comma\n"keep": {"array": [1, 2,],},\n}\n',
                     '{"keep": "https://example.invalid/a/*literal*/,}\\\"", "idf.buildPath": "old"}\n'):
            with self.subTest(text=text):
                settings, _ = switcher.parse_settings(text)
                updated = switcher.update_settings(text, {"idf.buildPath": "new", "added": {"x": [1, 2]}})
                actual, _ = switcher.parse_settings(updated)
                self.assertEqual(actual, {**settings, "idf.buildPath": "new", "added": {"x": [1, 2]}})
                if "// preserved comment" in text:
                    self.assertIn("// preserved comment, with a comma", updated)
                self.assertEqual(switcher.update_settings(updated, actual),
                                 switcher.update_settings(switcher.update_settings(updated, actual), actual))

    def test_missing_settings_and_stale_overrides(self):
        self.settings.unlink()
        switcher.switch_profile("s31")
        self.assertEqual(json.loads(self.settings.read_text(encoding="utf-8"))[
            "idf.customExtraVars"]["IDF_TARGET"], "esp32s31")
        settings = dict(self.original)
        settings["idf.customExtraVars"] = {"SDKCONFIG": "old", "SDKCONFIG_DEFAULTS": "old", "KEEP": "value"}
        settings["idf.cmakeCompilerArgs"] = ["-G", "Ninja", "-DESP_PLATFORM=1", "-DIDF_TARGET=esp32s31",
                                             "-D", "SDKCONFIG:FILEPATH=old", "-DSDKCONFIG_DEFAULTS=old", "-DUSER=1"]
        settings["clangd.arguments"] = ["--query-driver=**", "--compile-commands-dir", "old"]
        self.settings.write_text(json.dumps(settings), encoding="utf-8")
        directory = switcher.switch_profile("v3_0")
        actual = json.loads(self.settings.read_text(encoding="utf-8"))
        self.assertEqual(actual["idf.customExtraVars"], {"IDF_TARGET": "esp32p4", "KEEP": "value"})
        self.assertEqual(actual["idf.cmakeCompilerArgs"], ["-G", "Ninja", "-DESP_PLATFORM=1", "-DUSER=1"])
        self.assertEqual(actual["clangd.arguments"], ["--query-driver=**", f"--compile-commands-dir={directory}"])

    def test_rejects_incompatible_configs_and_caches_without_changing_settings(self):
        directory = self.root / "build_s31"
        contents = self.write_config("s31")
        original_settings = self.settings.read_bytes()
        for invalid in (contents.replace("16MB", "2MB"), contents.replace("esp32s31", "esp32p4"),
                        contents.replace("CONFIG_ESP32S31_REV_MIN_0=y", "CONFIG_ESP32P4_REV_MIN_301=y"),
                        contents.replace("CONFIG_COMPILER_OPTIMIZATION_PERF=y", "")):
            (directory / "sdkconfig").write_text(invalid, encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "existing sdkconfig"):
                switcher.switch_profile("s31")
            self.assertEqual((directory / "sdkconfig").read_text(encoding="utf-8"), invalid)
            self.assertEqual(self.settings.read_bytes(), original_settings)
        self.write_config("s31")
        cache = directory / "CMakeCache.txt"
        for invalid in ("IDF_TARGET:STRING=esp32p4\n", "IDF_TARGET:STRING=esp32s31\n",
                        f"IDF_TARGET:STRING=esp32s31\nSDKCONFIG:FILEPATH={self.root / 'sdkconfig'}\n"):
            cache.write_text(invalid, encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "existing build cache"):
                switcher.switch_profile("s31")
            self.assertEqual(self.settings.read_bytes(), original_settings)
        (directory / "sdkconfig").unlink()
        with self.assertRaisesRegex(ValueError, "no isolated sdkconfig"):
            switcher.switch_profile("s31")
        switcher.switch_profile("s31", "build_s31_fresh")
        self.assertEqual(cache.read_text(encoding="utf-8"), invalid)

    def test_invalid_paths_and_settings_are_not_modified(self):
        original = self.settings.read_bytes()
        for path in (str(self.root), str(self.root.parent / "outside"), "main", "config/build", ".vscode"):
            with self.subTest(path=path), self.assertRaisesRegex(ValueError, "build directory"):
                switcher.switch_profile("s31", path)
            self.assertEqual(self.settings.read_bytes(), original)
        for text in ('[]', '{"duplicate": 1, "duplicate": 2}', '{broken',
                     '{"idf.customExtraVars": []}', '{"clangd.arguments": "invalid"}',
                     '{"idf.cmakeCompilerArgs": [42]}'):
            self.settings.write_text(text, encoding="utf-8")
            with self.subTest(text=text), self.assertRaises(ValueError):
                switcher.switch_profile("s31")
            self.assertEqual(self.settings.read_text(encoding="utf-8"), text)

    def test_atomic_write_failure_keeps_original_settings(self):
        original = self.settings.read_bytes()
        with mock.patch.object(switcher.os, "replace", side_effect=OSError("write blocked")), \
                self.assertRaisesRegex(OSError, "write blocked"):
            switcher.switch_profile("s31")
        self.assertEqual(self.settings.read_bytes(), original)
        self.assertEqual(list(self.settings.parent.iterdir()), [self.settings])

    def test_interactive_choices_cancel_and_error_exit_status(self):
        for number, profile in enumerate(build.PROFILES, 1):
            with mock.patch("builtins.input", side_effect=["invalid", str(number)]), \
                    contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(switcher.main(["--manual"]), 0)
            self.assertEqual(json.loads(self.settings.read_text(encoding="utf-8"))[
                "idf.buildPath"], str(self.root / f"build_{profile}"))
        original = self.settings.read_bytes()
        for answer in ("q", EOFError(), KeyboardInterrupt()):
            with mock.patch("builtins.input", side_effect=[answer]), contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(switcher.main(["--manual"]), 0 if answer == "q" else 130)
            self.assertEqual(self.settings.read_bytes(), original)
        stderr = io.StringIO()
        with contextlib.redirect_stderr(stderr):
            self.assertEqual(switcher.main(["--profile", "s31", "--build-dir", "main"]), 1)
        self.assertIn("Unable to switch profile", stderr.getvalue())

    def test_chip_identity_mapping_checks_family_before_revision(self):
        cases = [("ESP32-P4", "0.1", "pre_v3"), ("ESP32-P4", "1.3", "pre_v3"),
                 ("ESP32-P4", "2.99", "pre_v3"), ("ESP32-P4", "3.0", "v3_0"),
                 ("ESP32-P4", "3.1", "v3_1"), ("ESP32-P4", "3.2", "v3_1"),
                 ("ESP32-P4", "3.10", "v3_1"), ("ESP32-P4", "4.0", "v3_1"),
                 ("ESP32-S31", "0.0", "s31"), ("ESP32-S31", "1.3", "s31"),
                 ("ESP32-S31", "3.0", "s31"), ("ESP32-S31", "3.2", "s31")]
        for chip, revision, expected in cases:
            description = f"{chip} (revision v{revision})"
            for prefix in ("Chip is ", "Chip type:          "):
                with self.subTest(chip=chip, revision=revision, prefix=prefix):
                    output = ("esptool v5.3.0\r\nConnecting...\r\n"
                              f"\x1b[1m{prefix}{description}\x1b[0m\r\n"
                              "Crystal frequency: 40MHz\r\n")
                    self.assertEqual(switcher.profile_from_chip_id(output), (expected, description))

    def test_incomplete_unsupported_and_ambiguous_chip_output_is_rejected(self):
        for output in ("Chip ID: 0x1234\n", "Chip is ESP32-P4\n", "Chip type: ESP32-S31\n",
                       "Chip is ESP32-S3 (revision v3.2)\n",
                       "Chip is Unknown ESP32-P4 (revision v3.2)\n",
                       "Chip is ESP32-P4 (revision 301)\n",
                       "Chip is ESP32-P4 (revision v3.x)\n",
                       "Chip type: ESP32-P4\nBlock revision v3.1\n",
                       "Chip is ESP32-P4 (revision v3.2)\nChip type: ESP32-S31 (revision v0.0)\n"):
            with self.subTest(output=output), self.assertRaises(ValueError):
                switcher.profile_from_chip_id(output)

    def test_default_detection_configures_each_profile_using_saved_port(self):
        for chip, revision, expected in (("ESP32-P4", "1.3", "pre_v3"),
                                         ("ESP32-P4", "3.0", "v3_0"),
                                         ("ESP32-P4", "3.2", "v3_1"),
                                         ("ESP32-S31", "0.0", "s31")):
            result = subprocess.CompletedProcess([], 0, f"Chip type: {chip} (revision v{revision})\n")
            with self.subTest(expected=expected), mock.patch.dict(os.environ, {}, clear=True), \
                    mock.patch.object(subprocess, "run", return_value=result) as run, \
                    mock.patch("builtins.input", side_effect=AssertionError("unexpected prompt")), \
                    contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(switcher.main([]), 0)
            run.assert_called_once_with(
                [sys.executable, "-m", "esptool", "-p", "COM12", "chip-id"],
                cwd=self.root, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, encoding="utf-8", errors="replace", timeout=45, check=False)
            settings = json.loads(self.settings.read_text(encoding="utf-8"))
            self.assertEqual(settings["idf.buildPath"], str(self.root / f"build_{expected}"))
            self.assertEqual(settings["idf.port"], "COM12")

    def test_explicit_port_is_used_and_saved_only_after_success(self):
        result = subprocess.CompletedProcess([], 0, "Chip is ESP32-P4 (revision v3.1)\n")
        with mock.patch.dict(os.environ, {}, clear=True), \
                mock.patch.object(subprocess, "run", return_value=result) as run, \
                contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(switcher.main(["--port", " COM9 "]), 0)
        self.assertEqual(run.call_args.args[0][-3:], ["-p", "COM9", "chip-id"])
        settings = json.loads(self.settings.read_text(encoding="utf-8"))
        self.assertEqual(settings["idf.port"], "COM9")
        self.assertEqual(settings["idf.buildPath"], str(self.root / "build_v3_1"))

    def test_missing_port_prompts_without_asking_for_profile(self):
        self.original.pop("idf.port")
        self.settings.write_text(json.dumps(self.original), encoding="utf-8")
        result = subprocess.CompletedProcess([], 0, "Chip type: ESP32-S31 (revision v0.0)\n")
        with mock.patch.dict(os.environ, {}, clear=True), \
                mock.patch.object(subprocess, "run", return_value=result) as run, \
                mock.patch("builtins.input", return_value="COM8") as prompt, \
                contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(switcher.main([]), 0)
        prompt.assert_called_once()
        self.assertIn("Serial port", prompt.call_args.args[0])
        self.assertEqual(run.call_args.args[0][-3:], ["-p", "COM8", "chip-id"])
        self.assertEqual(json.loads(self.settings.read_text(encoding="utf-8"))["idf.port"], "COM8")

    def test_detection_uses_loaded_idf_python_environment(self):
        self.eim_config.write_text("{invalid installer configuration", encoding="utf-8")
        python_env = self.root.parent / "idf-python"
        python = python_env / ("Scripts" if os.name == "nt" else "bin")
        python /= "python.exe" if os.name == "nt" else "python"
        python.parent.mkdir(parents=True)
        python.touch()
        result = subprocess.CompletedProcess([], 0, "Chip is ESP32-S31 (revision v0.0)\n")
        with mock.patch.dict(os.environ, {"IDF_PYTHON_ENV_PATH": str(python_env)}), \
                mock.patch.object(subprocess, "run", return_value=result) as run, \
                contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(switcher.detect_profile("COM12")[0], "s31")
        self.assertEqual(run.call_args.args[0][0], str(python))
        with mock.patch.dict(os.environ, {"IDF_PYTHON_ENV_PATH": str(python_env / "missing")}), \
                mock.patch.object(subprocess, "run") as run, \
                self.assertRaisesRegex(ValueError, "Python environment is unavailable"):
            switcher.detect_profile("COM12")
        run.assert_not_called()

    def test_eim_discovery_checks_custom_user_and_windows_default_locations(self):
        home = self.root.parent / "home"
        tools = self.root.parent / "custom tools"
        with mock.patch.dict(os.environ, {"IDF_TOOLS_PATH": str(tools)}, clear=True), \
                mock.patch.object(Path, "home", return_value=home):
            paths = self.discover_eim_paths()
        self.assertEqual(paths[:3], [tools / "eim_idf.json", home / ".espressif" / "eim_idf.json",
                                    home / ".espressif" / "tools" / "eim_idf.json"])
        if os.name == "nt":
            self.assertIn(Path("C:/Espressif/tools/eim_idf.json"), paths)
        with mock.patch.dict(os.environ, {"IDF_TOOLS_PATH": str(home / ".espressif")}, clear=True), \
                mock.patch.object(Path, "home", return_value=home):
            paths = self.discover_eim_paths()
        self.assertEqual(len(paths), len(set(paths)))

    def test_code_runner_uses_python_registered_for_workspace_idf_setup(self):
        python = self.root.parent / "registered Python" / "python.exe"
        python.parent.mkdir()
        python.touch()
        self.eim_config.write_text(json.dumps({"idfInstalled": [
            {"path": str(self.root.parent / "different IDF"), "python": "missing.exe"},
            {"path": self.original["idf.currentSetup"], "python": str(python)},
        ]}), encoding="utf-8-sig")
        result = subprocess.CompletedProcess([], 0, "Chip is ESP32-S31 (revision v0.0)\n")
        output = io.StringIO()
        with mock.patch.dict(os.environ, {"IDF_PATH": str(self.root.parent / "different IDF")}, clear=True), \
                mock.patch.object(subprocess, "run", return_value=result) as run, \
                contextlib.redirect_stdout(output):
            self.assertEqual(switcher.main([]), 0)
        run.assert_called_once_with(
            [str(python), "-m", "esptool", "-p", "COM12", "chip-id"],
            cwd=self.root, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, encoding="utf-8", errors="replace", timeout=45, check=False)
        self.assertIn(str(python), output.getvalue())
        self.assertEqual(json.loads(self.settings.read_text(encoding="utf-8"))["idf.buildPath"],
                         str(self.root / "build_s31"))

    def test_eim_lookup_uses_idf_path_when_workspace_has_no_setup(self):
        self.original.pop("idf.currentSetup")
        self.settings.write_text(json.dumps(self.original), encoding="utf-8")
        python = self.root.parent / "python.exe"
        python.touch()
        idf = self.root.parent / "selected IDF"
        self.eim_config.write_text(json.dumps({"idfInstalled": [
            {"path": str(idf), "python": str(python)}]}), encoding="utf-8")
        with mock.patch.dict(os.environ, {"IDF_PATH": str(idf)}, clear=True):
            self.assertEqual(switcher.esptool_python(), str(python))

    def test_eim_lookup_falls_back_without_guessing_a_different_installation(self):
        self.eim_config.write_text(json.dumps({"idfInstalled": [
            {"path": str(self.root.parent / "different IDF"), "python": "missing.exe"}]}), encoding="utf-8")
        with mock.patch.dict(os.environ, {}, clear=True):
            self.assertEqual(switcher.esptool_python(), sys.executable)
            self.original.pop("idf.currentSetup")
            self.settings.write_text(json.dumps(self.original), encoding="utf-8")
            self.assertEqual(switcher.esptool_python(), sys.executable)
            self.settings.unlink()
            self.assertEqual(switcher.esptool_python(), sys.executable)

    def test_invalid_eim_records_fail_before_contacting_the_board(self):
        original = self.settings.read_bytes()
        matching = {"path": self.original["idf.currentSetup"]}
        records = ["{broken", "[]", '{"idfInstalled": {}}',
                   '{"idfInstalled": [], "idfInstalled": []}',
                   json.dumps({"idfInstalled": [matching]}),
                   json.dumps({"idfInstalled": [{**matching, "python": "relative/python.exe"}]}),
                   json.dumps({"idfInstalled": [{**matching, "python": str(self.root.parent / "missing.exe")}]}),
                   json.dumps({"idfInstalled": [
                       {**matching, "python": str(self.root.parent / "one.exe")},
                       {**matching, "python": str(self.root.parent / "two.exe")}]}),
                   ]
        for record in records:
            self.eim_config.write_text(record, encoding="utf-8")
            with self.subTest(record=record), mock.patch.dict(os.environ, {}, clear=True), \
                    mock.patch.object(subprocess, "run") as run, \
                    contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(switcher.main([]), 1)
            run.assert_not_called()
            self.assertEqual(self.settings.read_bytes(), original)
            self.assertEqual(list(self.root.glob("build_*")), [])

    def test_repeated_eim_records_for_same_interpreter_are_accepted(self):
        python = self.root.parent / "python.exe"
        python.touch()
        entry = {"path": self.original["idf.currentSetup"], "python": str(python)}
        self.eim_config.write_text(json.dumps({"idfInstalled": [entry, entry]}), encoding="utf-8")
        with mock.patch.dict(os.environ, {}, clear=True):
            self.assertEqual(switcher.esptool_python(), str(python))

    def test_detection_failures_leave_settings_and_build_directories_unchanged(self):
        original = self.settings.read_bytes()
        results = [subprocess.CompletedProcess([], 2, "No module named esptool"),
                   subprocess.CompletedProcess([], 1, "Could not open COM9"),
                   subprocess.CompletedProcess([], 0, "Chip is ESP32-S3 (revision v3.2)\n"),
                   subprocess.CompletedProcess([], 0, "Chip is ESP32-P4\n"),
                   subprocess.TimeoutExpired([], 45), OSError("cannot start esptool")]
        for result in results:
            options = {"side_effect": result} if isinstance(result, Exception) else {"return_value": result}
            stderr = io.StringIO()
            with self.subTest(result=result), mock.patch.dict(os.environ, {}, clear=True), \
                    mock.patch.object(subprocess, "run", **options), \
                    contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(stderr):
                self.assertEqual(switcher.main(["--port", "COM9"]), 1)
            self.assertEqual(self.settings.read_bytes(), original)
            self.assertEqual(list(self.root.glob("build_*")), [])
            self.assertIn("Unable to switch profile", stderr.getvalue())

    def test_manual_overrides_do_not_contact_the_board(self):
        self.eim_config.write_text("{invalid installer configuration", encoding="utf-8")
        with mock.patch.object(subprocess, "run") as run, contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(switcher.main(["--profile", "s31"]), 0)
        run.assert_not_called()
        with mock.patch.object(subprocess, "run") as run, mock.patch("builtins.input", return_value="2"), \
                contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(switcher.main(["--manual"]), 0)
        run.assert_not_called()

    def test_invalid_detection_port_does_not_contact_the_board(self):
        original = self.settings.read_bytes()
        with mock.patch.object(subprocess, "run") as run, contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(switcher.main(["--port", "   "]), 1)
        run.assert_not_called()
        self.assertEqual(self.settings.read_bytes(), original)
        self.original["idf.port"] = ["COM12"]
        self.settings.write_text(json.dumps(self.original), encoding="utf-8")
        with mock.patch.object(subprocess, "run") as run, contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(switcher.main([]), 1)
        run.assert_not_called()
        self.assertEqual(json.loads(self.settings.read_text(encoding="utf-8")), self.original)

    def test_cli_help_from_another_working_directory(self):
        result = subprocess.run([sys.executable, "-B", str(PROJECT_ROOT / "tools" /
                                "switch_esp32type_for_building.py"), "--help"],
                                cwd=self.root.parent, capture_output=True, text=True, check=False)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("--profile {pre_v3,v3_0,v3_1,s31}", result.stdout)


if __name__ == "__main__":
    unittest.main()
