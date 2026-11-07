import contextlib
import importlib.util
import io
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest import mock


PROJECT_ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("time_server_build", PROJECT_ROOT / "tools" / "build.py")
if SPEC is None or SPEC.loader is None:
    raise RuntimeError("Unable to load the build helper")
build = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(build)


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


if __name__ == "__main__":
    unittest.main()
