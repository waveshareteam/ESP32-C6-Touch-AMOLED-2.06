from __future__ import annotations

import importlib.util
import json
import tempfile
import unittest
import zipfile
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Unable to load {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


idf_discovery = load_module(
    "idf_discovery", REPO_ROOT / ".github/scripts/discover_esp_idf_examples.py"
)
arduino_discovery = load_module(
    "arduino_discovery", REPO_ROOT / ".github/scripts/discover_arduino_examples.py"
)
packager = load_module(
    "firmware_packager", REPO_ROOT / "releases/package_firmware.py"
)


class DiscoveryTests(unittest.TestCase):
    def test_idf_discovery_only_accepts_direct_projects(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory) / "examples/esp-idf"
            project = root / "01_demo"
            nested = root / "component/test_apps/test_project"
            (project / "main").mkdir(parents=True)
            (project / "CMakeLists.txt").write_text("project(demo)\n", encoding="utf-8")
            (nested / "main").mkdir(parents=True)
            (nested / "CMakeLists.txt").write_text("project(test)\n", encoding="utf-8")
            self.assertEqual(idf_discovery.list_examples(root), [project.as_posix()])

    def test_arduino_discovery_excludes_bundled_library_examples(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory) / "examples/arduino"
            sketch = root / "01_demo"
            bundled = root / "libraries/vendor/examples/demo"
            sketch.mkdir(parents=True)
            bundled.mkdir(parents=True)
            (sketch / "01_demo.ino").write_text("void setup() {}\n", encoding="utf-8")
            (bundled / "demo.ino").write_text("void setup() {}\n", encoding="utf-8")
            self.assertEqual(arduino_discovery.list_examples(root), [sketch.as_posix()])

    def test_library_change_selects_all_arduino_sketches(self) -> None:
        known = {"examples/arduino/01_demo", "examples/arduino/02_demo"}
        selected = arduino_discovery.discover_from_paths(
            ["examples/arduino/libraries/vendor/src/file.cpp"], known
        )
        self.assertEqual(selected, sorted(known))


class PackageTests(unittest.TestCase):
    def test_esp_idf_bundle_contains_offsets_scripts_and_hashes(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary = Path(temporary_directory)
            project = temporary / "examples/esp-idf/01_demo"
            build = project / "build"
            (build / "bootloader").mkdir(parents=True)
            (build / "bootloader/bootloader.bin").write_bytes(b"boot")
            (build / "demo.bin").write_bytes(b"application")
            (build / "flasher_args.json").write_text(
                json.dumps(
                    {
                        "flash_files": {
                            "0x0": "bootloader/bootloader.bin",
                            "0x10000": "demo.bin",
                        }
                    }
                ),
                encoding="utf-8",
            )

            bundle = packager.package_esp_idf(project, build, "v5.5.5", temporary / "out")
            with zipfile.ZipFile(bundle) as archive:
                self.assertIn("flash.sh", archive.namelist())
                self.assertIn("flash.bat", archive.namelist())
                self.assertIn("metadata/flasher_args.json", archive.namelist())
                manifest = json.loads(archive.read("manifest.json"))
                self.assertEqual(manifest["chip"], "esp32c6")
                self.assertEqual(
                    [entry["offset"] for entry in manifest["files"]],
                    ["0x0", "0x10000"],
                )
                self.assertTrue(all(entry["sha256"] for entry in manifest["files"]))

    def test_arduino_bundle_prefers_merged_binary(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary = Path(temporary_directory)
            project = temporary / "examples/arduino/01_demo"
            build = temporary / "build"
            build.mkdir(parents=True)
            (build / "01_demo.ino.bin").write_bytes(b"application")
            (build / "01_demo.ino.merged.bin").write_bytes(b"merged")

            bundle = packager.package_arduino(
                project,
                build,
                "3.3.11",
                temporary / "out",
                packager.DEFAULT_ARDUINO_FQBN,
            )
            with zipfile.ZipFile(bundle) as archive:
                manifest = json.loads(archive.read("manifest.json"))
                flashable = [entry for entry in manifest["files"] if entry["offset"]]
                self.assertEqual(len(flashable), 1)
                self.assertEqual(flashable[0]["offset"], "0x0")
                self.assertIn("merged", flashable[0]["archive_path"])


if __name__ == "__main__":
    unittest.main()
