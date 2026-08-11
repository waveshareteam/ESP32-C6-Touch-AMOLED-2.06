from __future__ import annotations

import importlib.util
import json
import os
import subprocess
import sys
import tempfile
import unittest
import zipfile
from pathlib import Path
from unittest import mock


REPO_ROOT = Path(__file__).resolve().parents[2]


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Unable to load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


router = load_module(
    "ci_change_router", REPO_ROOT / ".github/scripts/route_ci_changes.py"
)
markdown_policy = load_module(
    "markdown_policy", REPO_ROOT / ".github/scripts/check_markdown.py"
)
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

    def test_manual_matrices_match_repository_inventory(self) -> None:
        idf_examples = idf_discovery.list_examples()
        arduino_examples = arduino_discovery.list_examples()
        self.assertEqual(len(idf_examples), 5)
        self.assertEqual(len(arduino_examples), 6)
        self.assertEqual(len(idf_discovery.build_matrix(idf_examples)["include"]), 10)
        self.assertEqual(
            len(arduino_discovery.build_matrix(arduino_examples)["include"]),
            6,
        )

    def test_manual_discovery_cli_writes_declared_workflow_outputs(self) -> None:
        expected_status = {
            "docs_only": "false",
            "firmware_touched": "false",
            "release_review_required": "false",
            "unknown_paths": "",
            "unknown_path_count": "0",
            "unknown_paths_truncated": "false",
        }
        for script in (
            "discover_esp_idf_examples.py",
            "discover_arduino_examples.py",
        ):
            with self.subTest(script=script), tempfile.TemporaryDirectory() as temporary_directory:
                output = Path(temporary_directory) / "github-output.txt"
                environment = os.environ.copy()
                environment["GITHUB_OUTPUT"] = str(output)
                result = subprocess.run(
                    [
                        sys.executable,
                        str(REPO_ROOT / ".github/scripts" / script),
                        "--example",
                        "all",
                    ],
                    cwd=REPO_ROOT,
                    env=environment,
                    check=False,
                    text=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                )
                self.assertEqual(result.returncode, 0, result.stderr)
                outputs = dict(
                    line.split("=", 1)
                    for line in output.read_text(encoding="utf-8").splitlines()
                )
                self.assertTrue({"matrix", "has_examples", "examples"}.issubset(outputs))
                self.assertEqual(
                    {name: outputs[name] for name in expected_status}, expected_status
                )


class RoutingTests(unittest.TestCase):
    idf = {
        "examples/esp-idf/01_demo",
        "examples/esp-idf/02_demo",
    }
    arduino = {
        "examples/arduino/01_demo",
        "examples/arduino/02_demo",
    }

    def route(self, *changes):
        return router.route_changes(list(changes), self.idf, self.arduino)

    def test_documentation_at_every_owned_depth_selects_no_examples(self) -> None:
        result = self.route(
            router.Change("M", "README.md"),
            router.Change("M", "examples/esp-idf/01_demo/README.md"),
            router.Change("M", "examples/arduino/01_demo/README.md"),
            router.Change("M", "examples/arduino/libraries/vendor/README.md"),
            router.Change("M", "firmware/README.md"),
        )
        self.assertTrue(result.docs_only)
        self.assertTrue(result.firmware_touched)
        self.assertEqual(result.idf_examples, [])
        self.assertEqual(result.arduino_examples, [])

    def test_direct_sources_select_only_affected_examples(self) -> None:
        result = self.route(
            router.Change("M", "examples/esp-idf/01_demo/main/main.c"),
            router.Change("M", "examples/arduino/02_demo/02_demo.ino"),
        )
        self.assertEqual(result.idf_examples, ["examples/esp-idf/01_demo"])
        self.assertEqual(result.arduino_examples, ["examples/arduino/02_demo"])

    def test_deleted_direct_source_selects_affected_example(self) -> None:
        result = self.route(
            router.Change("D", "examples/esp-idf/01_demo/main/removed.c")
        )
        self.assertEqual(result.idf_examples, ["examples/esp-idf/01_demo"])
        self.assertEqual(result.arduino_examples, [])

    def test_shared_library_and_workflow_inputs_select_expected_full_sets(self) -> None:
        library = self.route(
            router.Change(
                "M", "examples/arduino/libraries/vendor/src/library.cpp"
            )
        )
        self.assertEqual(library.arduino_examples, sorted(self.arduino))
        self.assertEqual(library.idf_examples, [])

        workflow = self.route(
            router.Change("M", ".github/workflows/esp-idf-examples.yml")
        )
        self.assertEqual(workflow.idf_examples, sorted(self.idf))
        self.assertEqual(workflow.arduino_examples, [])

        global_input = self.route(
            router.Change("M", ".github/scripts/route_ci_changes.py")
        )
        self.assertEqual(global_input.idf_examples, sorted(self.idf))
        self.assertEqual(global_input.arduino_examples, sorted(self.arduino))

    def test_firmware_source_and_artifact_never_enter_example_ci(self) -> None:
        result = self.route(
            router.Change("M", "firmware/project/main/main.c"),
            router.Change("M", "firmware/factory.bin"),
            router.Change("M", "firmware/recovery.zip"),
        )
        self.assertTrue(result.firmware_touched)
        self.assertTrue(result.release_review_required)
        self.assertEqual(result.idf_examples, [])
        self.assertEqual(result.arduino_examples, [])

    def test_rename_keeps_old_source_impact(self) -> None:
        result = self.route(
            router.Change(
                "R",
                "docs/renamed.md",
                old_path="examples/esp-idf/01_demo/main/old.c",
            )
        )
        self.assertEqual(result.idf_examples, ["examples/esp-idf/01_demo"])

    def test_unknown_non_document_path_fails_closed_to_all(self) -> None:
        result = self.route(router.Change("M", "tools/new_input.dat"))
        self.assertEqual(result.idf_examples, sorted(self.idf))
        self.assertEqual(result.arduino_examples, sorted(self.arduino))
        self.assertEqual(result.unknown_paths, ["tools/new_input.dat"])

    def test_empty_scope_is_operational_error(self) -> None:
        with self.assertRaises(router.RoutingError):
            router.route_changes([], self.idf, self.arduino)

    def test_zero_push_base_uses_empty_tree(self) -> None:
        completed = subprocess.CompletedProcess(
            args=[],
            returncode=0,
            stdout="A\0README.md\0",
            stderr="",
        )
        with mock.patch.object(router.subprocess, "run", return_value=completed) as run:
            changes = router.changes_from_git("0" * 40, "head-sha")
        self.assertEqual(changes, [router.Change("A", "README.md")])
        command = run.call_args.args[0]
        self.assertIn(router.EMPTY_TREE_SHA, command)
        self.assertIn("head-sha", command)
        self.assertNotIn("0" * 40, command)

    def test_github_output_uses_multiline_protocol(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            output = Path(temporary_directory) / "github-output.txt"
            with (
                mock.patch.dict(os.environ, {"GITHUB_OUTPUT": str(output)}),
                mock.patch.object(
                    router.uuid,
                    "uuid4",
                    return_value=mock.Mock(hex="fixed"),
                ),
            ):
                router.github_output("unknown_paths", "first\n$() `quoted`")
            self.assertEqual(
                output.read_text(encoding="utf-8"),
                "unknown_paths<<ci_router_fixed\n"
                "first\n$() `quoted`\n"
                "ci_router_fixed\n",
            )

    def test_github_unknown_path_summary_is_bounded(self) -> None:
        paths = [f"legacy/path-{index}.dat" for index in range(25)]
        visible, count, truncated = router.github_path_summary(paths)
        self.assertEqual(count, 25)
        self.assertTrue(truncated)
        self.assertEqual(visible.splitlines(), paths[: router.SUMMARY_PATH_LIMIT])
        self.assertNotIn(paths[-1], visible)

    def test_exact_docs_only_cli_contract(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            changed = Path(temporary_directory) / "changed.txt"
            changed.write_text("M\tREADME.md\n", encoding="utf-8")
            result = subprocess.run(
                [
                    sys.executable,
                    str(REPO_ROOT / ".github/scripts/route_ci_changes.py"),
                    "--framework",
                    "esp-idf",
                    "--changed-files-from",
                    str(changed),
                    "--expect-docs-only",
                    "--expect-no-example-builds",
                ],
                cwd=REPO_ROOT,
                check=False,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            self.assertEqual(result.returncode, 0, result.stderr)
            payload = json.loads(result.stdout)
            self.assertTrue(payload["scope"]["docs_only"])
            self.assertFalse(payload["scope"]["example_build_required"])

    def test_workflows_consume_router_and_policy_exactly(self) -> None:
        idf_workflow = (
            REPO_ROOT / ".github/workflows/esp-idf-examples.yml"
        ).read_text(encoding="utf-8")
        arduino_workflow = (
            REPO_ROOT / ".github/workflows/arduino-examples.yml"
        ).read_text(encoding="utf-8")
        policy_workflow = (
            REPO_ROOT / ".github/workflows/repository-policy.yml"
        ).read_text(encoding="utf-8")
        for workflow, framework in (
            (idf_workflow, "esp-idf"),
            (arduino_workflow, "arduino"),
        ):
            self.assertIn(
                f"route_ci_changes.py \\\n              --framework {framework}",
                workflow,
            )
            self.assertNotIn("--fallback-all", workflow)
            self.assertNotIn("    paths:", workflow)
            self.assertIn("SELECTED_EXAMPLES: ${{ steps.examples.outputs.examples }}", workflow)
            self.assertIn(
                "UNKNOWN_PATH_COUNT: ${{ steps.examples.outputs.unknown_path_count }}",
                workflow,
            )
            self.assertNotIn("echo '${{ steps.examples.outputs.examples }}'", workflow)
        self.assertIn(
            "check_markdown.py --all --config config/markdown-audit.json",
            policy_workflow,
        )


class MarkdownPolicyTests(unittest.TestCase):
    def test_current_repository_passes_exact_policy_command(self) -> None:
        result = subprocess.run(
            [
                sys.executable,
                str(REPO_ROOT / ".github/scripts/check_markdown.py"),
                "--all",
                "--config",
                "config/markdown-audit.json",
                "--format",
                "json",
            ],
            cwd=REPO_ROOT,
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        self.assertEqual(result.returncode, 0, result.stderr or result.stdout)
        payload = json.loads(result.stdout)
        self.assertEqual(payload["findings"], 0)

    def test_homepage_contract_is_loaded_from_config(self) -> None:
        config = markdown_policy.load_policy_config(
            REPO_ROOT,
            Path("config/markdown-audit.json"),
        )
        changed = json.loads(json.dumps(config))
        changed["homepage_pairs"][0]["required_h2_icons"] = ["❌"]
        findings = markdown_policy.homepage_findings(REPO_ROOT, changed)
        self.assertIn(
            "HOMEPAGE_H2_SEQUENCE_MISMATCH",
            {finding.code for finding in findings},
        )

    def test_product_homepage_policy_is_explicit(self) -> None:
        config = markdown_policy.load_policy_config(
            REPO_ROOT,
            Path("config/markdown-audit.json"),
        )
        homepage = config["homepage_pairs"][0]
        self.assertEqual(homepage["profile"], "single-product")
        self.assertIn("hero_image", homepage["required_components"])
        self.assertIn("product", homepage["required_quick_links"])
        self.assertEqual(
            config["docs_only_allowed_patterns"],
            ["docs/assets/esp32-c6-touch-amoled-2.06.jpg"],
        )

    def test_link_and_fragment_failures_are_reported(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            source = root / "README.md"
            target = root / "GUIDE.md"
            source.write_text(
                "[missing](NOPE.md)\n[fragment](GUIDE.md#absent)\n",
                encoding="utf-8",
            )
            target.write_text("# Present\n", encoding="utf-8")
            findings = markdown_policy.link_findings(
                root, source, source.read_text(encoding="utf-8")
            )
            self.assertEqual(
                {finding.code for finding in findings},
                {"RELATIVE_LINK_MISSING", "RELATIVE_LINK_FRAGMENT_MISSING"},
            )

    def test_sensitive_shapes_are_reported_without_echoing_values(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            source = root / "README.md"
            source.write_text(
                "flash.bat --port COM9\n"
                "local path C:\\Users\\example\\build\n",
                encoding="utf-8",
            )
            findings = markdown_policy.sensitive_findings(root, [source])
            self.assertEqual(
                {finding.code for finding in findings},
                {"ACTUAL_SERIAL_PORT", "LOCAL_WINDOWS_PATH"},
            )
            self.assertTrue(
                all("COM9" not in finding.message for finding in findings)
            )


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

    def test_esp_idf_bundle_rejects_path_outside_build_directory(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary = Path(temporary_directory)
            project = temporary / "examples/esp-idf/01_demo"
            build = project / "build"
            build.mkdir(parents=True)
            outside = project / "outside.bin"
            outside.write_bytes(b"outside")
            (build / "flasher_args.json").write_text(
                json.dumps({"flash_files": {"0x0": "../outside.bin"}}),
                encoding="utf-8",
            )
            with self.assertRaises(ValueError):
                packager.package_esp_idf(
                    project, build, "v5.5.5", temporary / "out"
                )

    def test_arduino_bundle_rejects_ambiguous_merged_binary(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary = Path(temporary_directory)
            project = temporary / "examples/arduino/01_demo"
            build = temporary / "build"
            build.mkdir(parents=True)
            (build / "first.merged.bin").write_bytes(b"first")
            (build / "second.merged.bin").write_bytes(b"second")
            with self.assertRaises(ValueError):
                packager.package_arduino(
                    project,
                    build,
                    "3.3.11",
                    temporary / "out",
                    packager.DEFAULT_ARDUINO_FQBN,
                )

    def test_arduino_bundle_requires_application_binary(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary = Path(temporary_directory)
            project = temporary / "examples/arduino/01_demo"
            build = temporary / "build"
            build.mkdir(parents=True)
            (build / "bootloader.bin").write_bytes(b"bootloader")
            (build / "partitions.bin").write_bytes(b"partitions")
            with self.assertRaises(ValueError):
                packager.package_arduino(
                    project,
                    build,
                    "3.3.11",
                    temporary / "out",
                    packager.DEFAULT_ARDUINO_FQBN,
                )

    def test_ci_manifest_requires_complete_git_sha(self) -> None:
        with mock.patch.dict(
            os.environ,
            {"CI": "true", "GITHUB_SHA": "short"},
        ):
            with self.assertRaises(ValueError):
                packager.manifest_git_sha()

    def test_ci_manifest_prefers_explicit_package_sha(self) -> None:
        package_sha = "a" * 40
        with mock.patch.dict(
            os.environ,
            {"CI": "true", "GITHUB_SHA": "b" * 40, "PACKAGE_GIT_SHA": package_sha},
            clear=False,
        ):
            self.assertEqual(packager.manifest_git_sha(), package_sha)


class RuntimeRegressionTests(unittest.TestCase):
    def test_flasher_changes_are_known_global_build_inputs(self) -> None:
        result = router.route_changes(
            [
                router.Change("M", "Flash-CI-Firmware.cmd"),
                router.Change("M", "scripts/Flash-CI-Firmware.ps1"),
            ]
        )
        self.assertTrue(result.all_idf)
        self.assertTrue(result.all_arduino)
        self.assertEqual(result.unknown_paths, [])

    def test_spec_analyzer_pins_c6_to_i2s0_at_configure_and_compile_time(self) -> None:
        defaults = (REPO_ROOT / "examples/esp-idf/05_Spec_Analyzer/sdkconfig.defaults").read_text(encoding="utf-8")
        source = (REPO_ROOT / "examples/esp-idf/05_Spec_Analyzer/components/bsp_extra/src/bsp_board_extra.c").read_text(encoding="utf-8")
        self.assertIn("CONFIG_BSP_I2S_NUM=0", defaults)
        self.assertIn('#include "sdkconfig.h"', source)
        self.assertIn("CONFIG_BSP_I2S_NUM != 0", source)

    def test_arduino_runtime_examples_use_small_safe_partial_rendering(self) -> None:
        examples = [
            "03_LVGL_PCF85063_simpleTime/03_LVGL_PCF85063_simpleTime.ino",
            "04_LVGL_QMI8658_ui/04_LVGL_QMI8658_ui.ino",
            "06_LVGL_Arduino_v9/06_LVGL_Arduino_v9.ino",
        ]
        for relative in examples:
            with self.subTest(example=relative):
                sketch = (REPO_ROOT / "examples/arduino" / relative).read_text(encoding="utf-8")
                self.assertIn("LVGL_BUFFER_LINES = 20", sketch)
                self.assertIn("uint8_t *disp_draw_buf", sketch)
                self.assertIn("lv_color_format_get_size(LV_COLOR_FORMAT_RGB565)", sketch)
                self.assertIn("lv_display_set_color_format(disp, LV_COLOR_FORMAT_RGB565)", sketch)
                self.assertNotIn("sizeof(lv_color_t)", sketch)
                self.assertIn("LV_DISPLAY_RENDER_MODE_PARTIAL", sketch)
                self.assertIn("if (!lvgl_ready)", sketch)
                self.assertIn("attempt <= 5", sketch)
                self.assertIn("delay(500)", sketch)
                self.assertIn("IIC_Interrupt_Flag", sketch)
                self.assertIn("area->x1 = 0", sketch)
                self.assertNotIn("screenWidth * screenHeight", sketch)
                self.assertNotIn("while (FT3168->begin()", sketch)
        clock = (REPO_ROOT / "examples/arduino" / examples[0]).read_text(encoding="utf-8")
        self.assertNotIn("DIRECT_RENDER_MODE", clock)
        self.assertIn("RTC unavailable", clock)
        qmi = (REPO_ROOT / "examples/arduino" / examples[1]).read_text(encoding="utf-8")
        self.assertIn("QMI8658 unavailable", qmi)
        self.assertIn("acc.x * 1000.0f", qmi)
        self.assertIn("-4000, 4000", qmi)
        status = (REPO_ROOT / "examples/arduino" / examples[2]).read_text(encoding="utf-8")
        self.assertNotIn("lv_demo_widgets()", status)
        self.assertIn("Uptime:", status)

    def test_build_checkouts_pin_artifacts_to_pr_head(self) -> None:
        expected = "ref: ${{ github.event.pull_request.head.sha || github.sha }}"
        for filename in ("esp-idf-examples.yml", "arduino-examples.yml"):
            workflow = (REPO_ROOT / ".github/workflows" / filename).read_text(encoding="utf-8")
            build = workflow.split("  build:", 1)[1]
            self.assertIn(expected, build)
            self.assertIn("PACKAGE_GIT_SHA: ${{ github.event.pull_request.head.sha || github.sha }}", build)

    def test_flasher_resolves_current_identity_and_rejects_bad_packages(self) -> None:
        flasher = (REPO_ROOT / "scripts/Flash-CI-Firmware.ps1").read_text(encoding="utf-8")
        self.assertIn("state-v3.json", flasher)
        self.assertIn("rev-parse HEAD", flasher)
        self.assertIn("--commit $FinalSha", flasher)
        self.assertIn("Test-PackageManifest", flasher)
        self.assertIn("function Get-FileSha256", flasher)
        self.assertIn("[System.IO.File]::OpenRead($Path)", flasher)
        self.assertIn("[System.Security.Cryptography.SHA256]::Create()", flasher)
        self.assertIn("[System.BitConverter]::ToString($algorithm.ComputeHash($stream)).Replace('-', '').ToLowerInvariant()", flasher)
        self.assertIn("$stream.Dispose()", flasher)
        self.assertIn("$algorithm.Dispose()", flasher)
        self.assertIn("$actual = Get-FileSha256 $fullPath", flasher)
        self.assertNotIn("Get-FileHash", flasher)
        self.assertNotIn("Convert.ToHexString", flasher)
        self.assertIn("Test-RelativePackagePath", flasher)
        self.assertIn("SourceProject", flasher)
        self.assertIn("Manifest flash offset is invalid", flasher)
        self.assertIn("overlapping flash ranges", flasher)
        self.assertIn("[int64]$file.size -le 0", flasher)
        self.assertIn("921600", flasher)
        self.assertIn("'esptool'", flasher)
        self.assertIn("finalSHA=resolved-at-runtime", flasher)
        self.assertIn("auto-detect-at-runtime", flasher)
        self.assertNotRegex(flasher, r"(?i)\b[0-9a-f]{40}\b")
        self.assertNotRegex(flasher, r"(?m)\bRun\s*=\s*['\"]?\d{8,}")
        self.assertNotRegex(flasher, r"(?i)[A-Z]:\\Users\\[^\\\r\n]+")
        self.assertNotIn("idf5.5_py3.13_env", flasher)
        self.assertNotRegex(flasher, r"['\"]COM\d+['\"]")
        self.assertNotIn("flash.bat", flasher)

    def test_flasher_sha256_helper_uses_powershell_5_compatible_apis(self) -> None:
        flasher = (REPO_ROOT / "scripts/Flash-CI-Firmware.ps1").read_text(encoding="utf-8")
        helper = flasher.split("function Get-FileSha256", 1)[1].split("if ($SelfTest)", 1)[0]
        self.assertIn("[System.IO.File]::OpenRead($Path)", helper)
        self.assertIn("[System.Security.Cryptography.SHA256]::Create()", helper)
        self.assertIn("[System.BitConverter]::ToString", helper)
        self.assertIn("$stream.Dispose()", helper)
        self.assertIn("$algorithm.Dispose()", helper)
        self.assertNotIn("Get-FileHash", helper)
        self.assertNotIn("Convert.ToHexString", helper)

    def test_flasher_python_probe_tolerates_native_stderr_and_tries_later_candidates(self) -> None:
        flasher = (REPO_ROOT / "scripts/Flash-CI-Firmware.ps1").read_text(encoding="utf-8")
        resolver = flasher.split("function Resolve-PythonWithEsptool", 1)[1].split(
            "function Resolve-FinalSha", 1
        )[0]
        self.assertIn("$savedErrorActionPreference = $ErrorActionPreference", resolver)
        self.assertIn("$ErrorActionPreference = 'Continue'", resolver)
        self.assertIn("& $candidate -c 'import esptool' *> $null", resolver)
        self.assertIn("$probeExitCode = $LASTEXITCODE", resolver)
        self.assertIn("catch {", resolver)
        self.assertIn("$probeExitCode = -1", resolver)
        self.assertIn("$ErrorActionPreference = $savedErrorActionPreference", resolver)
        self.assertLess(resolver.index("$probeExitCode = $LASTEXITCODE"), resolver.index("if ($probeExitCode -eq 0) { return $candidate }"))
        self.assertIn("foreach ($candidate", resolver)
        self.assertIn("throw 'No Python interpreter with esptool was found.'", resolver)

    def test_flasher_requires_a_verified_device_and_ready_pr_identity(self) -> None:
        flasher = (REPO_ROOT / "scripts/Flash-CI-Firmware.ps1").read_text(encoding="utf-8")
        port_resolver = flasher.split("function Resolve-DefaultPort", 1)[1].split(
            "function Resolve-Executable", 1
        )[0]
        self.assertIn("VID_303A&PID_1001", port_resolver)
        self.assertIn("$pnpPorts.Count -eq 1", port_resolver)
        self.assertNotIn("SerialPort]::GetPortNames", port_resolver)
        self.assertIn("status --porcelain=v1 --untracked-files=all", flasher)
        self.assertIn("symbolic-ref --quiet --short HEAD", flasher)
        self.assertIn("pr list --repo $Repo --head $Branch --state open", flasher)
        self.assertIn("number,state,isDraft,headRefName,headRefOid", flasher)
        self.assertIn("$pullRequests.Count -ne 1", flasher)
        self.assertIn("[bool]$pullRequest.isDraft", flasher)
        self.assertIn("$headRefOid -notmatch '^[0-9a-fA-F]{40}$'", flasher)
        self.assertLess(flasher.index("if ($SelfTest)"), flasher.index("function Resolve-DefaultPort"))
        self.assertLess(flasher.index("if ($ListOnly)"), flasher.index("function Resolve-DefaultPort"))


if __name__ == "__main__":
    unittest.main()
