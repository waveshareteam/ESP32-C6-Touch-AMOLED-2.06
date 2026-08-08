#!/usr/bin/env python3
"""Create traceable, directly flashable firmware bundles from CI build output."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shlex
import zipfile
from datetime import datetime, timezone
from pathlib import Path


BOARD = "ESP32-C6-Touch-AMOLED-2.06"
CHIP = "esp32c6"
DEFAULT_BAUD = 921600
DEFAULT_ARDUINO_FQBN = (
    "esp32:esp32:esp32c6:"
    "CDCOnBoot=cdc,FlashSize=16M,PartitionScheme=app3M_fat9M_16MB"
)


def slugify(value: str) -> str:
    slug = re.sub(r"[^a-z0-9]+", "-", value.lower()).strip("-")
    return slug or "firmware"


def repo_relative(path: Path) -> str:
    resolved = path.resolve()
    try:
        return resolved.relative_to(Path.cwd().resolve()).as_posix()
    except ValueError:
        return path.name


def contained_path(root: Path, candidate: Path, description: str) -> Path:
    resolved_root = root.resolve()
    resolved_candidate = candidate.resolve()
    try:
        resolved_candidate.relative_to(resolved_root)
    except ValueError as error:
        raise ValueError(
            f"{description} resolves outside {resolved_root}: {resolved_candidate}"
        ) from error
    return resolved_candidate


def manifest_git_sha() -> str:
    value = os.environ.get("GITHUB_SHA", "")
    if os.environ.get("CI", "").lower() == "true" and not re.fullmatch(
        r"[0-9a-fA-F]{40}", value
    ):
        raise ValueError("CI packaging requires a complete 40-character GITHUB_SHA")
    return value or "unknown"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def parse_offset(value: str | int) -> int:
    if isinstance(value, int):
        return value
    return int(str(value), 0)


def unique_archive_name(path: Path, used: set[str]) -> str:
    candidate = f"bin/{path.name}"
    if candidate not in used:
        used.add(candidate)
        return candidate

    stem = path.stem
    suffix = path.suffix
    counter = 2
    while True:
        candidate = f"bin/{stem}-{counter}{suffix}"
        if candidate not in used:
            used.add(candidate)
            return candidate
        counter += 1


def write_text(archive: zipfile.ZipFile, name: str, text: str, executable: bool = False) -> None:
    info = zipfile.ZipInfo(name)
    info.external_attr = (0o755 if executable else 0o644) << 16
    archive.writestr(info, text.encode("utf-8"))


def flash_scripts(files: list[dict[str, object]], baud: int) -> tuple[str, str, str]:
    ordered = sorted(
        (item for item in files if item.get("offset") is not None),
        key=lambda item: parse_offset(str(item["offset"])),
    )
    if not ordered:
        raise ValueError("No flashable binary offsets were found")

    command_pairs = [
        (str(item["offset"]), str(item["archive_path"])) for item in ordered
    ]
    display_command = "python -m esptool --chip {} --baud {} write_flash {}".format(
        CHIP,
        baud,
        " ".join(f"{offset} {shlex.quote(path)}" for offset, path in command_pairs),
    )
    shell_pairs = " ".join(
        f'{offset} "$SCRIPT_DIR/{path}"' for offset, path in command_pairs
    )
    batch_pairs = " ".join(
        f'{offset} "%~dp0{path.replace("/", chr(92))}"' for offset, path in command_pairs
    )

    shell_script = (
        "#!/usr/bin/env sh\n"
        "set -eu\n"
        'SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)\n'
        f"python -m esptool \"$@\" --chip {CHIP} --baud {baud} write_flash {shell_pairs}\n"
    )
    batch_script = (
        "@echo off\r\n"
        f"python -m esptool %* --chip {CHIP} --baud {baud} write_flash {batch_pairs}\r\n"
        "if errorlevel 1 exit /b %errorlevel%\r\n"
    )
    return display_command, shell_script, batch_script


def build_manifest(
    *,
    framework: str,
    framework_version: str,
    project: Path,
    files: list[dict[str, object]],
    flash_command: str,
    fqbn: str | None = None,
) -> dict[str, object]:
    manifest: dict[str, object] = {
        "schema_version": 1,
        "board": BOARD,
        "chip": CHIP,
        "framework": framework,
        "framework_version": framework_version,
        "source_project": repo_relative(project),
        "git_sha": manifest_git_sha(),
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "flash": {
            "baud": DEFAULT_BAUD,
            "command": flash_command,
        },
        "files": files,
    }
    if fqbn:
        manifest["fqbn"] = fqbn
    return manifest


def package_name(project: Path, framework: str, framework_version: str) -> str:
    return "-".join(
        (
            slugify(BOARD),
            slugify(project.name),
            slugify(framework),
            slugify(framework_version),
        )
    ) + ".zip"


def write_bundle(
    *,
    output_path: Path,
    binary_sources: list[tuple[Path, str]],
    manifest: dict[str, object],
    shell_script: str,
    batch_script: str,
    extra_files: list[tuple[Path, str]] | None = None,
) -> Path:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(output_path, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        for source, archive_name in binary_sources:
            archive.write(source, archive_name)
        for source, archive_name in extra_files or []:
            archive.write(source, archive_name)
        write_text(archive, "manifest.json", json.dumps(manifest, indent=2) + "\n")
        write_text(archive, "flash.sh", shell_script, executable=True)
        write_text(archive, "flash.bat", batch_script)
    return output_path


def package_esp_idf(
    project: Path,
    build_dir: Path,
    framework_version: str,
    output_dir: Path,
) -> Path:
    args_path = build_dir / "flasher_args.json"
    if not args_path.is_file():
        raise FileNotFoundError(f"ESP-IDF flasher arguments not found: {args_path}")

    flasher_args = json.loads(args_path.read_text(encoding="utf-8"))
    flash_files = flasher_args.get("flash_files")
    if not isinstance(flash_files, dict) or not flash_files:
        raise ValueError(f"No flash_files map in {args_path}")

    used_names: set[str] = set()
    records: list[dict[str, object]] = []
    binary_sources: list[tuple[Path, str]] = []
    for raw_offset, raw_path in sorted(
        flash_files.items(), key=lambda item: parse_offset(item[0])
    ):
        source = contained_path(
            build_dir,
            build_dir / str(raw_path),
            "ESP-IDF flasher binary",
        )
        if not source.is_file():
            raise FileNotFoundError(f"Referenced ESP-IDF binary not found: {source}")
        archive_name = unique_archive_name(source, used_names)
        offset = f"0x{parse_offset(raw_offset):x}"
        records.append(
            {
                "offset": offset,
                "archive_path": archive_name,
                "size": source.stat().st_size,
                "sha256": sha256(source),
            }
        )
        binary_sources.append((source, archive_name))

    command, shell_script, batch_script = flash_scripts(records, DEFAULT_BAUD)
    manifest = build_manifest(
        framework="esp-idf",
        framework_version=framework_version,
        project=project,
        files=records,
        flash_command=command,
    )
    output_path = output_dir / package_name(project, "esp-idf", framework_version)
    return write_bundle(
        output_path=output_path,
        binary_sources=binary_sources,
        manifest=manifest,
        shell_script=shell_script,
        batch_script=batch_script,
        extra_files=[(args_path, "metadata/flasher_args.json")],
    )


def package_arduino(
    project: Path,
    build_dir: Path,
    framework_version: str,
    output_dir: Path,
    fqbn: str,
) -> Path:
    binaries = sorted(
        contained_path(build_dir, path, "Arduino binary")
        for path in build_dir.rglob("*.bin")
    )
    if not binaries:
        raise FileNotFoundError(f"No Arduino binaries found under {build_dir}")

    merged = [path for path in binaries if "merged" in path.name.lower()]
    flash_offsets: dict[Path, str] = {}
    if merged:
        if len(merged) != 1:
            raise ValueError(
                "Expected exactly one Arduino merged binary, found "
                f"{len(merged)}"
            )
        flash_offsets[merged[0]] = "0x0"
    else:
        special = {
            "bootloader": "0x0",
            "partitions": "0x8000",
            "boot_app0": "0xe000",
        }
        for marker, offset in special.items():
            match = next((path for path in binaries if marker in path.name.lower()), None)
            if match:
                flash_offsets[match] = offset
        app_candidates = [
            path
            for path in binaries
            if path not in flash_offsets
            and "merged" not in path.name.lower()
            and "bootloader" not in path.name.lower()
            and "partitions" not in path.name.lower()
        ]
        if not app_candidates:
            raise ValueError("Arduino build output does not contain an application binary")
        expected_names = {
            f"{project.name}.bin",
            f"{project.name}.ino.bin",
        }
        exact_app = [path for path in app_candidates if path.name in expected_names]
        if len(exact_app) == 1:
            application = exact_app[0]
        elif len(app_candidates) == 1:
            application = app_candidates[0]
        else:
            raise ValueError(
                "Unable to choose one Arduino application binary from: "
                + ", ".join(path.name for path in app_candidates)
            )
        flash_offsets[application] = "0x10000"

    used_names: set[str] = set()
    records: list[dict[str, object]] = []
    binary_sources: list[tuple[Path, str]] = []
    for source in binaries:
        archive_name = unique_archive_name(source, used_names)
        records.append(
            {
                "offset": flash_offsets.get(source),
                "archive_path": archive_name,
                "size": source.stat().st_size,
                "sha256": sha256(source),
            }
        )
        binary_sources.append((source, archive_name))

    command, shell_script, batch_script = flash_scripts(records, DEFAULT_BAUD)
    manifest = build_manifest(
        framework="arduino-esp32",
        framework_version=framework_version,
        project=project,
        files=records,
        flash_command=command,
        fqbn=fqbn,
    )
    output_path = output_dir / package_name(project, "arduino", framework_version)
    return write_bundle(
        output_path=output_path,
        binary_sources=binary_sources,
        manifest=manifest,
        shell_script=shell_script,
        batch_script=batch_script,
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="framework", required=True)
    for framework in ("esp-idf", "arduino"):
        subparser = subparsers.add_parser(framework)
        subparser.add_argument("--project", type=Path, required=True)
        subparser.add_argument("--build-dir", type=Path, required=True)
        subparser.add_argument("--framework-version", required=True)
        subparser.add_argument("--output-dir", type=Path, required=True)
        if framework == "arduino":
            subparser.add_argument("--fqbn", default=DEFAULT_ARDUINO_FQBN)
    args = parser.parse_args()

    if args.framework == "esp-idf":
        output = package_esp_idf(
            args.project, args.build_dir, args.framework_version, args.output_dir
        )
    else:
        output = package_arduino(
            args.project,
            args.build_dir,
            args.framework_version,
            args.output_dir,
            args.fqbn,
        )
    print(output.as_posix())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
