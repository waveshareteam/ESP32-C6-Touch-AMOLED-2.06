#!/usr/bin/env python3
"""Classify a complete Git change set and select product example CI matrices.

The router is intentionally repository-specific. It keeps documentation and
delivered firmware out of source-build matrices, selects a directly affected
first-party example when possible, and conservatively selects every applicable
example for an unknown non-document path. Missing or empty diff data is an
operational error rather than a successful no-op.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
import uuid
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable


REPO_ROOT = Path(__file__).resolve().parents[2]
EMPTY_TREE_SHA = "4b825dc642cb6eb9a060e54bf8d69288fbee4904"
ZERO_SHA_RE = re.compile(r"^0{40,64}$")
SUMMARY_PATH_LIMIT = 20
TOOLCHAINS = json.loads(
    (REPO_ROOT / "config/toolchains.json").read_text(encoding="utf-8")
)
IDF_ROOT = Path("examples/esp-idf")
ARDUINO_ROOT = Path("examples/arduino")

DOCUMENT_EXTENSIONS = {
    ".md",
    ".markdown",
    ".mdown",
    ".png",
    ".jpg",
    ".jpeg",
    ".gif",
    ".webp",
    ".svg",
    ".pdf",
}
ARCHIVE_EXTENSIONS = {".zip", ".tar", ".gz", ".tgz", ".7z", ".rar"}
FIRMWARE_EXTENSIONS = {".bin"}

NON_BUILD_PREFIXES = (
    ".github/ISSUE_TEMPLATE/",
)
NON_BUILD_EXACT = {
    ".gitignore",
    ".github/pull_request_template.md",
    ".github/workflows/repository-policy.yml",
    ".github/scripts/check_markdown.py",
}
BOTH_FRAMEWORK_GLOBALS = {
    ".github/scripts/route_ci_changes.py",
    ".github/scripts/test_ci_tools.py",
    "Flash-CI-Firmware.cmd",
    "config/toolchains.json",
    "releases/package_firmware.py",
    "scripts/Flash-CI-Firmware.ps1",
}
IDF_GLOBALS = {
    ".github/scripts/discover_esp_idf_examples.py",
    ".github/workflows/esp-idf-examples.yml",
}
ARDUINO_GLOBALS = {
    ".github/scripts/discover_arduino_examples.py",
    ".github/workflows/arduino-examples.yml",
}


class RoutingError(RuntimeError):
    """The change set could not be obtained or classified safely."""


@dataclass(frozen=True)
class Change:
    status: str
    path: str
    old_path: str | None = None


@dataclass(frozen=True)
class PathRoute:
    path: str
    category: str
    reason: str


@dataclass
class RoutingResult:
    changes: list[Change]
    routes: list[PathRoute]
    idf_examples: list[str]
    arduino_examples: list[str]
    all_idf: bool
    all_arduino: bool
    docs_only: bool
    firmware_touched: bool
    release_review_required: bool
    unknown_paths: list[str]


def normalize_path(value: str) -> str:
    normalized = value.strip().replace("\\", "/").strip("/")
    if not normalized or normalized == ".":
        raise RoutingError("Changed paths must be non-empty repository-relative paths")
    path = Path(normalized)
    if path.is_absolute() or ".." in path.parts:
        raise RoutingError(f"Unsafe changed path: {value}")
    return normalized


def parse_name_status_z(output: str) -> list[Change]:
    tokens = output.split("\0")
    if tokens and tokens[-1] == "":
        tokens.pop()
    changes: list[Change] = []
    index = 0
    while index < len(tokens):
        status_token = tokens[index]
        index += 1
        if not status_token:
            raise RoutingError("Malformed empty Git status token")
        status = status_token[0]
        if status in {"R", "C"}:
            if index + 1 >= len(tokens):
                raise RoutingError("Malformed rename/copy record in Git diff")
            old_path = normalize_path(tokens[index])
            new_path = normalize_path(tokens[index + 1])
            index += 2
            changes.append(Change(status=status, path=new_path, old_path=old_path))
        elif status in {"A", "D", "M", "T", "U"}:
            if index >= len(tokens):
                raise RoutingError("Malformed path record in Git diff")
            changes.append(Change(status=status, path=normalize_path(tokens[index])))
            index += 1
        else:
            raise RoutingError(f"Unsupported Git change status: {status_token}")
    return changes


def changes_from_git(base_ref: str, head_ref: str) -> list[Change]:
    if not base_ref:
        raise RoutingError("A non-empty base ref is required")
    diff_refs = (
        [EMPTY_TREE_SHA, head_ref]
        if ZERO_SHA_RE.fullmatch(base_ref)
        else [f"{base_ref}...{head_ref}"]
    )
    command = [
        "git",
        "diff",
        "--name-status",
        "-z",
        "--find-renames",
        *diff_refs,
    ]
    try:
        result = subprocess.run(
            command,
            cwd=REPO_ROOT,
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            encoding="utf-8",
            errors="surrogateescape",
        )
    except (OSError, subprocess.CalledProcessError) as error:
        detail = getattr(error, "stderr", "") or str(error)
        raise RoutingError(f"Unable to read complete Git diff: {detail.strip()}") from error
    changes = parse_name_status_z(result.stdout)
    if not changes:
        raise RoutingError("The changed-file scope is empty; refusing a silent no-op")
    return changes


def changes_from_file(path: Path) -> list[Change]:
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError as error:
        raise RoutingError(f"Unable to read changed-file input: {error}") from error

    changes: list[Change] = []
    for raw_line in lines:
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        fields = line.split("\t")
        if len(fields) == 1:
            changes.append(Change("M", normalize_path(fields[0])))
            continue
        status = fields[0][:1]
        if status in {"R", "C"} and len(fields) == 3:
            changes.append(
                Change(
                    status,
                    normalize_path(fields[2]),
                    old_path=normalize_path(fields[1]),
                )
            )
        elif status in {"A", "D", "M", "T", "U"} and len(fields) == 2:
            changes.append(Change(status, normalize_path(fields[1])))
        else:
            raise RoutingError(f"Malformed changed-file line: {raw_line}")
    if not changes:
        raise RoutingError("The changed-file scope is empty; refusing a silent no-op")
    return changes


def list_idf_examples(root: Path = IDF_ROOT) -> list[str]:
    absolute_root = root if root.is_absolute() else REPO_ROOT / root
    if not absolute_root.is_dir():
        return []
    return sorted(
        path.relative_to(REPO_ROOT).as_posix()
        for path in absolute_root.iterdir()
        if path.is_dir()
        and (path / "CMakeLists.txt").is_file()
        and (path / "main").is_dir()
    )


def list_arduino_examples(root: Path = ARDUINO_ROOT) -> list[str]:
    absolute_root = root if root.is_absolute() else REPO_ROOT / root
    if not absolute_root.is_dir():
        return []
    return sorted(
        path.relative_to(REPO_ROOT).as_posix()
        for path in absolute_root.iterdir()
        if path.is_dir() and (path / f"{path.name}.ino").is_file()
    )


def impacted_paths(change: Change) -> Iterable[str]:
    if change.old_path:
        yield change.old_path
    yield change.path


def is_documentation(path: str) -> bool:
    pure = Path(path)
    name_upper = pure.name.upper()
    if pure.suffix.lower() in DOCUMENT_EXTENSIONS:
        return True
    if name_upper.startswith(("LICENSE", "COPYING", "NOTICE")):
        return True
    if path.startswith(("docs/", "Schematic/")):
        return True
    return False


def under(path: str, root: str) -> bool:
    return path == root or path.startswith(root + "/")


def direct_example(path: str, examples: Iterable[str]) -> str | None:
    return next((example for example in examples if under(path, example)), None)


def route_changes(
    changes: list[Change],
    known_idf: Iterable[str] | None = None,
    known_arduino: Iterable[str] | None = None,
) -> RoutingResult:
    if not changes:
        raise RoutingError("The changed-file scope is empty; refusing a silent no-op")

    idf_available = sorted(known_idf if known_idf is not None else list_idf_examples())
    arduino_available = sorted(
        known_arduino if known_arduino is not None else list_arduino_examples()
    )
    if not idf_available or not arduino_available:
        raise RoutingError("Expected first-party ESP-IDF and Arduino examples were not found")

    idf_selected: set[str] = set()
    arduino_selected: set[str] = set()
    all_idf = False
    all_arduino = False
    docs_only = True
    firmware_touched = False
    release_review_required = False
    unknown_paths: set[str] = set()
    routes: list[PathRoute] = []
    seen_paths: set[str] = set()

    for change in changes:
        for path in impacted_paths(change):
            if path in seen_paths:
                continue
            seen_paths.add(path)
            suffix = Path(path).suffix.lower()

            if path.startswith("firmware/"):
                firmware_touched = True
                if suffix in FIRMWARE_EXTENSIONS | ARCHIVE_EXTENSIONS:
                    release_review_required = True

            if is_documentation(path):
                routes.append(PathRoute(path, "documentation", "documentation file or asset"))
                continue

            docs_only = False

            if path.startswith("firmware/"):
                routes.append(
                    PathRoute(
                        path,
                        "firmware",
                        "maintainer-directed firmware surface outside example CI",
                    )
                )
                continue

            if path in NON_BUILD_EXACT or path.startswith(NON_BUILD_PREFIXES):
                routes.append(PathRoute(path, "policy", "non-build repository policy input"))
                continue

            if path in BOTH_FRAMEWORK_GLOBALS:
                all_idf = True
                all_arduino = True
                if path == "releases/package_firmware.py":
                    release_review_required = True
                routes.append(
                    PathRoute(path, "global_build", "shared CI or packaging input")
                )
                continue

            if path in IDF_GLOBALS:
                all_idf = True
                routes.append(PathRoute(path, "idf_global", "ESP-IDF workflow input"))
                continue

            if path in ARDUINO_GLOBALS:
                all_arduino = True
                routes.append(PathRoute(path, "arduino_global", "Arduino workflow input"))
                continue

            idf_example = direct_example(path, idf_available)
            if idf_example:
                idf_selected.add(idf_example)
                routes.append(
                    PathRoute(path, "idf_example", f"direct input for {idf_example}")
                )
                continue

            arduino_example = direct_example(path, arduino_available)
            if arduino_example:
                arduino_selected.add(arduino_example)
                routes.append(
                    PathRoute(
                        path,
                        "arduino_example",
                        f"direct input for {arduino_example}",
                    )
                )
                continue

            if path.startswith("examples/arduino/libraries/"):
                all_arduino = True
                routes.append(
                    PathRoute(path, "arduino_shared", "bundled Arduino library input")
                )
                continue

            if path.startswith("examples/esp-idf/"):
                all_idf = True
                routes.append(
                    PathRoute(
                        path,
                        "idf_shared_or_new",
                        "shared, new, renamed, or removed ESP-IDF input",
                    )
                )
                continue

            if path.startswith("examples/arduino/"):
                all_arduino = True
                routes.append(
                    PathRoute(
                        path,
                        "arduino_shared_or_new",
                        "shared, new, renamed, or removed Arduino input",
                    )
                )
                continue

            if path.startswith("config/"):
                all_idf = True
                all_arduino = True
                routes.append(PathRoute(path, "global_build", "shared configuration input"))
                continue

            if path.startswith(".github/workflows/"):
                all_idf = True
                all_arduino = True
                routes.append(PathRoute(path, "global_build", "unclassified workflow input"))
                continue

            if path.startswith("releases/"):
                all_idf = True
                all_arduino = True
                release_review_required = True
                routes.append(PathRoute(path, "release_build", "release tooling input"))
                continue

            if suffix in FIRMWARE_EXTENSIONS | ARCHIVE_EXTENSIONS:
                release_review_required = True

            all_idf = True
            all_arduino = True
            unknown_paths.add(path)
            routes.append(
                PathRoute(
                    path,
                    "unknown_build_impact",
                    "unknown non-document input; conservatively selecting all examples",
                )
            )

    if all_idf:
        idf_selected = set(idf_available)
    if all_arduino:
        arduino_selected = set(arduino_available)

    return RoutingResult(
        changes=changes,
        routes=routes,
        idf_examples=sorted(idf_selected),
        arduino_examples=sorted(arduino_selected),
        all_idf=all_idf,
        all_arduino=all_arduino,
        docs_only=docs_only,
        firmware_touched=firmware_touched,
        release_review_required=release_review_required,
        unknown_paths=sorted(unknown_paths),
    )


def slugify(value: str) -> str:
    return re.sub(r"[^a-z0-9]+", "-", value.lower()).strip("-") or "example"


def framework_matrix(framework: str, selected: list[str]) -> dict[str, list[dict[str, str]]]:
    if framework == "esp-idf":
        return {
            "include": [
                {
                    "example": example,
                    "name": slugify(Path(example).name),
                    "idf_version": version,
                    "target": str(TOOLCHAINS["target"]),
                }
                for example in selected
                for version in TOOLCHAINS["esp_idf_versions"]
            ]
        }
    return {
        "include": [
            {
                "sketch": sketch,
                "name": slugify(Path(sketch).name),
                "arduino_core_version": str(TOOLCHAINS["arduino_core_version"]),
                "fqbn": str(TOOLCHAINS["arduino_fqbn"]),
                "board_manager_url": str(TOOLCHAINS["arduino_board_manager_url"]),
            }
            for sketch in selected
        ]
    }


def github_output(name: str, value: str) -> None:
    output_path = os.environ.get("GITHUB_OUTPUT")
    if output_path:
        delimiter = f"ci_router_{uuid.uuid4().hex}"
        while delimiter in value:
            delimiter = f"ci_router_{uuid.uuid4().hex}"
        with open(output_path, "a", encoding="utf-8") as output:
            output.write(f"{name}<<{delimiter}\n{value}\n{delimiter}\n")


def github_path_summary(paths: list[str]) -> tuple[str, int, bool]:
    return (
        "\n".join(paths[:SUMMARY_PATH_LIMIT]),
        len(paths),
        len(paths) > SUMMARY_PATH_LIMIT,
    )


def report(result: RoutingResult, framework: str) -> dict[str, object]:
    selected = (
        result.idf_examples if framework == "esp-idf" else result.arduino_examples
    )
    matrix = framework_matrix(framework, selected)
    return {
        "schema_version": 1,
        "framework": framework,
        "scope": {
            "changed_files": len(result.changes),
            "impact_paths": len(result.routes),
            "docs_only": result.docs_only,
            "example_build_required": bool(selected),
            "firmware_touched": result.firmware_touched,
            "release_review_required": result.release_review_required,
        },
        "selected": selected,
        "matrix": matrix,
        "unknown_paths": result.unknown_paths,
        "routes": [asdict(route) for route in result.routes],
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--framework", choices=("esp-idf", "arduino"), required=True)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--base-ref")
    source.add_argument("--changed-files-from", type=Path)
    parser.add_argument("--head-ref", default="HEAD")
    parser.add_argument("--strict-unknown", action="store_true")
    parser.add_argument("--expect-docs-only", action="store_true")
    parser.add_argument("--expect-no-example-builds", action="store_true")
    parser.add_argument("--format", choices=("text", "json"), default="json")
    args = parser.parse_args()

    try:
        changes = (
            changes_from_git(args.base_ref, args.head_ref)
            if args.base_ref is not None
            else changes_from_file(args.changed_files_from)
        )
        result = route_changes(changes)
        payload = report(result, args.framework)
    except RoutingError as error:
        print(f"Routing error: {error}", file=sys.stderr)
        return 2
    except Exception as error:  # pragma: no cover - defensive CI boundary
        print(f"Unexpected routing failure: {error}", file=sys.stderr)
        return 3

    selected = payload["selected"]
    matrix_json = json.dumps(payload["matrix"], separators=(",", ":"))
    github_output("matrix", matrix_json)
    github_output("has_examples", "true" if selected else "false")
    github_output("examples", "\n".join(selected))
    github_output("docs_only", "true" if result.docs_only else "false")
    github_output(
        "firmware_touched", "true" if result.firmware_touched else "false"
    )
    github_output(
        "release_review_required",
        "true" if result.release_review_required else "false",
    )
    unknown_paths, unknown_path_count, unknown_paths_truncated = github_path_summary(
        result.unknown_paths
    )
    github_output("unknown_paths", unknown_paths)
    github_output("unknown_path_count", str(unknown_path_count))
    github_output(
        "unknown_paths_truncated",
        "true" if unknown_paths_truncated else "false",
    )

    if args.format == "json":
        print(json.dumps(payload, indent=2))
    else:
        print(
            f"{args.framework}: selected={len(selected)} "
            f"docs_only={result.docs_only} firmware={result.firmware_touched}"
        )
        for route in result.routes:
            print(f"[{route.category}] {route.path}: {route.reason}")

    policy_failed = False
    if args.strict_unknown and result.unknown_paths:
        print("Unknown build-impact paths are not allowed in strict mode", file=sys.stderr)
        policy_failed = True
    if args.expect_docs_only and not result.docs_only:
        print("Expected a documentation-only change set", file=sys.stderr)
        policy_failed = True
    if args.expect_no_example_builds and selected:
        print("Expected no product example builds", file=sys.stderr)
        policy_failed = True
    return 1 if policy_failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
