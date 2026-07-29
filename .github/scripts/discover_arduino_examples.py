#!/usr/bin/env python3
"""Discover first-party Arduino sketches that should be built by CI."""

from __future__ import annotations

import argparse
import fnmatch
import json
import os
import re
import subprocess
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
TOOLCHAINS = json.loads((REPO_ROOT / "config/toolchains.json").read_text(encoding="utf-8"))

EXAMPLE_ROOT = Path("examples/arduino")
ARDUINO_CORE_VERSION = str(TOOLCHAINS["arduino_core_version"])
ARDUINO_FQBN = str(TOOLCHAINS["arduino_fqbn"])
BOARD_MANAGER_URL = str(TOOLCHAINS["arduino_board_manager_url"])
GLOBAL_PATTERNS = (
    ".github/workflows/arduino-examples.yml",
    ".github/scripts/discover_arduino_examples.py",
    ".github/scripts/test_ci_tools.py",
    "config/toolchains.json",
    "examples/arduino/libraries/**",
    "releases/package_firmware.py",
)


def run_git(args: list[str]) -> list[str]:
    result = subprocess.run(
        ["git", *args],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    )
    return [line.strip() for line in result.stdout.splitlines() if line.strip()]


def is_sketch(path: Path) -> bool:
    return path.is_dir() and (path / f"{path.name}.ino").is_file()


def list_examples(root: Path = EXAMPLE_ROOT) -> list[str]:
    if not root.is_dir():
        return []
    return sorted(path.as_posix() for path in root.iterdir() if is_sketch(path))


def slugify(value: str) -> str:
    slug = re.sub(r"[^a-z0-9]+", "-", value.lower()).strip("-")
    return slug or "sketch"


def normalize_example(value: str, known_examples: set[str]) -> str:
    value = value.strip().strip("/")
    if not value or value == "all":
        return value

    normalized = Path(value).as_posix()
    if normalized in known_examples:
        return normalized

    matches = [example for example in known_examples if Path(example).name == value]
    if len(matches) == 1:
        return matches[0]
    return normalized


def discover_from_paths(paths: list[str], known_examples: set[str]) -> list[str]:
    selected: set[str] = set()
    root_path = EXAMPLE_ROOT.as_posix()

    for changed_path in paths:
        changed_path = changed_path.strip().strip("/")
        if any(fnmatch.fnmatch(changed_path, pattern) for pattern in GLOBAL_PATTERNS):
            selected.update(known_examples)
            continue

        for example in known_examples:
            if changed_path == example or changed_path.startswith(example + "/"):
                selected.add(example)
                break
        else:
            if changed_path == root_path or changed_path.startswith(root_path + "/"):
                selected.update(known_examples)

    return sorted(selected)


def discover_changed_examples(
    base_ref: str | None,
    head_ref: str,
    known_examples: set[str],
) -> list[str]:
    if base_ref:
        diff_args = ["diff", "--name-only", f"{base_ref}...{head_ref}"]
    else:
        diff_args = ["diff-tree", "--no-commit-id", "--name-only", "-r", head_ref]
    return discover_from_paths(run_git(diff_args), known_examples)


def github_output(name: str, value: str) -> None:
    output_path = os.environ.get("GITHUB_OUTPUT")
    if output_path:
        with open(output_path, "a", encoding="utf-8") as output:
            output.write(f"{name}={value}\n")


def build_matrix(selected: list[str]) -> dict[str, list[dict[str, str]]]:
    return {
        "include": [
            {
                "sketch": sketch,
                "name": slugify(Path(sketch).name),
                "arduino_core_version": ARDUINO_CORE_VERSION,
                "fqbn": ARDUINO_FQBN,
                "board_manager_url": BOARD_MANAGER_URL,
            }
            for sketch in selected
        ]
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-ref")
    parser.add_argument("--head-ref", default="HEAD")
    parser.add_argument("--example", default="")
    parser.add_argument(
        "--fallback-all",
        action="store_true",
        help="Build all sketches when no changed first-party sketch is detected.",
    )
    args = parser.parse_args()

    known_examples = set(list_examples())
    requested_example = normalize_example(args.example, known_examples)

    if not known_examples:
        print(f"No first-party Arduino sketches found under {EXAMPLE_ROOT}", file=sys.stderr)
        return 1

    if requested_example == "all":
        selected = sorted(known_examples)
    elif requested_example:
        if requested_example not in known_examples:
            print(f"Unknown Arduino sketch: {args.example}", file=sys.stderr)
            print("Known sketches:", file=sys.stderr)
            for example in sorted(known_examples):
                print(f"  {example}", file=sys.stderr)
            return 1
        selected = [requested_example]
    else:
        selected = discover_changed_examples(args.base_ref, args.head_ref, known_examples)
        if args.fallback_all and not selected:
            selected = sorted(known_examples)

    matrix_json = json.dumps(build_matrix(selected), separators=(",", ":"))
    github_output("matrix", matrix_json)
    github_output("has_examples", "true" if selected else "false")
    github_output("examples", ",".join(selected))
    print(matrix_json)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
