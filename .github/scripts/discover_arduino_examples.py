#!/usr/bin/env python3
"""Discover first-party Arduino sketches that should be built by CI."""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
TOOLCHAINS = json.loads((REPO_ROOT / "config/toolchains.json").read_text(encoding="utf-8"))

EXAMPLE_ROOT = Path("examples/arduino")
ARDUINO_CORE_VERSION = str(TOOLCHAINS["arduino_core_version"])
ARDUINO_FQBN = str(TOOLCHAINS["arduino_fqbn"])
BOARD_MANAGER_URL = str(TOOLCHAINS["arduino_board_manager_url"])
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
    parser.add_argument(
        "--example",
        default="all",
        help="Manual sketch directory name, repository-relative path, or all.",
    )
    args = parser.parse_args()

    known_examples = set(list_examples())
    requested_example = normalize_example(args.example, known_examples)

    if not known_examples:
        print(f"No first-party Arduino sketches found under {EXAMPLE_ROOT}", file=sys.stderr)
        return 1

    if requested_example in {"", "all"}:
        selected = sorted(known_examples)
    elif requested_example:
        if requested_example not in known_examples:
            print(f"Unknown Arduino sketch: {args.example}", file=sys.stderr)
            print("Known sketches:", file=sys.stderr)
            for example in sorted(known_examples):
                print(f"  {example}", file=sys.stderr)
            return 1
        selected = [requested_example]
    matrix_json = json.dumps(build_matrix(selected), separators=(",", ":"))
    github_output("matrix", matrix_json)
    github_output("has_examples", "true" if selected else "false")
    github_output("examples", ",".join(selected))
    github_output("docs_only", "false")
    github_output("firmware_touched", "false")
    github_output("release_review_required", "false")
    github_output("unknown_paths", "")
    github_output("unknown_path_count", "0")
    github_output("unknown_paths_truncated", "false")
    print(matrix_json)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
