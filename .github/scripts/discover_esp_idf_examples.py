#!/usr/bin/env python3
"""Discover first-party ESP-IDF examples that should be built by CI."""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
TOOLCHAINS = json.loads((REPO_ROOT / "config/toolchains.json").read_text(encoding="utf-8"))

EXAMPLE_ROOT = Path("examples/esp-idf")
DEFAULT_IDF_VERSIONS = tuple(TOOLCHAINS["esp_idf_versions"])
TARGET = str(TOOLCHAINS["target"])
def is_project(path: Path) -> bool:
    return (path / "CMakeLists.txt").is_file() and (path / "main").is_dir()


def list_examples(root: Path = EXAMPLE_ROOT) -> list[str]:
    if not root.is_dir():
        return []
    examples = [path.as_posix() for path in root.iterdir() if path.is_dir() and is_project(path)]
    return sorted(examples)


def slugify(value: str) -> str:
    slug = re.sub(r"[^a-z0-9]+", "-", value.lower()).strip("-")
    return slug or "example"


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
                "example": example,
                "name": slugify(Path(example).name),
                "idf_version": idf_version,
                "target": TARGET,
            }
            for example in selected
            for idf_version in DEFAULT_IDF_VERSIONS
        ]
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--example",
        default="all",
        help="Manual project directory name, repository-relative path, or all.",
    )
    args = parser.parse_args()

    known_examples = set(list_examples())
    requested_example = normalize_example(args.example, known_examples)

    if not known_examples:
        print(f"No ESP-IDF projects found under {EXAMPLE_ROOT}", file=sys.stderr)
        return 1

    if requested_example in {"", "all"}:
        selected = sorted(known_examples)
    elif requested_example:
        if requested_example not in known_examples:
            print(f"Unknown ESP-IDF example: {args.example}", file=sys.stderr)
            print("Known examples:", file=sys.stderr)
            for example in sorted(known_examples):
                print(f"  {example}", file=sys.stderr)
            return 1
        selected = [requested_example]
    matrix_json = json.dumps(build_matrix(selected), separators=(",", ":"))
    github_output("matrix", matrix_json)
    github_output("has_examples", "true" if selected else "false")
    github_output("examples", ",".join(selected))
    print(matrix_json)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
