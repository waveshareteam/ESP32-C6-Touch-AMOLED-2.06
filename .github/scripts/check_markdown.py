#!/usr/bin/env python3
"""Run the repository-specific lightweight Markdown policy gate.

This checker deliberately covers the first-party documentation maintained by
this product repository. Bundled library and embedded upstream documentation is
outside its ownership boundary.
"""

from __future__ import annotations

import argparse
import html
import json
import re
import sys
import urllib.parse
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable


DEFAULT_REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG_PATH = Path("config/markdown-audit.json")
PAIR_ROOT_FILES = ("README.md", "CONTRIBUTING.md", "SUPPORT.md", "SECURITY.md")
PAIR_EXAMPLE_FILES = (
    "examples/arduino/README.md",
    "examples/esp-idf/README.md",
    "firmware/README.md",
    "releases/README.md",
)
QUICK_LINK_ICONS = {
    "product": "🌐",
    "documentation": "📚",
    "esp_idf": "🧩",
    "arduino": "🔧",
    "firmware": "📦",
}
SUPPORTED_HOMEPAGE_COMPONENTS = {
    "centered_header",
    "html_h1",
    "subtitle",
    "hero_image",
    "badges",
    "language_switch",
    "quick_links",
    "separator",
    "h2",
}

MARKDOWN_LINK_RE = re.compile(r"!?\[([^\]]*)\]\(([^)]+)\)")
REFERENCE_LINK_RE = re.compile(r"^\s*\[[^\]]+\]:\s*(\S+)")
HTML_LINK_RE = re.compile(r"\b(?:href|src)\s*=\s*[\"']([^\"']+)[\"']", re.IGNORECASE)
HEADING_RE = re.compile(r"^(#{1,6})\s+(.+?)\s*$")
HTML_ANCHOR_RE = re.compile(
    r"\b(?:id|name)\s*=\s*[\"']([^\"']+)[\"']", re.IGNORECASE
)
EMOJI_RE = re.compile("[\U0001F000-\U0001FAFF\u2600-\u27BF]")
SENSITIVE_PATTERNS = (
    (
        "LOCAL_WINDOWS_PATH",
        re.compile(r"(?i)(?:^|[\s\"'(])(?:[a-z]:\\|\\\\)[^\s\"')]+"),
    ),
    ("LOCAL_UNIX_HOME", re.compile(r"(?:/home/|/Users/)[A-Za-z0-9._-]+/")),
    (
        "ACTUAL_SERIAL_PORT",
        re.compile(r"(?i)\bCOM[0-9]+\b|/dev/tty(?:ACM|USB|S)[0-9]+"),
    ),
    (
        "MAC_ADDRESS",
        re.compile(r"(?i)\b(?:[0-9a-f]{2}[:-]){5}[0-9a-f]{2}\b"),
    ),
    (
        "CREDENTIAL_OR_TOKEN",
        re.compile(
            r"\b(?:gh[pousr]_[A-Za-z0-9]{20,}|AKIA[0-9A-Z]{16}|"
            r"sk-[A-Za-z0-9_-]{20,})\b|-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----"
        ),
    ),
    (
        "TOOL_OR_MODEL_PROVENANCE",
        re.compile(r"(?i)\b(?:codex|chatgpt|claude|cursor|copilot)\b"),
    ),
)


@dataclass(frozen=True)
class Finding:
    code: str
    path: str
    line: int
    message: str


@dataclass(frozen=True)
class Link:
    label: str
    target: str
    line: int


class MarkdownPolicyError(RuntimeError):
    """The policy could not run to completion."""


def load_policy_config(root: Path, config_path: Path) -> dict[str, object]:
    path = config_path if config_path.is_absolute() else root / config_path
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise MarkdownPolicyError(f"Unable to read Markdown policy config: {error}") from error
    if not isinstance(payload, dict):
        raise MarkdownPolicyError("Markdown policy config must be a JSON object")
    for key in ("pair_exempt_patterns", "homepage_pairs"):
        if not isinstance(payload.get(key), list):
            raise MarkdownPolicyError(f"Markdown policy config key {key} must be a list")
    if not payload["homepage_pairs"]:
        raise MarkdownPolicyError("Markdown policy config requires at least one homepage pair")
    return payload


def configured_markdown_files(root: Path, patterns: Iterable[object]) -> list[Path]:
    files: set[Path] = set()
    for raw_pattern in patterns:
        if not isinstance(raw_pattern, str) or not raw_pattern:
            raise MarkdownPolicyError("Markdown path patterns must be non-empty strings")
        candidate = Path(raw_pattern)
        if candidate.is_absolute() or ".." in candidate.parts:
            raise MarkdownPolicyError(f"Unsafe Markdown path pattern: {raw_pattern}")
        matches = root.glob(raw_pattern) if any(char in raw_pattern for char in "*?[") else [root / candidate]
        files.update(path for path in matches if path.is_file() and path.suffix.lower() == ".md")
    return sorted(files)


def repo_relative(root: Path, path: Path) -> str:
    return path.resolve().relative_to(root.resolve()).as_posix()


def chinese_companion(path: Path) -> Path:
    return path.with_name(f"{path.stem}_ZH{path.suffix}")


def discover_english_docs(root: Path) -> list[Path]:
    paths: set[Path] = set()
    for relative in (*PAIR_ROOT_FILES, *PAIR_EXAMPLE_FILES):
        path = root / relative
        if path.is_file():
            paths.add(path)
    docs_root = root / "docs"
    if docs_root.is_dir():
        for path in docs_root.rglob("*.md"):
            if not path.stem.endswith("_ZH"):
                paths.add(path)
    return sorted(paths)


def strip_fenced_code(lines: list[str]) -> list[tuple[int, str]]:
    result: list[tuple[int, str]] = []
    fence: str | None = None
    backtick_fence = chr(96) * 3
    inline_code = re.compile(chr(96) + "[^" + chr(96) + "]*" + chr(96))
    for number, line in enumerate(lines, start=1):
        stripped = line.lstrip()
        marker = (
            backtick_fence
            if stripped.startswith(backtick_fence)
            else "~~~"
            if stripped.startswith("~~~")
            else None
        )
        if marker:
            fence = None if fence == marker else marker if fence is None else fence
            continue
        if fence is None:
            result.append((number, inline_code.sub("", line)))
    return result


def clean_destination(raw: str) -> str:
    value = raw.strip()
    if value.startswith("<") and ">" in value:
        value = value[1 : value.index(">")]
    elif re.search(r"\s+[\"']", value):
        value = re.split(r"\s+[\"']", value, maxsplit=1)[0]
    return html.unescape(value.strip())


def iter_links(text: str) -> Iterable[Link]:
    for line_number, line in strip_fenced_code(text.splitlines()):
        for match in MARKDOWN_LINK_RE.finditer(line):
            yield Link(match.group(1), clean_destination(match.group(2)), line_number)
        reference = REFERENCE_LINK_RE.match(line)
        if reference:
            yield Link("", clean_destination(reference.group(1)), line_number)
        for match in HTML_LINK_RE.finditer(line):
            yield Link("", clean_destination(match.group(1)), line_number)


def local_target(root: Path, source: Path, target: str) -> tuple[Path, str] | None:
    if not target:
        return None
    parsed = urllib.parse.urlsplit(target)
    if parsed.scheme or parsed.netloc:
        return None
    raw_path = urllib.parse.unquote(parsed.path)
    if not raw_path:
        resolved = source
    else:
        candidate = Path(raw_path)
        resolved = candidate if candidate.is_absolute() else source.parent / candidate
    resolved = resolved.resolve()
    try:
        resolved.relative_to(root.resolve())
    except ValueError as error:
        raise MarkdownPolicyError(
            f"{repo_relative(root, source)} contains a link escaping the repository"
        ) from error
    return resolved, urllib.parse.unquote(parsed.fragment)


def github_slug(value: str) -> str:
    value = re.sub(r"<[^>]+>", "", html.unescape(value))
    value = re.sub(r"[*_~]", "", value).strip().lower()
    value = "".join(
        character
        for character in value
        if character in {" ", "-", "_"} or character.isalnum()
    )
    return re.sub(r"[\s-]+", "-", value).strip("-")


def anchors(text: str) -> set[str]:
    result: set[str] = set()
    duplicates: dict[str, int] = {}
    for _line_number, line in strip_fenced_code(text.splitlines()):
        for explicit in HTML_ANCHOR_RE.findall(line):
            result.add(explicit)
        heading = HEADING_RE.match(line)
        if not heading:
            continue
        base = github_slug(heading.group(2))
        count = duplicates.get(base, 0)
        duplicates[base] = count + 1
        result.add(base if count == 0 else f"{base}-{count}")
    return result


def link_findings(root: Path, source: Path, text: str) -> list[Finding]:
    findings: list[Finding] = []
    anchor_cache: dict[Path, set[str]] = {}
    source_name = repo_relative(root, source)
    for link in iter_links(text):
        try:
            reference = local_target(root, source, link.target)
        except MarkdownPolicyError as error:
            findings.append(
                Finding("LINK_ESCAPES_REPOSITORY", source_name, link.line, str(error))
            )
            continue
        if reference is None:
            continue
        target_path, fragment = reference
        if not target_path.exists():
            findings.append(
                Finding(
                    "RELATIVE_LINK_MISSING",
                    source_name,
                    link.line,
                    f"missing local target: {link.target}",
                )
            )
            continue
        if fragment and target_path.is_file() and target_path.suffix.lower() == ".md":
            if target_path not in anchor_cache:
                anchor_cache[target_path] = anchors(
                    target_path.read_text(encoding="utf-8")
                )
            if fragment not in anchor_cache[target_path]:
                findings.append(
                    Finding(
                        "RELATIVE_LINK_FRAGMENT_MISSING",
                        source_name,
                        link.line,
                        f"missing Markdown fragment: {link.target}",
                    )
                )
    return findings


def contains_link_to(
    root: Path,
    source: Path,
    text: str,
    expected: Path,
    limit: int = 40,
) -> bool:
    for link in iter_links(text):
        if link.line > limit:
            continue
        try:
            reference = local_target(root, source, link.target)
        except MarkdownPolicyError:
            continue
        if reference and reference[0] == expected.resolve():
            return True
    return False


def pair_findings(
    root: Path,
    english_docs: list[Path],
) -> tuple[list[Finding], dict[str, str]]:
    findings: list[Finding] = []
    mapping: dict[str, str] = {}
    for english in english_docs:
        chinese = chinese_companion(english)
        english_name = repo_relative(root, english)
        chinese_name = repo_relative(root, chinese)
        mapping[english_name] = chinese_name
        if not chinese.is_file():
            findings.append(
                Finding(
                    "BILINGUAL_PAIR_MISSING",
                    english_name,
                    1,
                    f"missing Simplified Chinese companion: {chinese_name}",
                )
            )
            continue
        english_text = english.read_text(encoding="utf-8")
        chinese_text = chinese.read_text(encoding="utf-8")
        if not contains_link_to(root, english, english_text, chinese):
            findings.append(
                Finding(
                    "BILINGUAL_RECIPROCAL_LINK_MISSING",
                    english_name,
                    1,
                    f"top navigation must link to {chinese_name}",
                )
            )
        if not contains_link_to(root, chinese, chinese_text, english):
            findings.append(
                Finding(
                    "BILINGUAL_RECIPROCAL_LINK_MISSING",
                    chinese_name,
                    1,
                    f"top navigation must link to {english_name}",
                )
            )
        if len(re.findall(r"[\u3400-\u9fff]", chinese_text)) < 20:
            findings.append(
                Finding(
                    "BILINGUAL_TRANSLATION_TOO_SPARSE",
                    chinese_name,
                    1,
                    "Chinese companion does not contain enough Simplified Chinese text",
                )
            )
    return findings, mapping


def same_language_findings(
    root: Path,
    documents: list[Path],
    english_to_chinese: dict[str, str],
) -> list[Finding]:
    findings: list[Finding] = []
    chinese_to_english = {value: key for key, value in english_to_chinese.items()}
    for source in documents:
        source_name = repo_relative(root, source)
        text = source.read_text(encoding="utf-8")
        source_is_chinese = source_name in chinese_to_english
        own_counterpart = (
            chinese_to_english.get(source_name)
            if source_is_chinese
            else english_to_chinese.get(source_name)
        )
        for link in iter_links(text):
            try:
                reference = local_target(root, source, link.target)
            except MarkdownPolicyError:
                continue
            if reference is None:
                continue
            target_path, _fragment = reference
            if not target_path.exists():
                continue
            target_name = repo_relative(root, target_path)
            label = link.label.lower()
            if source_is_chinese and target_name in english_to_chinese:
                explicit_english = "english" in label or "英文" in link.label
                if target_name != own_counterpart and not explicit_english:
                    findings.append(
                        Finding(
                            "WRONG_LANGUAGE_INTERNAL_LINK",
                            source_name,
                            link.line,
                            f"Chinese page should link to {english_to_chinese[target_name]}",
                        )
                    )
            elif not source_is_chinese and target_name in chinese_to_english:
                explicit_chinese = (
                    "chinese" in label
                    or "中文" in link.label
                    or "简体" in link.label
                )
                if target_name != own_counterpart and not explicit_chinese:
                    findings.append(
                        Finding(
                            "WRONG_LANGUAGE_INTERNAL_LINK",
                            source_name,
                            link.line,
                            f"English page should link to {chinese_to_english[target_name]}",
                        )
                    )
    return findings


def heading_icon_sequence(text: str, level: int) -> list[str]:
    prefix = "#" * level + " "
    sequence: list[str] = []
    for _number, line in strip_fenced_code(text.splitlines()):
        if line.startswith(prefix) and not line.startswith(prefix + "#"):
            heading = line[len(prefix) :].strip()
            sequence.append(heading.split(maxsplit=1)[0] if heading else "")
    return sequence


def homepage_findings(root: Path, config: dict[str, object]) -> list[Finding]:
    findings: list[Finding] = []
    profiles = config["homepage_pairs"]
    assert isinstance(profiles, list)
    for raw_profile in profiles:
        if not isinstance(raw_profile, dict):
            raise MarkdownPolicyError("Each homepage pair must be a JSON object")
        try:
            english_name = str(raw_profile["english"])
            chinese_name = str(raw_profile["chinese"])
            required_components = set(raw_profile["required_components"])
            quick_roles = list(raw_profile["required_quick_links"])
            expected_h2 = list(raw_profile["required_h2_icons"])
        except (KeyError, TypeError) as error:
            raise MarkdownPolicyError(f"Invalid homepage pair: {error}") from error
        unknown_components = required_components - SUPPORTED_HOMEPAGE_COMPONENTS
        if unknown_components:
            raise MarkdownPolicyError(
                f"Unsupported homepage components: {sorted(unknown_components)}"
            )
        try:
            expected_quick_icons = [QUICK_LINK_ICONS[str(role)] for role in quick_roles]
        except KeyError as error:
            raise MarkdownPolicyError(f"Unsupported homepage quick-link role: {error}") from error
        english = root / english_name
        chinese = root / chinese_name
        if not english.is_file() or not chinese.is_file():
            continue
        english_text = english.read_text(encoding="utf-8")
        chinese_text = chinese.read_text(encoding="utf-8")
        findings.extend(
            homepage_pair_findings(
                root,
                english,
                chinese,
                english_text,
                chinese_text,
                required_components,
                expected_quick_icons,
                expected_h2,
                list(raw_profile.get("required_badges", [])),
            )
        )
    return findings


def homepage_pair_findings(
    root: Path,
    english: Path,
    chinese: Path,
    english_text: str,
    chinese_text: str,
    required_components: set[object],
    expected_quick_icons: list[str],
    expected_h2: list[object],
    required_badges: list[object],
) -> list[Finding]:
    findings: list[Finding] = []
    for path, text in ((english, english_text), (chinese, chinese_text)):
        name = repo_relative(root, path)
        header_match = re.search(
            r"<div\s+align=[\"']center[\"']>(.*?)</div>",
            text,
            re.DOTALL | re.IGNORECASE,
        )
        if not header_match and "centered_header" in required_components:
            findings.append(
                Finding(
                    "HOMEPAGE_CENTERED_HEADER_MISSING",
                    name,
                    1,
                    "missing centered HTML header",
                )
            )
            continue
        if not header_match:
            continue
        header = header_match.group(1)
        title = re.search(
            r"<h1>(.*?)</h1>", header, re.DOTALL | re.IGNORECASE
        )
        if not title and "html_h1" in required_components:
            findings.append(
                Finding(
                    "HOMEPAGE_HTML_H1_MISSING",
                    name,
                    1,
                    "missing plain HTML h1",
                )
            )
        elif title and EMOJI_RE.search(title.group(1)):
            findings.append(
                Finding(
                    "HOMEPAGE_H1_EMOJI",
                    name,
                    1,
                    "product h1 must not contain emoji",
                )
            )
        if "subtitle" in required_components and not re.search(
            r"<p><strong>.+?</strong></p>", header, re.DOTALL | re.IGNORECASE
        ):
            findings.append(
                Finding(
                    "HOMEPAGE_SUBTITLE_MISSING",
                    name,
                    1,
                    "missing product subtitle",
                )
            )
        if "hero_image" in required_components and not re.search(
            r"<p><img\b[^>]*\bwidth=[\"'][^\"']+[\"'][^>]*></p>",
            header,
            re.IGNORECASE,
        ):
            findings.append(
                Finding(
                    "HOMEPAGE_HERO_IMAGE_MISSING",
                    name,
                    1,
                    "missing centered product hero image",
                )
            )
        badge_missing = False
        if "build" in required_badges:
            badge_missing = badge_missing or "actions/workflows/" not in header or "badge.svg" not in header
        if "license" in required_badges:
            badge_missing = badge_missing or "shields.io/github/license/" not in header
        if "badges" in required_components and badge_missing:
            findings.append(
                Finding(
                    "HOMEPAGE_BADGES_MISSING",
                    name,
                    1,
                    "expected build and license badges",
                )
            )
        quick_icons = sorted(
            (icon for icon in expected_quick_icons if icon in header),
            key=header.index,
        )
        if "quick_links" in required_components and quick_icons != expected_quick_icons:
            findings.append(
                Finding(
                    "HOMEPAGE_QUICK_LINKS_MISMATCH",
                    name,
                    1,
                    f"expected quick-link icon order: {' '.join(expected_quick_icons)}",
                )
            )
        counterpart = chinese if path == english else english
        if "language_switch" in required_components and not contains_link_to(
            root, path, text, counterpart
        ):
            findings.append(
                Finding(
                    "HOMEPAGE_LANGUAGE_SWITCH_MISSING",
                    name,
                    1,
                    f"homepage header must link to {repo_relative(root, counterpart)}",
                )
            )
        if "separator" in required_components and not re.search(
            r"</div>\s*---", text, re.IGNORECASE
        ):
            findings.append(
                Finding(
                    "HOMEPAGE_SEPARATOR_MISSING",
                    name,
                    1,
                    "missing separator after centered homepage header",
                )
            )

    english_h2 = heading_icon_sequence(english_text, 2)
    chinese_h2 = heading_icon_sequence(chinese_text, 2)
    expected_h2_strings = [str(icon) for icon in expected_h2]
    if "h2" in required_components and (
        english_h2 != expected_h2_strings or chinese_h2 != expected_h2_strings
    ):
        findings.append(
            Finding(
                "HOMEPAGE_H2_SEQUENCE_MISMATCH",
                "README_ZH.md",
                1,
                "English and Chinese H2 emoji sequences must match the configured contract",
            )
        )
    for path, text in ((english, english_text), (chinese, chinese_text)):
        name = repo_relative(root, path)
        for number, line in enumerate(text.splitlines(), start=1):
            if line.startswith("### ") and EMOJI_RE.search(line[4:5]):
                findings.append(
                    Finding(
                        "HOMEPAGE_H3_EMOJI",
                        name,
                        number,
                        "tertiary headings should remain plain",
                    )
                )
    return findings


def sensitive_findings(root: Path, documents: list[Path]) -> list[Finding]:
    findings: list[Finding] = []
    for path in documents:
        name = repo_relative(root, path)
        text = path.read_text(encoding="utf-8")
        for number, line in enumerate(text.splitlines(), start=1):
            for code, pattern in SENSITIVE_PATTERNS:
                if pattern.search(line):
                    findings.append(
                        Finding(
                            code,
                            name,
                            number,
                            "public Markdown contains a disallowed environment or private-data shape",
                        )
                    )
    return findings


def run_audit(
    root: Path,
    config_path: Path = DEFAULT_CONFIG_PATH,
) -> tuple[list[Finding], dict[str, object]]:
    root = root.resolve()
    if not (root / ".git").exists():
        raise MarkdownPolicyError(f"Not a Git repository: {root}")
    config = load_policy_config(root, config_path)
    english_docs = discover_english_docs(root)
    pair_issues, mapping = pair_findings(root, english_docs)
    paired_docs = [
        path
        for english in english_docs
        for path in (english, chinese_companion(english))
        if path.is_file()
    ]
    machine_docs = configured_markdown_files(root, config["pair_exempt_patterns"])
    documents = sorted(set(paired_docs + machine_docs))
    findings = list(pair_issues)
    for document in documents:
        text = document.read_text(encoding="utf-8")
        findings.extend(link_findings(root, document, text))
    findings.extend(same_language_findings(root, documents, mapping))
    findings.extend(homepage_findings(root, config))
    findings.extend(sensitive_findings(root, documents))
    findings.sort(key=lambda item: (item.path, item.line, item.code))
    summary = {
        "schema_version": 1,
        "repository": root.name,
        "first_party_english": len(english_docs),
        "documents_checked": len(documents),
        "findings": len(findings),
    }
    return findings, summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "repository", nargs="?", type=Path, default=DEFAULT_REPO_ROOT
    )
    parser.add_argument("--all", action="store_true")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--format", choices=("text", "json"), default="text")
    args = parser.parse_args()
    if not args.all:
        print("Markdown policy requires the explicit --all scope", file=sys.stderr)
        return 2
    try:
        findings, summary = run_audit(args.repository, args.config)
    except (OSError, UnicodeError, MarkdownPolicyError) as error:
        print(f"Markdown policy error: {error}", file=sys.stderr)
        return 2
    except Exception as error:  # pragma: no cover - defensive CI boundary
        print(f"Unexpected Markdown policy failure: {error}", file=sys.stderr)
        return 3

    if args.format == "json":
        print(
            json.dumps(
                {
                    **summary,
                    "items": [asdict(finding) for finding in findings],
                },
                indent=2,
                ensure_ascii=False,
            )
        )
    else:
        print(
            f"Markdown policy: checked={summary['documents_checked']} "
            f"findings={summary['findings']}"
        )
        for finding in findings:
            print(
                f"[{finding.code}] {finding.path}:{finding.line}: "
                f"{finding.message}"
            )
    return 1 if findings else 0


if __name__ == "__main__":
    raise SystemExit(main())
