#!/usr/bin/env python3
"""Check that the exported application and documentation contain no CJK copy."""

from __future__ import annotations

import argparse
from pathlib import Path
import re
import sys

from export_github_source import selected_files


CJK_COPY = re.compile(r"[\u3400-\u4dbf\u4e00-\u9fff\uf900-\ufaff\U00020000-\U0002fa1f]")


def non_english_copy(paths: list[Path], root: Path) -> list[str]:
    findings: list[str] = []
    for path in paths:
        relative = path.relative_to(root).as_posix()
        if CJK_COPY.search(relative):
            findings.append(f"{relative}: non-English filename")
        try:
            content = path.read_text(encoding="utf-8")
        except UnicodeDecodeError:
            # Images, URCaps, and other binary assets retain their original bytes.
            continue
        for number, line in enumerate(content.splitlines(), start=1):
            if CJK_COPY.search(line):
                findings.append(f"{relative}:{number}: {line.strip()[:160]}")
    return findings


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project-root", type=Path, default=Path(__file__).resolve().parents[1])
    arguments = parser.parse_args(argv)
    root = arguments.project_root.resolve()
    paths, _ = selected_files(root)
    findings = non_english_copy(paths, root)
    if findings:
        print("English release check failed:", file=sys.stderr)
        print("\n".join(findings), file=sys.stderr)
        return 1
    print(f"English release check passed: {len(paths)} exported files; no CJK copy or filenames.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
