#!/usr/bin/env python3
"""Check local Markdown links and fenced-code balance without dependencies."""

from __future__ import annotations

import re
import sys
from pathlib import Path
from urllib.parse import unquote


REPOSITORY_ROOT = Path(__file__).resolve().parent.parent
LINK_PATTERN = re.compile(r"\[[^\]]*\]\(([^)]+)\)")
EXTERNAL_PREFIXES = ("http://", "https://", "mailto:")


def markdown_files() -> list[Path]:
    files = [
        REPOSITORY_ROOT / "README.md",
        REPOSITORY_ROOT / "CONTRIBUTING.md",
        REPOSITORY_ROOT / "SECURITY.md",
    ]
    files.extend(sorted((REPOSITORY_ROOT / "docs").glob("**/*.md")))
    files.extend(sorted((REPOSITORY_ROOT / "paper").glob("**/*.md")))
    return [path for path in files if path.is_file()]


def main() -> int:
    failures: list[str] = []
    checked_links = 0

    for document in markdown_files():
        contents = document.read_text(encoding="utf-8")
        fences = sum(1 for line in contents.splitlines() if line.lstrip().startswith("```"))
        if fences % 2:
            failures.append(f"{document.relative_to(REPOSITORY_ROOT)}: unbalanced code fence")

        for match in LINK_PATTERN.finditer(contents):
            destination = match.group(1).strip().strip("<>")
            if not destination or destination.startswith("#") or destination.startswith(EXTERNAL_PREFIXES):
                continue
            local_part = unquote(destination.split("#", 1)[0])
            target = (document.parent / local_part).resolve()
            checked_links += 1
            try:
                target.relative_to(REPOSITORY_ROOT)
            except ValueError:
                failures.append(
                    f"{document.relative_to(REPOSITORY_ROOT)}: link escapes repository: {destination}"
                )
                continue
            if not target.exists():
                failures.append(
                    f"{document.relative_to(REPOSITORY_ROOT)}: missing link target: {destination}"
                )

    if failures:
        print("Documentation checks failed:", file=sys.stderr)
        for failure in failures:
            print(f"- {failure}", file=sys.stderr)
        return 1

    print(f"Documentation checks passed ({len(markdown_files())} files, {checked_links} local links).")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
