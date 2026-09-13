#!/usr/bin/env python3
"""Collapse short bilingual comments onto one "中文 / English" line.

sg200x-ll headers carry paired comments:

    * @brief 复位一个已停止的 I2C 实例。
    *        Reset one stopped I2C instance.

For short pairs the two lines are unnecessary and read worse than one line:

    * @brief 复位一个已停止的 I2C 实例 / Reset one stopped I2C instance.

Only pairs that fit inside the column limit are joined; a pair whose English
half is itself wrapped, or whose joined form would overrun, keeps the two-line
layout.

Usage:
  collapse_bilingual.py FILE... [--check]
"""
from __future__ import annotations

import argparse
import re
import sys
import unicodedata
from pathlib import Path

CJK = re.compile(r"[\u4e00-\u9fff]")
COLUMN_LIMIT = 110


def display_width(line: str) -> int:
    """Columns a line occupies; CJK glyphs are double width.

    Counting characters would let a mostly-Chinese merged line pass the check
    while rendering well past the column limit.
    """
    return sum(2 if unicodedata.east_asian_width(char) in "WF" else 1 for char in line)
# Trailing Chinese sentence punctuation is dropped before the separator.
TRAILING = "。；、，："
MARKER = re.compile(r"^(?P<prefix>\s*(?:\*|///<?)\s*)(?P<body>.*)$")


def split_marker(line: str) -> tuple[str, str] | None:
    match = MARKER.match(line)
    if not match:
        return None
    return match.group("prefix"), match.group("body")


def is_english_continuation(line: str) -> str | None:
    """Body of a comment line that carries English and no Chinese."""
    parts = split_marker(line)
    if parts is None:
        return None
    body = parts[1].strip()
    if not body or CJK.search(body) or body.startswith("@"):
        return None
    # "*/" and "/**" parse as marker + "/" or "*"; a translation always carries
    # letters or digits. Without this the comment terminator is swallowed.
    if not re.search(r"[A-Za-z0-9]", body):
        return None
    return body


def same_line_split(body: str) -> tuple[str, str] | None:
    """Split "中文。English" only when the English half carries no Chinese.

    Splitting on the first "；" would cut a Chinese sentence in half whenever a
    digit follows it ("...的偶数；0 表示..."), so the tail is required to be
    free of Chinese and the search runs from the right.
    """
    for index in range(len(body) - 1, -1, -1):
        if body[index] in "。；":
            tail = body[index + 1:].strip()
            if tail and not CJK.search(tail):
                return body[:index].rstrip(TRAILING), tail
    return None


def wrap_english(indent: str, text: str) -> list[str]:
    """Greedy word wrap of an English paragraph under the column limit.

    Greedy wrapping is idempotent, so re-running the script leaves an already
    wrapped paragraph untouched.
    """
    lines: list[str] = []
    current = ""
    for word in text.split():
        candidate = f"{current} {word}".strip()
        if current and display_width(indent + candidate) > COLUMN_LIMIT:
            lines.append(indent + current)
            current = word
        else:
            current = candidate
    if current:
        lines.append(indent + current)
    return lines


def collapse(lines: list[str]) -> tuple[list[str], int]:
    out: list[str] = []
    joined = 0
    index = 0
    while index < len(lines):
        line = lines[index]
        parts = split_marker(line)
        if parts and CJK.search(line) and index + 1 < len(lines):
            prefix, body = parts
            english = is_english_continuation(lines[index + 1])
            # A translation is a paragraph: keep consuming following English
            # comment lines until one carries Chinese, opens a new tag, or closes
            # the block. Hand-wrapped paragraphs mix indents, so the whole run is
            # re-wrapped at the indent of its first line.
            end = index + 1
            if english is not None:
                while end < len(lines):
                    deeper = split_marker(lines[end])
                    if deeper is None or len(deeper[0]) < len(prefix):
                        break
                    if is_english_continuation(lines[end]) is None:
                        break
                    end += 1
            if english is not None:
                paragraph = " ".join(
                    is_english_continuation(part) for part in lines[index + 1:end])
                candidate = f"{prefix}{body.rstrip().rstrip(TRAILING)} / {paragraph}"
                if display_width(candidate) <= COLUMN_LIMIT:
                    out.append(candidate)
                    joined += 1
                    index = end
                    continue
                # Too long to merge, so the Chinese keeps its own line and the
                # translation is re-wrapped underneath it. No "/" is used here:
                # the fixed Chinese-then-English order carries the meaning.
                indent = split_marker(lines[index + 1])[0]
                out.append(line)
                out.extend(wrap_english(indent, paragraph))
                index = end
                continue
        # Same-line "中文。English" pairs become "中文 / English".
        if parts and CJK.search(line):
            prefix, body = parts
            split = same_line_split(body)
            if split:
                candidate = f"{prefix}{split[0]} / {split[1]}"
                if display_width(candidate) <= COLUMN_LIMIT:
                    out.append(candidate)
                    joined += 1
                    index += 1
                    continue
        out.append(line)
        index += 1
    return out, joined


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("files", nargs="+", type=Path)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()

    total = 0
    for path in args.files:
        lines = path.read_text(encoding="utf-8").split("\n")
        result, joined = collapse(lines)
        total += joined
        print(f"{path}: {joined} 处合并")
        if not args.check and joined:
            path.write_text("\n".join(result), encoding="utf-8")
    print(f"合计合并 {total} 处")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
