#!/usr/bin/env python3
"""Restyle sg200x-ll device headers into the STM32 LL / CMSIS comment style.

Target style, as in the CMSIS device headers (stm32g4xx.h):

    volatile uint32_t SYS_CTRL;   /*!< System-control register, Address offset: 0x008 */
    uint32_t RESERVED_04C[6];     /*!< Reserved, Address offset: 0x04C */

Rules applied:
  1. Struct members use one trailing "/*!< description, Address offset: 0xNNN */",
     with the comment column aligned inside each struct and wrapped at the file's
     110-column limit.
  2. Unnamed padding becomes "Reserved".
  3. Leading bilingual blocks ("/// @brief <cn>" + "///        <en>") collapse to
     the English line alone.
  4. Trailing bilingual comments ("///< <cn>. <en>") keep the English part as a
     "/*!< ... */" comment; a Chinese-only gloss is dropped, since it merely
     restates the identifier.

Usage:
  restyle_cmsis.py FILE [--check]
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

COLUMN_LIMIT = 110
STRUCT_RE = re.compile(r"typedef struct\s*\n\{\n(.*?)\n\} ([A-Za-z_][A-Za-z0-9_]*);",
                       re.S)
CJK = re.compile(r"[\u4e00-\u9fff]")
# The final member of a struct is not followed by a newline, so the comment
# pattern has to accept end-of-input as a line terminator.
TRAILING = re.compile(r"[ \t]*///<[ \t]?(.*?)(?:\n|$)")
OFFSET = r"0x[0-9A-Fa-f]+"


def split_declarations(body: str) -> list[str]:
    """Split on ';' while ignoring semicolons inside comments."""
    parts, current, index, length = [], [], 0, len(body)
    while index < length:
        if body.startswith("//", index):
            end = body.find("\n", index)
            end = length if end < 0 else end
            current.append(body[index:end])
            index = end
        elif body.startswith("/*", index):
            end = body.find("*/", index + 2)
            end = length if end < 0 else end + 2
            current.append(body[index:end])
            index = end
        elif body[index] == ";":
            parts.append("".join(current))
            current = []
            index += 1
        else:
            current.append(body[index])
            index += 1
    parts.append("".join(current))
    return parts


def take_comment(text: str) -> tuple[str, str]:
    """Consume a leading comment, in either the old or the target style."""
    stripped = text.lstrip()
    if stripped.startswith("/*!<"):
        end = stripped.find("*/")
        if end >= 0:
            return stripped[5:end].strip(), stripped[end + 2:]
    pieces = []
    while True:
        match = TRAILING.match(text)
        if not match:
            break
        pieces.append(match.group(1).strip())
        text = text[match.end():]
    return " ".join(p for p in pieces if p), text


def english_only(text: str) -> str:
    """Strip a Chinese gloss and the duplicated offset prefix."""
    text = re.sub(r",?\s*Address offset: " + OFFSET + r"$", "", text.strip())
    # "偏移 0xNNN：<cn>。 Offset 0xNNN: <en>" -> "<en>"
    match = re.match(rf"^偏移 ({OFFSET})：[^。]*。\s*(?:Address )?[Oo]ffset \1:\s*(.*)$", text)
    if match:
        return match.group(2).strip()
    match = re.match(rf"^偏移 ({OFFSET})：[^。]*。$", text)
    if match:
        return ""
    match = re.match(rf"^(?:Address )?[Oo]ffset ({OFFSET}):\s*(.*)$", text)
    if match:
        return match.group(2).strip()
    match = re.match(rf"^(?:Address )?[Oo]ffset ({OFFSET})$", text)
    if match:
        return ""
    # Generic "中文。English" shape, used by the generated pad-function notes.
    tail = english_tail(text)
    return tail if tail else text.strip()


def offset_of(text: str) -> str | None:
    match = re.search(rf"(?:偏移|Address offset|Offset)\s*:?\s*({OFFSET})", text)
    return match.group(1) if match else None


def wrap(prefix: str, description: str, suffix: str, limit: int = COLUMN_LIMIT) -> list[str]:
    """Lay out `<prefix>/*!< <description><suffix> */`, wrapping when needed."""
    head = f"{prefix}/*!< "
    tail = f"{suffix} */"
    if len(head) + len(description) + len(tail) <= limit:
        return [head + description + tail]
    # When the declaration itself is long there is no sensible continuation
    # column, so the comment stays on one line and overruns the limit rather
    # than collapsing to one word per line.
    # Wrap only when the continuation column still leaves usable room.
    if len(head) > limit - 40 or not description.split():
        return [head + description + suffix + " */"]
    words = description.split()
    pad = " " * len(head)
    lines, current = [], ""
    for word in words:
        candidate = f"{current} {word}".strip()
        if len(head if not lines else pad) + len(candidate) + len(tail) > limit and current:
            lines.append(current)
            current = word
        else:
            current = candidate
    if current:
        lines.append(current)
    out = [head + lines[0]]
    out += [pad + line for line in lines[1:]]
    out[-1] += tail
    return out


def restyle_struct(body: str) -> str:
    chunks = split_declarations(body)
    members: list[tuple[str, str, str | None]] = []
    for index, chunk in enumerate(chunks[:-1]):
        # The chunk still starts with the previous member's trailing comment,
        # because the comment follows the previous ";".
        stripped = re.sub(r"/\*.*?\*/", " ", chunk, flags=re.S)
        stripped = re.sub(r"//[^\n]*", " ", stripped)
        raw_lines = [line for line in stripped.split("\n") if line.strip()]
        declaration = re.sub(r"\s+", " ", stripped).strip()
        if not declaration:
            continue
        # A declaration whose size expression is long was already laid out over
        # several lines by hand; collapsing it would overrun the column limit.
        original_shape = raw_lines if len(raw_lines) > 1 and len(declaration) > 60 else None
        comment, _ = take_comment(chunks[index + 1])
        offset = offset_of(comment)
        description = english_only(comment) if comment else ""
        if declaration.split()[-1].split("[")[0].startswith("RESERVED"):
            description = "Reserved"
        members.append((declaration, description, offset, original_shape))
    if not members:
        return body

    lines = []
    for declaration, description, offset, original_shape in members:
        # split_declarations consumes the ";", and the house style keeps a
        # single trailing sentence, so the full stop is trimmed before the
        # offset clause is appended.
        declaration += ";"
        description = description.rstrip().rstrip(".").rstrip()
        if description and offset:
            suffix = f", Address offset: {offset}"
        elif offset:
            suffix = f"Address offset: {offset}"
        else:
            suffix = ""
        padded = "    " + declaration
        # Trailing-comment alignment belongs to clang-format
        # (AlignTrailingComments); padding here would only be undone by the next
        # format run and makes the generator non-idempotent.
        prefix = padded + " "
        if original_shape:
            head = original_shape[-1].rstrip() + ";"
            lines += original_shape[:-1] + wrap(head + " ", description, suffix)
        else:
            lines += wrap(prefix, description, suffix)
    return "\n".join(lines)


def restyle_leading(text: str) -> str:
    """Collapse '/// @brief <cn>' + '///        <en>' to the English line."""
    pattern = re.compile(r"^(?P<i>[ \t]*)/// @brief (?P<cn>.*)\n"
                         r"(?:[ \t]*///[ \t]*(?P<en>[^\n]*)\n)+", re.M)

    def fix(match: re.Match) -> str:
        block = match.group(0)
        # Start after the @brief line, otherwise the Chinese line itself is
        # mistaken for an English continuation.
        body = block.split("\n", 1)[1] if "\n" in block else ""
        english = re.findall(r"[ \t]*///[ \t]*([^\n]*)\n", body)
        english = [line.strip() for line in english if line.strip()]
        chosen = next((line for line in english if not CJK.search(line)), "")
        if not chosen:
            chosen = next((line for line in english if CJK.search(line)), "")
            chosen = re.sub(r"^[\u4e00-\u9fff、，。：；（）\s]+", "", chosen) or chosen
        return f"{match.group('i')}/// @brief {chosen}\n"

    return pattern.sub(fix, text)


def restyle_trailing(text: str) -> str:
    """Rewrite standalone '///< ...' trailing comments as '/*!< ... */'."""
    pattern = re.compile(r"^(?P<body>.*?)[ \t]*///< (?P<comment>[^\n]*)\n", re.M)

    def fix(match: re.Match) -> str:
        body, comment = match.group("body"), match.group("comment")
        if not body.strip():
            return match.group(0)
        body = body.rstrip()
        english = english_only(comment)
        # With no English counterpart the gloss only restates the identifier, so
        # it is dropped rather than kept.
        return f"{body} /*!< {english} */\n" if english else f"{body}\n"

    return pattern.sub(fix, text)


# A few generated notes carry a Chinese unit word on the English side; the
# glossary equivalent keeps them readable once the Chinese is dropped.
TERMS = [(r"位\s*(\d+)", r"bit \1"), (r"偏移\s*0x", "offset 0x")]


def translate_terms(text: str) -> str:
    for pattern, replacement in TERMS:
        text = re.sub(pattern, replacement, text)
    return text


def english_tail(content: str) -> str:
    """English half of a bilingual doxygen line, '' when there is none.

    Sentence separators are tried before "/" so that "I2C/UART" inside an
    English sentence is not mistaken for the Chinese/English divider.
    """
    # A line that is already English apart from trailing Chinese punctuation.
    stripped = content.rstrip("。；、，： ")
    if stripped and not CJK.search(stripped):
        return stripped
    for separators in ("。；", "/"):
        for index in range(len(content) - 1, -1, -1):
            if content[index] in separators:
                tail = content[index + 1:].strip()
                if tail and not CJK.search(tail):
                    return _keep_ascii_tokens(content[:index], tail)
    return ""


def _keep_ascii_tokens(chinese_half: str, english: str) -> str:
    """Re-attach file names and identifiers that sat in the Chinese half.

    A line such as "sg2002-licheerv-nano-b.svd；I2C/UART 寄存器描述。I2C/UART
    register descriptions." carries a document name on the Chinese side, which
    must survive the Chinese being dropped.
    """
    kept = []
    for token in re.findall(r"[A-Za-z0-9][A-Za-z0-9_./+-]{2,}", chinese_half):
        if token not in english and token not in kept:
            kept.append(token)
    return ", ".join(kept + [english]) if kept else english


def restyle_doxygen(text: str) -> str:
    """Drop the Chinese half of bilingual doxygen lines."""
    lines = text.split("\n")
    out: list[str] = []
    index = 0
    while index < len(lines):
        line = lines[index]
        stripped = line.strip()
        # One-line "/** @brief <cn>. <en> */" blocks are handled here too.
        if stripped.startswith("/**") and stripped.endswith("*/") and CJK.search(line):
            inner = stripped[3:-2].strip()
            match = re.match(r"(@\w+)\s+(.*)$", inner)
            english = english_tail(translate_terms(match.group(2) if match else inner))
            if english:
                tag = f"{match.group(1)} " if match else ""
                indent = line[:len(line) - len(line.lstrip())]
                out.append(f"{indent}/** {tag}{english} */")
                index += 1
                continue
        is_doc = stripped.startswith(("*", "///"))
        if is_doc and CJK.search(line):
            head, _, content = line.partition(stripped[:1] == "*" and "*" or "///")
            marker = "*" if stripped.startswith("*") else "///"
            prefix, _, body = line.partition(marker)
            body = body.lstrip()
            tag = ""
            match = re.match(r"(@\w+)\s+(.*)$", body)
            if match:
                tag, body = match.group(1), match.group(2)
            english = english_tail(translate_terms(body))
            if english:
                out.append(f"{prefix}{marker} {tag + ' ' if tag else ''}{english}".rstrip())
                index += 1
                continue
            # Chinese-only line: the English usually sits on the next line, but
            # only a comment line may be consumed. Treating an arbitrary next
            # line as the translation silently commented out declarations.
            nxt = lines[index + 1] if index + 1 < len(lines) else ""
            nxt_stripped = nxt.strip()
            is_comment = nxt_stripped.startswith(("*", "///"))
            nxt_body = (nxt_stripped.lstrip("*").lstrip("/").strip()
                        if is_comment else "")
            if nxt_body and not CJK.search(nxt_body) and not nxt_body.startswith("@"):
                if tag:
                    out.append(f"{prefix}{marker} {tag} {nxt_body}".rstrip())
                    index += 2
                    continue
                if not tag:
                    # Untagged Chinese line: drop it, the English line stands.
                    index += 1
                    continue
        out.append(line)
        index += 1
    return "\n".join(out)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("file", type=Path)
    parser.add_argument("--write", action="store_true",
                        help="apply the rewrite; without it the run is a dry run")
    args = parser.parse_args()

    original = args.file.read_text(encoding="utf-8")
    # One-shot migration: the input carries bilingual '///' comments. Running it
    # again over its own CMSIS output is destructive, so refuse that case.
    if not CJK.search(original):
        print(f"{args.file}: already in CMSIS style, nothing to do")
        return 0
    restyled = STRUCT_RE.sub(lambda m: "typedef struct\n{\n" + restyle_struct(m.group(1))
                             + "\n} " + m.group(2) + ";", original)
    restyled = restyle_leading(restyled)
    restyled = restyle_trailing(restyled)
    restyled = restyle_doxygen(restyled)

    changed = sum(1 for a, b in zip(original.splitlines(), restyled.splitlines()) if a != b)
    print(f"{args.file}: {changed} lines differ, "
          f"{len(original.splitlines())} -> {len(restyled.splitlines())} lines")
    if not args.write:
        return 0
    args.file.write_text(restyled, encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
