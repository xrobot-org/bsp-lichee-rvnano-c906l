"""Strip file-level and group-level Doxygen scaffolding from sg200x-ll sources.

Doxygen is kept for declarations only, so the file banner and the module
grouping machinery are removed wholesale:

* ``/** @file ... */`` banners.
* ``@defgroup`` / ``@addtoupgroup`` blocks, which also carry the ``@{`` that
  opens the group.
* ``/** @} */`` closers and bare ``@name ... @{`` sub-group openers.
* The ``@ingroup`` line inside an otherwise useful declaration comment, which
  would otherwise point at a group that no longer exists.

Comments attached to functions, variables, struct members, enums, and macros
are left untouched. The run is a dry run unless ``--write`` is given because
the previous restyling script was destructive when re-run.
"""

from __future__ import annotations

import argparse
import re
from pathlib import Path

# A block comment that starts a line. The tempered body stops at the first
# "*/" so one banner can never swallow the declaration comment that follows it.
BLOCK = re.compile(r"^[ \t]*/\*\*(?:(?!\*/).)*?\*/[ \t]*\n", re.S | re.M)
SCAFFOLD = re.compile(r"@(?:file|defgroup|addtogroup|\{|\})")
INGROUP = re.compile(r"^[ \t]*\*[ \t]*@ingroup\b[^\n]*\n", re.M)
# "///" style grouping: a "@name" caption and the prose under it run up to the
# "@{" that opens the group, so the whole span is group documentation.
NAME_GROUP = re.compile(r"^[ \t]*///[ \t]*@name\b[^\n]*\n"
                        r"(?:[ \t]*///[^\n]*\n)*?"
                        r"[ \t]*///[ \t]*@\{[^\n]*\n", re.M)
BRACE = re.compile(r"^[ \t]*///[ \t]*@[{}][^\n]*\n", re.M)


def strip(text: str) -> tuple[str, int, int]:
    """Return the text with scaffolding removed plus both removal counts."""
    out: list[str] = []
    position = 0
    dropped = 0
    for match in BLOCK.finditer(text):
        block = match.group(0)
        out.append(text[position:match.start()])
        position = match.end()
        if SCAFFOLD.search(block):
            dropped += 1
        elif INGROUP.search(block):
            out.append(INGROUP.sub("", block))
            dropped += 1
        else:
            out.append(block)
    out.append(text[position:])
    joined = "".join(out)
    joined, named = NAME_GROUP.subn("", joined)
    joined, braced = BRACE.subn("", joined)
    return tidy(joined), dropped + named + braced


def tidy(text: str) -> str:
    """Collapse the blank-line runs left behind and drop leading blanks."""
    text = re.sub(r"\n{3,}", "\n\n", text)
    return text.lstrip("\n")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("files", nargs="+", type=Path)
    parser.add_argument("--write", action="store_true",
                        help="apply the rewrite; without it the run is a dry run")
    args = parser.parse_args()

    for path in args.files:
        original = path.read_text(encoding="utf-8")
        stripped, dropped = strip(original)
        left = len(re.findall(r"@(?:file|defgroup|addtogroup|ingroup|name)\b|@[{}]", stripped))
        print(f"{path}: 移除 {dropped} 块, 残留指令 {left}")
        if args.write:
            path.write_text(stripped, encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
