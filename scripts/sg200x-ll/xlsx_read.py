#!/usr/bin/env python3
"""Minimal XLSX reader.

The SG2002 PINOUT workbook is the only published source for the pin-control
registers, and it is a plain OOXML package, so a small reader avoids depending
on openpyxl just to look at it.

Usage:
  xlsx_read.py FILE --sheets                 # list sheets
  xlsx_read.py FILE --sheet 3 --rows 1 40    # print a cell range
"""
from __future__ import annotations

import argparse
import re
import sys
import xml.etree.ElementTree as ET
import zipfile
from pathlib import Path

NS = "{http://schemas.openxmlformats.org/spreadsheetml/2006/main}"
REL_NS = "{http://schemas.openxmlformats.org/officeDocument/2006/relationships}"
PKG_REL_NS = "{http://schemas.openxmlformats.org/package/2006/relationships}"

CELL_RE = re.compile(r"([A-Z]+)(\d+)")


def column_index(reference: str) -> int:
    """A1 -> 0, B1 -> 1, AA1 -> 26."""
    letters = CELL_RE.match(reference).group(1)
    index = 0
    for letter in letters:
        index = index * 26 + (ord(letter) - ord("A") + 1)
    return index - 1


def load_shared_strings(package: zipfile.ZipFile) -> list[str]:
    try:
        data = package.read("xl/sharedStrings.xml")
    except KeyError:
        return []
    root = ET.fromstring(data)
    strings = []
    for item in root.findall(f"{NS}si"):
        strings.append("".join(node.text or "" for node in item.iter(f"{NS}t")))
    return strings


def sheet_targets(package: zipfile.ZipFile) -> list[tuple[str, str]]:
    workbook = ET.fromstring(package.read("xl/workbook.xml"))
    rels = ET.fromstring(package.read("xl/_rels/workbook.xml.rels"))
    targets = {rel.get("Id"): rel.get("Target")
               for rel in rels.findall(f"{PKG_REL_NS}Relationship")}
    result = []
    for sheet in workbook.find(f"{NS}sheets").findall(f"{NS}sheet"):
        target = targets.get(sheet.get(f"{REL_NS}id"), "")
        if not target.startswith("xl/"):
            target = "xl/" + target.lstrip("/")
        result.append((sheet.get("name"), target))
    return result


def read_sheet(package: zipfile.ZipFile, target: str,
               strings: list[str]) -> list[list[str]]:
    root = ET.fromstring(package.read(target))
    rows: list[list[str]] = []
    for row in root.iter(f"{NS}row"):
        cells: list[str] = []
        for cell in row.findall(f"{NS}c"):
            index = column_index(cell.get("r", "A1"))
            while len(cells) < index:
                cells.append("")
            kind = cell.get("t")
            if kind == "inlineStr":
                value = "".join(n.text or "" for n in cell.iter(f"{NS}t"))
            else:
                node = cell.find(f"{NS}v")
                value = node.text if node is not None and node.text else ""
                if kind == "s" and value:
                    value = strings[int(value)]
            cells.append(value.strip())
        rows.append(cells)
    return rows


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("file", type=Path)
    parser.add_argument("--sheets", action="store_true")
    parser.add_argument("--sheet", help="sheet number (1-based) or name substring")
    parser.add_argument("--rows", nargs=2, type=int, default=[1, 40],
                        metavar=("FROM", "TO"))
    parser.add_argument("--cols", type=int, default=12)
    args = parser.parse_args()

    with zipfile.ZipFile(args.file) as package:
        sheets = sheet_targets(package)
        if args.sheets or not args.sheet:
            for number, (name, _) in enumerate(sheets, 1):
                print(f"{number}. {name}")
            return 0
        match = None
        if args.sheet.isdigit():
            match = sheets[int(args.sheet) - 1]
        else:
            match = next((s for s in sheets if args.sheet in s[0]), None)
        if match is None:
            parser.error(f"no sheet matches {args.sheet!r}")
        print(f"### {match[0]}")
        rows = read_sheet(package, match[1], load_shared_strings(package))
        first, last = args.rows
        for number, row in enumerate(rows[first - 1:last], first):
            cells = row[:args.cols]
            if any(cells):
                print(f"{number:>4} | " + " | ".join(cells))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
