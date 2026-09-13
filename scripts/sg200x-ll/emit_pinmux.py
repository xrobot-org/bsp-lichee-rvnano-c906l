#!/usr/bin/env python3
"""Emit the PINMUX pad-selector offsets from the vendor SDK header.

TRM v1.02 chapter 10 is a stub pointing at the online PINOUT workbook, so the
SDK's generated `cv181x_reg_fmux_gpio.h` is the authoritative map for this
block. It defines one 3-bit function-selector register per pad:

  FMUX_GPIO_REG_IOCTRL_<PAD>    <byte offset>   register offset in the block
  FMUX_GPIO_FUNCSEL_<PAD>       <byte offset>   same value, named for the field
  FMUX_GPIO_FUNCSEL_<PAD>_OFFSET 0              field position inside the word
  FMUX_GPIO_FUNCSEL_<PAD>_MASK   0x7            field width

sg200x-ll already models the block as `volatile uint32_t FUNCTION[117]` at
PINMUX_BASE, so only the per-pad offsets are missing.

Usage:
  emit_pinmux.py SDK_HEADER
"""
from __future__ import annotations

import argparse
import collections
import re
import sys
import zipfile
from pathlib import Path

import xlsx_read as xr

PREFIX = "FMUX_GPIO_REG_IOCTRL_"


def read_pads(header: Path) -> dict[str, int]:
    text = header.read_text(encoding="utf-8", errors="ignore")
    pads: dict[str, int] = {}
    for name, value in re.findall(r"^#define\s+(\w+)\s+(\S+)", text, re.M):
        if name.startswith(PREFIX):
            pads[name[len(PREFIX):]] = int(value, 0)
    return pads


FUNCTION_SHEET = "2. "


def read_functions(workbook: Path) -> dict[str, dict[int, str]]:
    """Per-pad function encodings from PINOUT sheet 2."""
    with zipfile.ZipFile(workbook) as package:
        target = next(t for n, t in xr.sheet_targets(package)
                      if n.startswith(FUNCTION_SHEET))
        rows = xr.read_sheet(package, target, xr.load_shared_strings(package))
    pads: dict[str, dict[int, str]] = collections.defaultdict(dict)
    for row in rows[1:]:
        row = row + [""] * (7 - len(row))
        _, signal, _, _, pin, number = row[:6]
        if pin and number.strip().isdigit():
            pads[pin][int(number)] = signal.strip()
    return pads


def identifier(signal: str) -> str:
    """SDIO0_CLK unchanged, XGPIOA[7] -> XGPIOA7, PWR_GPIO[18] -> PWR_GPIO18.

    Brackets and parentheses are dropped rather than turned into separators so
    indexed signals read naturally; underscores in the TRM name are preserved.
    """
    stripped = re.sub(r"[\[\]()]", "", signal)
    return re.sub(r"[^A-Za-z0-9_]+", "_", stripped).strip("_").upper() or "RESERVED"


def emit_functions(pads: dict[str, dict[int, str]]) -> str:
    out = ["/// @name 引脚功能编码 / Pin function encodings",
           "///",
           "/// 每个焊盘的功能选择字段取值到信号的对应关系，取自 SG2002 PINOUT 表 2。",
           "/// Per-pad function-selector encodings, from SG2002 PINOUT sheet 2.",
           "/// @{"]
    for pad in sorted(pads):
        out += ["", f"/// @brief {pad} 的功能编码。",
                f"///        Function encodings for {pad}."]
        for number in sorted(pads[pad]):
            signal = pads[pad][number]
            out.append(f"static constexpr auto PINMUX_{pad}_{identifier(signal)}_FUNCTION "
                       f"= {number}U; ///< {pad} 功能 {number}：{signal}。"
                       f" Function {number} of {pad} selects {signal}.")
    out += ["", "/// @}", ""]
    out.append("")
    return "\n".join(out)


def emit(pads: dict[str, int]) -> str:
    out = ["/// @name 全部引脚功能选择偏移（按寄存器偏移排序）/ All pad selectors, in offset order",
           "/// @{"]
    for pad, offset in sorted(pads.items(), key=lambda kv: (kv[1], kv[0])):
        out += [f"/// @brief {pad} 引脚功能选择偏移。",
                f"///        Function-selector offset of {pad}.",
                f"static constexpr auto PINMUX_{pad}_OFFSET = 0x{offset:03X}UL;"]
    out += ["/// @}", ""]
    return "\n".join(out)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("header", type=Path)
    parser.add_argument("--verify", type=Path,
                        help="existing sg2002.h, checked for disagreements")
    parser.add_argument("--functions", type=Path,
                        help="PINOUT workbook; also emit function encodings")
    args = parser.parse_args()
    if not args.header.is_file():
        parser.error(f"header not found: {args.header}")

    pads = read_pads(args.header)
    if len(pads) < 100:
        parser.error(f"only {len(pads)} pads parsed; wrong header?")

    if args.verify:
        text = args.verify.read_text(encoding="utf-8")
        known = {m.group(1): int(m.group(2), 16) for m in re.finditer(
            r"PINMUX_([A-Z0-9_]+)_OFFSET = (0x[0-9A-Fa-f]+)UL", text)}
        bad = [(p, o, known[p]) for p, o in pads.items()
               if p in known and known[p] != o]
        print(f"# {len(pads)} pads parsed, {len(known)} already present, "
              f"{len(bad)} disagree", file=sys.stderr)
        for pad, expected, found in bad:
            print(f"#   {pad}: SDK 0x{expected:03X} vs sg2002.h 0x{found:03X}",
                  file=sys.stderr)

    print(emit(pads))
    if args.functions:
        print(emit_functions(read_functions(args.functions)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
