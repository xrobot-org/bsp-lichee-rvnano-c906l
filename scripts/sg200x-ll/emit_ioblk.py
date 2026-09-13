#!/usr/bin/env python3
"""Emit IOBLK / RTC_IO pad-control definitions from the SG2002 PINOUT workbook.

TRM v1.02 chapter 10 is a stub that says only:

    For information on pin multiplexing and pin control, please refer directly
    to the online table: .../04_SG2002_PINOUT.xlsx

so that workbook is the published source for the pad-control registers. Sheet
"3. 管脚控制寄存器" lists one 32-bit register per pad, giving its address and the
bit of every field. The field layout is one of exactly three variants:

  A  34 pads  PU2 PD3 DS0:5 DS1:6 ST0:8 ST1:9 HE:10 SL:11
  B  28 pads  PU2 PD3 DS0:5 DS1:6 DS2:7 ST0:8 SL:11
  C   1 pad   XDS0:14 XDS1:15 XDS2:16

Usage:
  emit_ioblk.py WORKBOOK [--group G1|GRTC]
"""
from __future__ import annotations

import argparse
import collections
import sys
import zipfile
from dataclasses import dataclass
from pathlib import Path

import xlsx_read as xr

SHEET = "管脚控制寄存器"

# Field bit positions shared by every pad, then the variant-specific ones.
COMMON_FIELDS = [("PU", 2, "上拉电阻使能", "pull-up enable"),
                 ("PD", 3, "下拉电阻使能", "pull-down enable"),
                 ("DS0", 5, "输出驱动能力档位 bit 0", "drive strength bit 0"),
                 ("DS1", 6, "输出驱动能力档位 bit 1", "drive strength bit 1"),
                 ("ST0", 8, "输入施密特触发器强度 bit 0", "Schmitt trigger level bit 0"),
                 ("SL", 11, "输出压摆率限制", "output slew-rate limit")]
LAYOUT_A = [("ST1", 9, "输入施密特触发器强度 bit 1", "Schmitt trigger level bit 1"),
            ("HE", 10, "弱电平维持器使能", "bus-holder enable")]
LAYOUT_B = [("DS2", 7, "输出驱动能力档位 bit 2", "drive strength bit 2")]
LAYOUT_C = [("XDS0", 14, "扩展驱动能力档位 bit 0", "extended drive strength bit 0"),
            ("XDS1", 15, "扩展驱动能力档位 bit 1", "extended drive strength bit 1"),
            ("XDS2", 16, "扩展驱动能力档位 bit 2", "extended drive strength bit 2")]


@dataclass
class Pad:
    offset: int
    register: str
    member: str
    pin: str
    layout: str


def read_pads(workbook: Path) -> tuple[list[Pad], list[Pad]]:
    with zipfile.ZipFile(workbook) as package:
        target = next(t for n, t in xr.sheet_targets(package) if SHEET in n)
        rows = xr.read_sheet(package, target, xr.load_shared_strings(package))

    collected: dict[str, dict] = collections.OrderedDict()
    for row in rows[1:]:
        row = row + [""] * (9 - len(row))
        pin, name, register, address, field, bit = row[:6]
        if register and address:
            collected[register] = {"address": int(address.replace("_", ""), 16),
                                   "pin": pin if pin not in ("#N/A", "") else name,
                                   "name": name, "fields": []}
        if field and bit and collected:
            collected[list(collected)[-1]]["fields"].append(
                (field.rsplit("_", 1)[-1], int(bit)))

    def layout_of(fields) -> str:
        keys = {f for f, _ in fields}
        if "XDS0" in keys:
            return "C"
        return "A" if "ST1" in keys else "B"

    active, rtc = [], []
    for register, info in collected.items():
        member = register.replace("IOBLK_G1_REG_", "").replace("IOBLK_GRTC_REG_", "")
        pad = Pad(0, register, member, info["pin"], layout_of(info["fields"]))
        (rtc if info["address"] >= 0x05000000 else active).append((info["address"], pad))

    def offsets(items, base):
        return [Pad(address - base, p.register, p.member, p.pin, p.layout)
                for address, p in sorted(items)]

    return offsets(active, 0x03001800), offsets(rtc, 0x05027000)


def emit_struct(pads: list[Pad], type_name: str, label: str, macro: str,
                base: str, see: str) -> str:
    out = ["/**", f" * @defgroup {macro} {label} 焊盘电气控制 / {label} pad electrical control",
           " * @ingroup SG2002_DEVICE", f" * @see {see}", " * @{", " */", ""]
    out += ["/**",
            f" * @brief {label} 各焊盘的上下拉、驱动能力与施密特触发配置。",
            f" *        Per-pad pull, drive-strength and Schmitt-trigger configuration.",
            " */",
            "typedef struct", "{"]
    previous = 0
    for pad in pads:
        if pad.offset > previous:
            words = (pad.offset - previous) // 4
            if words:
                out.append(f"    uint32_t RESERVED_{previous:03X}[{words}];"
                           f" ///< 偏移 0x{previous:03X}：未公开区域的占位空间。"
                           f" Offset 0x{previous:03X}: unexposed padding.")
        out.append(f"    volatile uint32_t {pad.member};"
                   f" ///< 偏移 0x{pad.offset:03X}：{pad.pin} 焊盘控制，字段布局 {pad.layout}。"
                   f" Offset 0x{pad.offset:03X}: {pad.pin} pad control, field layout {pad.layout}.")
        previous = pad.offset + 4
    out += [f"}} {type_name};", ""]
    for pad in pads:
        out.append(f"static_assert(offsetof({type_name}, {pad.member}) == 0x{pad.offset:03X}U);")
    out += ["", "/**", f" * @brief {macro} 寄存器实例指针。"
            f" {macro} register-instance pointer.", " */",
            f"#define {macro} (({type_name} *)(uintptr_t){base})", "", "/** @} */"]
    return "\n".join(out)


def emit_fields() -> str:
    out = ["/**",
           " * @defgroup SG2002_IOBLK_FIELDS IOBLK 焊盘字段位 / IOBLK pad field bits",
           " * @ingroup SG2002_DEVICE",
           " * @see SG2002 PINOUT 表 3「管脚控制寄存器」；TRM v1.02 第 10 章未收录。",
           " *      SG2002 PINOUT sheet 3; TRM v1.02 chapter 10 does not cover this.",
           " * @{", " */", "",
           "/// @name 全部焊盘共有的字段 / Fields present on every pad", "/// @{"]
    for name, bit, cn, en in COMMON_FIELDS:
        out += [f"/// @brief {cn}。", f"///        {en.capitalize()}. 位 {bit}。",
                f"static constexpr auto IOBLK_{name}_BIT = {bit}U;"]
    out += ["/// @}", "",
            "/// @name 布局 A 独有（34 个焊盘有输入施密特与总线保持）/ Layout A only", "/// @{"]
    for name, bit, cn, en in LAYOUT_A:
        out += [f"/// @brief {cn}。", f"///        {en.capitalize()}. 位 {bit}。",
                f"static constexpr auto IOBLK_{name}_BIT = {bit}U;"]
    out += ["/// @}", "",
            "/// @name 布局 B 独有（28 个焊盘有第三档驱动能力）/ Layout B only", "/// @{"]
    for name, bit, cn, en in LAYOUT_B:
        out += [f"/// @brief {cn}。", f"///        {en.capitalize()}. 位 {bit}。",
                f"static constexpr auto IOBLK_{name}_BIT = {bit}U;"]
    out += ["/// @}", "",
            "/// @name 布局 C 独有（XDS 扩展驱动档位）/ Layout C only", "/// @{"]
    for name, bit, cn, en in LAYOUT_C:
        out += [f"/// @brief {cn}。", f"///        {en.capitalize()}. 位 {bit}。",
                f"static constexpr auto IOBLK_{name}_BIT = {bit}U;"]
    out += ["/// @}", "", "/** @} */"]
    return "\n".join(out)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("workbook", type=Path)
    parser.add_argument("--group", choices=["G1", "GRTC", "fields", "all"], default="all")
    args = parser.parse_args()
    if not args.workbook.is_file():
        parser.error(f"workbook not found: {args.workbook}")

    active, rtc = read_pads(args.workbook)
    blocks = []
    if args.group in ("G1", "all"):
        blocks.append(emit_struct(
            active, "IOBLK_Type", "IOBLK_G1", "IOBLK_REGS", "IOBLK_BASE",
            "SG2002 PINOUT 表 3；TRM v1.02 第 10 章指向该表。"
            "SG2002 PINOUT sheet 3, referenced by TRM v1.02 chapter 10."))
    if args.group in ("GRTC", "all"):
        blocks.append(emit_struct(
            rtc, "RTC_IO_Type", "IOBLK_GRTC", "RTC_IO_REGS", "RTC_IO_BASE",
            "SG2002 PINOUT 表 3 的 RTC 域焊盘；基址即 RTC_IO_BASE。"
            "RTC-domain pads from PINOUT sheet 3, based at RTC_IO_BASE."))
    if args.group in ("fields", "all"):
        blocks.append(emit_fields())
    print("\n\n".join(blocks))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
