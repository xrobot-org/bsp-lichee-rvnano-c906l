#!/usr/bin/env python3
"""Extract register overview tables from the extracted SG2002 TRM text.

The TRM lists each peripheral twice: an "X Registers Overview" table giving the
offset, name and one-line description of every register, and one "Register
Description" table per register giving its bit fields. The overview tables are
what a device struct needs, so they drive the migration; the per-register tables
supply the bit fields on demand.

Usage:
  trm_registers.py --list                 # every overview table found
  trm_registers.py --peripheral GPIO      # one peripheral, as a table
  trm_registers.py --peripheral GPIO --c  # ... rendered as struct members
"""
from __future__ import annotations

import argparse
import os
import re
import sys
from dataclasses import dataclass
from pathlib import Path

# This file lives in <bsp>/scripts/sg200x-ll/, so the BSP root is two levels up.
BSP_ROOT = Path(__file__).resolve().parents[2]
TRM_NAME = "sg2002_trm_en_v1.02.txt"


def default_trm() -> Path:
    """The TRM text is a Git-ignored reference copy, so probe likely homes.

    Override with --trm or the SG2002_TRM environment variable.
    """
    override = os.environ.get("SG2002_TRM")
    if override:
        return Path(override)
    candidates = [BSP_ROOT / "docs/reference" / TRM_NAME]
    # A sibling template checkout keeps the extracted TRM next to the PDF.
    candidates += sorted(Path.home().glob(f"*/docs/reference/{TRM_NAME}"))
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    return candidates[0]


DEFAULT_TRM = default_trm()

# A register name in the overview tables is any C identifier: the TRM mixes
# GPIO_SWPORTA_DR, Timer1LoadCount and conf_info. The column header words are
# excluded, and a name only counts as a record when an offset follows it, which
# keeps wrapped description fragments from being mistaken for names.
NAME_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]{1,40}$")
HEADER_WORDS = {"Name", "Address", "Offset", "Description", "Bits", "Access", "Reset"}
# Register offsets are written with at least three hex digits (0x000, 0x004).
# Accepting a bare "0x1" would let a bit-field table's reset-value column be
# mistaken for an offset, which is exactly how the clock-bypass tables fooled an
# earlier revision.
OFFSET_RE = re.compile(r"^0x[0-9a-fA-F]{3,8}$")
# A peripheral register block is small; anything larger is an address map.
MAX_REGISTER_OFFSET = 0x20000
# Table captions that introduce an offset/name/description listing. The TRM also
# contains the typo "Registes Overview", so match the word loosely.
OVERVIEW_RE = re.compile(r"^Table ([\d.]+):\s*(.*?)\s*$")
TITLE_RE = re.compile(r"^\s*Table\s+[\d.]+:\s*(.*?)\s*$")
# Page furniture repeated on every extracted page: page markers, the copyright
# line, bare page numbers, running chapter headers, the "continues on next page"
# note, the "Table N.M - continued from previous page" banner, and the column
# headers repeated after a page break.
NOISE_RE = re.compile(
    r"^(===== PAGE \d+ =====|Copyright © \d+ SOPHGO Co\., Ltd|\d+|CHAPTER \d+\..*"
    r"|continues on next page|Table [\d.]+ [–-] continued from previous page"
    r"|Name|Address Offset|Address|Offset|Description|Bits|Access|Reset)$")
SECTION_RE = re.compile(r"^(\d+(?:\.\d+)+)\s+(\S.*)$")


@dataclass
class Register:
    offset: int
    name: str
    description: str


@dataclass
class Overview:
    table: str
    title: str
    registers: list[Register]

    @property
    def peripheral(self) -> str:
        head = re.split(r"\s*Overview", self.title, flags=re.I)[0]
        head = re.sub(r"\s+(Registers?|Registes|REG)$", "", head, flags=re.I).strip()
        return head or self.title

    @property
    def base(self) -> int | None:
        """Base address stated in the caption, when the TRM gives one."""
        match = re.search(r"Ba\w*[:\s]+0x([0-9A-Fa-f_]+)", self.title)
        return int(match.group(1).replace("_", ""), 16) if match else None


def load(path: Path) -> list[str]:
    return path.read_text(encoding="utf-8").split("\n")


def is_overview(title: str) -> bool:
    return "overview" in title.lower()


def is_record(lines: list[str], index: int) -> bool:
    """True when ``index`` starts a NAME line followed by an offset line."""
    if index + 1 >= len(lines):
        return False
    name = lines[index].strip()
    return (bool(NAME_RE.match(name)) and name not in HEADER_WORDS
            and bool(OFFSET_RE.match(lines[index + 1].strip())))


def parse_overview(lines: list[str], start: int) -> tuple[Overview, int]:
    """Parse the overview table whose caption is at ``start``."""
    caption = OVERVIEW_RE.match(lines[start])
    table, title = caption.group(1), caption.group(2)
    registers: list[Register] = []
    index = start + 1
    # Skip the column header ("Name / Address Offset / Description") and page
    # furniture. A record is a NAME line followed by an offset line, and the
    # table ends at the next caption or numbered section heading.
    while index < len(lines):
        line = lines[index].strip()
        if OVERVIEW_RE.match(line) or SECTION_RE.match(line):
            break
        if is_record(lines, index):
            offset = int(lines[index + 1].strip(), 16)
            name = line
            index += 2
            description: list[str] = []
            while index < len(lines):
                nxt = lines[index].strip()
                if OVERVIEW_RE.match(nxt) or SECTION_RE.match(nxt):
                    break
                if is_record(lines, index):
                    break
                if not NOISE_RE.match(nxt) and nxt:
                    description.append(nxt)
                index += 1
            # A word broken across a line ends in "-"; the next fragment joins it.
            text = ""
            for fragment in description:
                if text.endswith("-"):
                    text = text[:-1] + fragment
                elif text:
                    text += " " + fragment
                else:
                    text = fragment
            registers.append(Register(offset, name, re.sub(r"\s+", " ", text).strip()))
            continue
        index += 1
    return Overview(table, title, registers), index


def is_register_listing(overview: Overview) -> bool:
    """True when a parsed table really looks like a peripheral register list.

    A register listing has at least three registers, offsets that strictly
    increase, and offsets inside one small block. Bit-field tables and the
    memory map fail these tests.
    """
    offsets = [r.offset for r in overview.registers]
    if len(offsets) < 3 or max(offsets) > MAX_REGISTER_OFFSET:
        return False
    return all(later > earlier for earlier, later in zip(offsets, offsets[1:]))


def find_overviews(lines: list[str]) -> list[Overview]:
    """Every table that lists registers as NAME followed by an offset.

    Recognition is structural rather than by caption: the temperature-sensor
    table is captioned "BassAddress: 0x030A0000" with no "Overview" word at all,
    and the per-register description tables never produce a NAME/offset pair.
    A minimum of three registers keeps stray two-row tables out.
    """
    found: list[Overview] = []
    index = 0
    while index < len(lines):
        if OVERVIEW_RE.match(lines[index].strip()):
            overview, index = parse_overview(lines, index)
            if is_register_listing(overview):
                found.append(overview)
            continue
        index += 1
    return found


STRUCT_RE = re.compile(
    r"typedef struct\s*\{(.*?)\n\}\s*([A-Za-z_][A-Za-z0-9_]*);", re.S)
# Every member in sg2002.h carries its offset in the trailing doc comment
# ("偏移 0xNNN" / "Offset 0xNNN"), which is authoritative and avoids modelling C
# layout for symbolic sizes such as NUM_IRQ or sizeof(...) expressions.
# Covers the historic bilingual marker and the CMSIS-style "Address offset:".
OFFSET_COMMENT_RE = re.compile(
    r"(?:偏移|Address offset|Offset)\s*:?\s*0x([0-9A-Fa-f]+)")
IDENT_RE = re.compile(r"[A-Za-z_][A-Za-z0-9_]*")
COMMENT_RE = re.compile(r"/\*.*?\*/|//[^\n]*", re.S)

# Which TRM overview table describes each device struct in sg2002.h. Peripherals
# without an overview table in the TRM (PLIC, PINMUX, CLKGEN, PLL, RSTGEN, DMA,
# SARADC) cannot be cross-checked this way.
# type -> (TRM table, overview_is_partial). A partial overview describes a
# repeating block once in prose instead of listing every instance, as TIMER does
# for Timer2-8, so sg200x-ll-only offsets there are expected rather than a defect.
STRUCT_TABLES = {
    "GPIO_Type": ("21.141", False),
    # Table 9.2 omits the DDR address-mode register at 0x064 that the vendor SDK
    # drives (TOP_DDR_ADDR_MODE_REG), so sg200x-ll models one register more.
    "TOP_Type": ("9.2", True),
    "SPI_Type": ("21.72", True),
    # The I2C, SPI and UART overviews omit registers that sg200x-ll models from the
    # DesignWare datasheet (IC_COMP_*, UART_SCR/CPR/UCV_CTR, SPI_DR), so
    # sg200x-ll-only offsets there are expected rather than a defect.
    "I2C_Type": ("21.3", True),
    "UART_Type": ("21.45", True),
    "PWM_Type": ("21.265", True),
    "WDT_Type": ("13.1", False),
    "RTC_CTRL_Type": ("6.3", False),
    "TIMER_Type": ("12.1", True),
    "TRNG_Type": ("22.76", False),
    "KEYSCAN_Type": ("21.292", False),
    "WGN_Type": ("21.304", False),
    "IRRX_Type": ("21.322", False),
    "SPI_NOR_Type": ("16.1", False),
    "SPI_NAND_Type": ("17.3", False),
    "GMAC_Type": ("18.1", False),
    "TEMPSEN_Type": ("21.246", False),
    "SDMMC_Type": ("21.102", False),
    "USB_Type": ("21.154", False),
    "USB_HOST_Type": ("21.197", False),
    "USB_DEVICE_Type": ("21.213", False),
    "CRYPTO_DMA_Type": ("22.1", False),
    "I2S_GLOBAL_Type": ("20.1", False),
    "I2S_Type": ("20.2", False),
    "AUDIO_CODEC_Type": ("20.31", False),
    "RTC_CORE_Type": ("6.1", False),
    "RTC_MACRO_Type": ("6.2", False),
    "PLL_G2_Type": ("8.5", False),
    # Table 21.232 documents the SARADC block once, using the No-die instance
    # (RTCSYS_SARADC, 0x0502C000); the active-domain instance shares it.
    "SARADC_Type": ("21.232", False),
    "RSTGEN_Type": ("7.1", True),
    "PLL_G6_Type": ("8.6", False),
    # Table 8.51 runs several divider rows together, so the generic reader
    # absorbs one entry into the previous description; the divider mapping was
    # verified separately (20/20 against the TRM) before that audit was retired.
    "CLKGEN_Type": ("8.51", True),
}


@dataclass
class Member:
    offset: int
    name: str
    type: str
    count: int


def leading_comment(text: str) -> str:
    """Return the comment that follows a member's ";", either style."""
    text = text.lstrip()
    if text.startswith("/*"):
        end = text.find("*/")
        return text[:end + 2] if end >= 0 else text
    lines = []
    for line in text.split("\n"):
        stripped = line.lstrip()
        if not stripped.startswith("///"):
            break
        lines.append(stripped[3:])
    return " ".join(lines)


def split_declarations(body: str) -> list[str]:
    """Split a struct body on ";" while ignoring semicolons inside comments.

    The member offsets live in the doc comments, so the comments cannot simply
    be stripped first. A plain split(";") breaks whenever a comment contains a
    semicolon, which silently corrupts the following member's name.
    """
    parts: list[str] = []
    current: list[str] = []
    index, length = 0, len(body)
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


def parse_structs(header: str) -> dict[str, list[Member]]:
    """Members of every device struct in sg2002.h.

    The offset comes from each member's doc comment rather than from a C layout
    model, so arrays sized by constants or sizeof() expressions are handled.
    """
    # Array sizes are often symbolic (PWM_CHANNELS_PER_CONTROLLER), so collect
    # the integer constants declared in the same header first.
    constants = {name: int(value) for name, value in
                 re.findall(r"^(?:static constexpr auto|#define)\s+"
                            r"([A-Za-z_][A-Za-z0-9_]*)\s*=?\s*\(?\s*(\d+)"
                            r"[uUlL]*\s*\)?\s*;?\s*$", header, re.M)}
    structs: dict[str, list[Member]] = {}
    for body, name in STRUCT_RE.findall(header):
        members: list[Member] = []
        chunks = split_declarations(body)
        for index, chunk in enumerate(chunks[:-1]):
            declaration = COMMENT_RE.sub(" ", chunk)
            # Drop leading qualifiers, then the declaration is TYPE NAME [...].
            # Taking the last identifier instead would pick up identifiers from
            # an array size such as "[(0x154U - 0x00CU) / sizeof(uint32_t)]".
            unqualified = re.sub(r"^\s*(?:(?:volatile|const|static|struct)\s+)+", "",
                                 declaration)
            identifiers = IDENT_RE.findall(unqualified)
            if len(identifiers) < 2:
                continue
            # Only the comment that trails this declaration may be searched: a
            # fixed-size window would miss the offset once a long description
            # wraps, and scanning further would pick up the next member's.
            match = OFFSET_COMMENT_RE.search(leading_comment(chunks[index + 1]))
            if not match:
                continue
            size = re.search(r"\[\s*([A-Za-z_][A-Za-z0-9_]*|\d+)\s*\]", declaration)
            count = 1
            if size:
                token = size.group(1)
                count = int(token) if token.isdigit() else constants.get(token, 1)
            members.append(Member(int(match.group(1), 16), identifiers[1],
                                  identifiers[0], count))
        structs[name] = members
    return structs


def flatten(structs: dict[str, list[Member]], type_name: str,
            depth: int = 0) -> dict[int, str]:
    """Offsets of every register in a struct, expanding nested struct arrays."""
    result: dict[int, str] = {}
    for member in structs.get(type_name, []):
        if member.type in structs and depth < 3:
            inner = flatten(structs, member.type, depth + 1)
            if not inner:
                continue
            stride = max(inner) + 4
            for instance in range(member.count):
                for offset, name in inner.items():
                    absolute = member.offset + instance * stride + offset
                    result[absolute] = (f"{member.name}[{instance}].{name}"
                                        if member.count > 1 else name)
            continue
        # A scalar array (uint32_t PCOUNT[4]) occupies one word per element, so
        # every element offset has to be recorded, not just the first.
        for instance in range(member.count):
            offset = member.offset + instance * 4
            result[offset] = (f"{member.name}[{instance}]" if member.count > 1
                              else member.name)
    return result


def strip_prefix(name: str, peripheral: str) -> str:
    """The TRM prefixes GPIO_/I2C_/UART_... where sg200x-ll drops the prefix."""
    token = re.sub(r"[^A-Za-z0-9]", "", peripheral).upper()
    upper = name.upper()
    return upper[len(token):] if token and upper.startswith(token) else upper


def describe(members) -> dict[int, str]:
    return {offset: name for offset, name in members}


def render_c(overview: Overview, type_name: str, gap_name: str = "RESERVED") -> str:
    """Render struct members with reserved gaps, mirroring sg2002.h style."""
    out: list[str] = []
    previous_end = 0
    for register in sorted(overview.registers, key=lambda r: r.offset):
        if register.offset > previous_end:
            words = (register.offset - previous_end) // 4
            if words:
                out.append(f"    uint32_t {gap_name}_{previous_end:03X}[{words}];"
                           f" ///< 偏移 0x{previous_end:03X}：未公开区域。"
                           f" Offset 0x{previous_end:03X}: unexposed area.")
        out.append(f"    volatile uint32_t {register.name};"
                   f" ///< 偏移 0x{register.offset:03X}：{register.description}")
        previous_end = register.offset + 4
    return "\n".join(out)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trm", type=Path, default=None)
    parser.add_argument("--list", action="store_true", help="list every overview table")
    parser.add_argument("--peripheral", help="substring match on the table title")
    parser.add_argument("--table", help="exact TRM table id, e.g. 16.1")
    parser.add_argument("--c", action="store_true", help="render struct members")
    parser.add_argument("--type-name", default="Device_Type")
    parser.add_argument("--verify", type=Path,
                        help="cross-check a sg2002.h against the TRM overview tables")
    args = parser.parse_args()

    args.trm = args.trm or default_trm()
    if not args.trm.is_file():
        parser.error(f"TRM text not found: {args.trm}\n"
                     "Put the extracted TRM at <bsp>/docs/reference/" + TRM_NAME +
                     ", or pass --trm / set SG2002_TRM.")
    overviews = find_overviews(load(args.trm))

    if args.verify:
        by_table = {o.table: o for o in overviews}
        structs = parse_structs(args.verify.read_text(encoding="utf-8"))
        total_missing = total_reserved = total_extra = 0
        for type_name, (table, partial) in sorted(STRUCT_TABLES.items()):
            overview = by_table.get(table)
            if overview is None or type_name not in structs:
                print(f"{type_name:<16} SKIP (no TRM table {table} or no struct)")
                continue
            trm = describe([(r.offset, r.name) for r in overview.registers])
            sg200x_ll = describe(sorted(flatten(structs, type_name).items()))
            # A TRM register absent from the struct entirely, or present only as
            # unnamed padding, is a real gap; unnamed padding elsewhere is not.
            missing = sorted(o for o in trm if o not in sg200x_ll)
            reserved = sorted(o for o in trm
                              if o in sg200x_ll and sg200x_ll[o].startswith("RESERVED"))
            extra = [] if partial else sorted(
                o for o in sg200x_ll
                if o not in trm and not sg200x_ll[o].startswith("RESERVED"))
            total_missing += len(missing)
            total_reserved += len(reserved)
            total_extra += len(extra)
            named = len([o for o in sg200x_ll if not sg200x_ll[o].startswith("RESERVED")])
            status = "OK" if not (missing or reserved or extra) else "DIFF"
            print(f"{type_name:<16} table {table:<8} TRM {len(trm):>3} "
                  f"sg200x-ll {named:>3}  {status}")
            for offset in missing:
                print(f"    MISSING   0x{offset:03X} {trm[offset]}")
            for offset in reserved:
                print(f"    PADDED    0x{offset:03X} {trm[offset]}  <- sg200x-ll 用 RESERVED 占位")
            for offset in extra:
                print(f"    sg200x-ll-only 0x{offset:03X} {sg200x_ll[offset]}")
        print(f"\n合计：完全缺失 {total_missing}，仅占位 {total_reserved}，"
              f"TRM 未列 {total_extra}")
        return 1 if total_missing or total_reserved or total_extra else 0

    if args.list:
        for overview in overviews:
            print(f"Table {overview.table:<8} {overview.peripheral:<28} "
                  f"{len(overview.registers):>4} registers")
        print(f"\n{len(overviews)} overview tables, "
              f"{sum(len(o.registers) for o in overviews)} registers")
        return 0

    if args.table:
        matches = [o for o in overviews if o.table == args.table]
        if not matches:
            parser.error(f"no overview table with id {args.table!r}")
    elif not args.peripheral:
        parser.error("--peripheral, --table or --list is required")
    else:
        matches = [o for o in overviews
                   if args.peripheral.lower() in o.title.lower()
                   or args.peripheral.lower() in o.peripheral.lower()]
    if not matches:
        parser.error(f"no overview table matches {args.peripheral!r}")
    for overview in matches:
        print(f"# Table {overview.table}: {overview.title} "
              f"({len(overview.registers)} registers)")
        if args.c:
            print(render_c(overview, args.type_name))
        else:
            for register in sorted(overview.registers, key=lambda r: r.offset):
                print(f"  0x{register.offset:03X}  {register.name:<28} {register.description}")
        print()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
