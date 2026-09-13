#!/usr/bin/env python3
"""Render sg2002.h device structs from the TRM register overview tables.

The overview tables are not enough on their own: several peripherals describe a
repeating block once ("Timer2 Registers 0x014~0x024 ... the contents are the
same as Timer1") instead of listing every instance. Each such peripheral needs a
small structural spec here, giving the repeating sub-type, its stride and count,
and the member names to use inside it.

Comments follow the house style: the English sentence is the TRM text verbatim
so it stays machine-comparable, and the Chinese annotation is assembled from the
register name tokens through the glossary below.
"""
from __future__ import annotations

import argparse
import re
from dataclasses import dataclass, field
from pathlib import Path

import trm_registers as trm

# Register-name tokens to short Chinese annotations. Adjacent tokens are matched
# as a compound first, so LOAD + COUNT resolves through the LOADCOUNT entry.
# Trailing digits are split off before matching and re-appended ("status0" ->
# status + "0"), so indexed registers keep their index.
GLOSSARY = [
    # Counting, timing and control
    ("LOAD_COUNT", "装载计数"), ("LOADCOUNT", "装载计数"), ("CURRENT_VALUE", "当前值"),
    ("CURRENTVALUE", "当前值"), ("LOAD", "装载"), ("COUNT", "计数"), ("VALUE", "值"),
    ("PERIOD", "周期"), ("WIDTH", "宽度"), ("LEN", "长度"), ("LENGTH", "长度"),
    ("THRESHOLD", "阈值"), ("TIMEOUT", "超时"), ("DELAY", "延时"), ("WAIT", "等待"),
    ("START", "启动"), ("STOP", "停止"), ("UPDATE", "更新"), ("MODE", "模式"),
    ("IDLE", "空闲"), ("BUSY", "忙"), ("DONE", "完成"), ("READY", "就绪"),
    # Interrupts and status
    ("RAW_INT_STATUS", "原始中断状态"), ("INT_STATUS", "中断状态"),
    ("INTSTATUS", "中断状态"), ("INTERRUPT", "中断"), ("INT", "中断"), ("IRQ", "中断"),
    ("EOI", "中断清除"), ("PENDING", "待处理"), ("CLAIM", "领取"), ("COMPLETE", "完成"),
    ("STATUS", "状态"), ("FLAG", "标志"), ("MASK", "屏蔽"), ("ENABLE", "使能"),
    ("DISABLE", "禁用"), ("EN", "使能"), ("TRIGGER", "触发"), ("LEVEL", "电平"),
    ("EDGE", "边沿"), ("POLARITY", "极性"), ("CLEAR", "清除"), ("CLR", "清除"),
    # Clocks, reset and power
    ("CLK", "时钟"), ("CLOCK", "时钟"), ("CLKBYP", "时钟旁路"), ("CLKEN", "时钟使能"),
    ("CLKDIV", "分频"), ("DIV", "分频"), ("MUX", "选择"), ("BYPASS", "旁路"),
    ("RESET", "复位"), ("RST", "复位"), ("POR", "上电复位"), ("PMU", "电源管理"),
    ("PWR", "电源"), ("POWER", "电源"), ("WKUP", "唤醒"), ("WAKE", "唤醒"),
    ("ISO", "隔离"), ("REQ", "请求"), ("ACK", "应答"), ("GATE", "门控"),
    # Register-block vocabulary
    ("CONTROL", "控制"), ("CONTROLREG", "控制"), ("CTRL", "控制"), ("REG", "寄存器"),
    ("CONFIG", "配置"), ("CFG", "配置"), ("VERSION", "版本"), ("TYPE", "类型"),
    ("OPTION", "选项"), ("SELECT", "选择"), ("SEL", "选择"), ("SOURCE", "源"),
    ("SPARE", "备用"), ("TEMP", "临时"), ("DEBUG", "调试"), ("DBG", "调试"),
    ("SOFT", "软"), ("SW", "软件"), ("HW", "硬件"), ("RO", "只读"),
    # Identity and calibration
    ("RTC", "RTC"), ("SYS", "系统"), ("MCU51", "MCU51"), ("SRAM", "SRAM"),
    ("GPIO", "GPIO"), ("IP", "IP"), ("FAB", "制造"), ("CHIP", "芯片"),
    ("CAL", "校准"), ("COARSE", "粗调"), ("FINE", "细调"), ("TRIM", "微调"),
    ("UNLOCK", "解锁"), ("UNLOCKKEY", "解锁密钥"), ("KEY", "密钥"), ("LOCK", "锁定"),
    ("PARAM", "参数"), ("STAT", "统计"), ("CONF", "配置"), ("INFO", "信息"),
    ("REMAP", "重映射"), ("URGENT", "紧急"), ("QOS", "QoS"), ("NUM", "数量"), ("MAX", "最大"),
    ("MIN", "最小"), ("ALL", "全部"), ("RAW", "原始"), ("TOTAL", "总计"),
    # Data movement
    ("DATA", "数据"), ("ADDR", "地址"), ("ADDRESS", "地址"), ("FIFO", "先进先出"),
    ("BUFFER", "缓冲"), ("BUF", "缓冲"), ("TX", "发送"), ("RX", "接收"),
    ("SEND", "发送"), ("RECV", "接收"), ("FULL", "满"), ("EMPTY", "空"),
    ("OVF", "溢出"), ("UNDERRUN", "欠载"), ("OVERRUN", "过载"), ("FLUSH", "刷新"),
    ("CHANNEL", "通道"), ("CH", "通道"), ("PORT", "端口"), ("PIN", "引脚"),
    ("SAMPLE", "采样"), ("RATE", "速率"), ("SHIFT", "移位"), ("GEN", "生成"),
    # Storage, network and audio
    ("SDMMC", "SDMMC"), ("SD", "SD"), ("EMMC", "eMMC"), ("CMD", "命令"),
    ("ARG", "参数"), ("RESP", "响应"), ("BLOCK", "块"), ("SECTOR", "扇区"),
    ("ETH", "以太网"), ("GMAC", "GMAC"), ("MAC", "MAC"), ("PHY", "PHY"),
    ("LINK", "链路"), ("SPEED", "速率"), ("DUPLEX", "双工"), ("PAUSE", "暂停"),
    ("FLOW", "流控"), ("COLLISION", "冲突"), ("JABBER", "超长帧"),
    ("USB", "USB"), ("EP", "端点"), ("ENDPOINT", "端点"), ("PORT", "端口"),
    ("CONNECT", "连接"), ("SUSPEND", "挂起"), ("RESUME", "恢复"), ("FRAME", "帧"),
    ("I2S", "I2S"), ("TDM", "TDM"), ("AUDIO", "音频"), ("DAC", "DAC"), ("ADC", "ADC"),
    ("VOLUME", "音量"), ("MUTE", "静音"), ("VALID", "有效"),
    ("CRYPTO", "加密"), ("AES", "AES"), ("SHA", "SHA"), ("RNG", "随机数"),
    ("IV", "初始向量"), ("SEED", "种子"), ("TRNG", "真随机数"),
]

CJK = re.compile(r"[\u4e00-\u9fff]")


def name_tokens(name: str) -> list[str]:
    """Split GPIO_SWPORTA_DR / Timer1LoadCount / conf_info into tokens."""
    spaced = re.sub(r"(?<=[a-z0-9])(?=[A-Z])", " ", name)
    return [t for t in re.split(r"[^A-Za-z0-9]+", spaced) if t]


def screaming(name: str) -> str:
    """Normalise any TRM spelling to the SCREAMING_SNAKE style sg200x-ll uses."""
    spaced = re.sub(r"(?<=[a-z0-9])(?=[A-Z])", "_", name)
    spaced = re.sub(r"(?<=[A-Z])(?=[A-Z][a-z])", "_", spaced)
    return re.sub(r"[^A-Za-z0-9]+", "_", spaced).upper().strip("_")


def chinese_hint(name: str) -> str:
    """A short Chinese annotation assembled from the register-name tokens.

    Adjacent tokens are matched as a compound first, so LOAD + COUNT resolves
    through the LOADCOUNT entry rather than as two separate words. A trailing
    digit is split off before matching and re-appended so indexed registers keep
    their index ("status0" -> "状态0").
    """
    tokens = name_tokens(name)
    words: list[str] = []
    index = 0
    while index < len(tokens):
        token = tokens[index]
        digits = ""
        bare = re.fullmatch(r"([A-Za-z_]+)(\d+)", token)
        if bare:
            token, digits = bare.group(1), bare.group(2)
        for span in range(min(3, len(tokens) - index), 0, -1):
            parts = [token] if span == 1 else [t for t in tokens[index:index + span]]
            key = "".join(t.upper() for t in parts)
            match = next((value for entry, value in GLOSSARY if entry == key), None)
            if match:
                words.append(match + (digits if span == 1 else ""))
                index += span
                break
        else:
            words.append(token + digits)
            index += 1
    text = "".join(words)
    # A gloss assembled mostly from untranslated name tokens just repeats the
    # identifier, so it is dropped rather than shown next to it.
    cjk = len(re.findall(r"[\u4e00-\u9fff]", text))
    return text if text and cjk / len(text) >= 0.30 else ""


# A repeating block is described once and then referenced by a sentence such as
# "Timer2 Registers 0x014~ 0x024 There are 5 registers in total, the contents
# are the same as Timer1." That sentence belongs to the table, not to the last
# register of the first block, so cut it before the description is emitted.
REPEAT_NOTE_RE = re.compile(
    r"\s*\w*\s*Registers?\s+0x[0-9a-fA-F]+\s*~?.*$", re.S)


def clean_description(text: str) -> str:
    return REPEAT_NOTE_RE.sub("", text).strip().rstrip(".")


@dataclass
class Spec:
    """How one peripheral's registers are laid out."""
    peripheral: str
    type_name: str
    group: str
    title: str = ""                      # display name, defaults to peripheral
    table: str | None = None             # TRM table id, when the name is ambiguous
    instance: str | None = None          # instance macro name
    base_macro: str | None = None        # BASE constant to point at
    # More than one instance of the same block, as (macro, base constant).
    instances: list[tuple[str, str]] = field(default_factory=list)
    # Byte offset added to the base constant, for sub-blocks such as the USB
    # host and device register banks.
    base_offset: int = 0
    # offset -> member name override
    renames: dict[int, str] = field(default_factory=dict)
    # offset -> description override, for TRM copy/paste errors
    descriptions: dict[int, str] = field(default_factory=dict)
    # A repeating block: (type name, count, stride, first offset, member count)
    repeat: tuple[str, int, int, int, int] | None = None
    see: str = ""


SPECS: dict[str, Spec] = {
    "Timer": Spec(
        peripheral="Timer",
        type_name="TIMER_Type",
        group="SG2002_TIMER",
        instance="TIMER_REGS",
        base_macro="TIMER_BASE",
        renames={0x000: "LOAD_COUNT", 0x004: "CURRENT_VALUE", 0x008: "CONTROL",
                 0x00C: "EOI", 0x010: "INT_STATUS"},
        repeat=("TIMER_Channel_Type", 8, 0x14, 0x000, 5),
        see="SG2002 技术参考手册 v1.02, 表 12.1-12.9。SG2002 TRM v1.02, Tables 12.1-12.9.",
    ),
    # RTC_CTRL was originally modelled down to the two registers SARADC needs.
    # The whole block is migrated now, but those two keep their published names:
    # sg200x_ll_rcc.c writes RTC_CTRL_REGS->RESET and ->CLOCK_MUX.
    "RTC_CTRL_REG": Spec(
        peripheral="RTC_CTRL",
        type_name="RTC_CTRL_Type",
        group="SG2002_RTC_CTRL",
        instance="RTC_CTRL_REGS",
        base_macro="RTC_CTRL_BASE",
        renames={0x018: "RESET", 0x01C: "CLOCK_MUX"},
        see="SG2002 技术参考手册 v1.02, 表 6.3。SG2002 TRM v1.02, Table 6.3.",
    ),
    # TOP was modelled down to the registers the drivers touch; the whole block
    # is migrated now, keeping the published member names. The TRM's own
    # description for top_wdt_ctrl repeats the previous row, hence the override.
    "System Control": Spec(
        peripheral="System Control",
        type_name="TOP_Type",
        group="SG2002_TOP",
        title="TOP",
        table="9.2",
        instance="TOP",
        base_macro="TOP_MISC_BASE",
        renames={0x008: "SYS_CTRL", 0x154: "DMA_REMAP0", 0x158: "DMA_REMAP1",
                 0x1A8: "WDT_CTRL", 0x294: "SD1_SELECT", 0x298: "DMA_INTERRUPT_MUX"},
        descriptions={0x1A8: "Watchdog reset-routing control"},
        see="SG2002 技术参考手册 v1.02, 表 9.2。SG2002 TRM v1.02, Table 9.2.",
    ),
    "TRNG": Spec(
        peripheral="TRNG",
        type_name="TRNG_Type",
        group="SG2002_TRNG",
        table="22.76",
        instance="TRNG_REGS",
        base_macro="TRNG_BASE",
        see="SG2002 技术参考手册 v1.02, 表 22.76-22.83。SG2002 TRM v1.02, Tables 22.76-22.83.",
    ),
    "Key scan": Spec(
        peripheral="Key scan",
        type_name="KEYSCAN_Type",
        group="SG2002_KEYSCAN",
        title="KeyScan",
        table="21.292",
        instance="KEYSCAN_REGS",
        base_macro="KEYSCAN_BASE",
        see="SG2002 技术参考手册 v1.02, 表 21.292-21.303。SG2002 TRM v1.02, Tables 21.292-21.303.",
    ),
    # One block, three instances.
    "Wiegand": Spec(
        peripheral="Wiegand",
        type_name="WGN_Type",
        group="SG2002_WGN",
        table="21.304",
        instances=[("WGN0_REGS", "WGN0_BASE"), ("WGN1_REGS", "WGN1_BASE"),
                   ("WGN2_REGS", "WGN2_BASE")],
        see="SG2002 技术参考手册 v1.02, 表 21.304-21.321。SG2002 TRM v1.02, Tables 21.304-21.321.",
    ),
    "IRRX": Spec(
        peripheral="IRRX",
        type_name="IRRX_Type",
        group="SG2002_IRRX",
        table="21.322",
        instance="IRRX_REGS",
        base_macro="IRRX_BASE",
        see="SG2002 技术参考手册 v1.02, 表 21.322 起。SG2002 TRM v1.02, Table 21.322 onwards.",
    ),
    # Table 16.1 is captioned just "Registers Overview", so it is selected by id.
    # The TRM documents the block but never states its register base, so no
    # instance macro is emitted; only the flash window has a published address.
    "SPI NOR": Spec(
        peripheral="SPI NOR",
        type_name="SPI_NOR_Type",
        group="SG2002_SPI_NOR",
        title="SPI NOR",
        table="16.1",
        see="SG2002 技术参考手册 v1.02, 表 16.1-16.10。SG2002 TRM v1.02, Tables 16.1-16.10.",
    ),
    "SPI NANDFLASH": Spec(
        peripheral="SPI NANDFLASH",
        type_name="SPI_NAND_Type",
        group="SG2002_SPI_NAND",
        title="SPI NAND",
        table="17.3",
        instance="SPI_NAND_REGS",
        base_macro="SPI_NAND_BASE",
        see="SG2002 技术参考手册 v1.02, 表 17.3-17.13。SG2002 TRM v1.02, Tables 17.3-17.13.",
    ),
    "GMAC": Spec(
        peripheral="GMAC",
        type_name="GMAC_Type",
        group="SG2002_ETH0",
        title="ETH0/GMAC",
        table="18.1",
        instance="ETH0_REGS",
        base_macro="ETH0_BASE",
        see="SG2002 技术参考手册 v1.02, 表 18.1 起。SG2002 TRM v1.02, Table 18.1 onwards.",
    ),
    # The TRM captions this table "BassAddress: 0x030A0000", which is TIMER's
    # base and a copy/paste error; TEMPSEN_BASE from the memory map is used.
    "BassAddress": Spec(
        peripheral="Temperature sensor",
        type_name="TEMPSEN_Type",
        group="SG2002_TEMPSEN",
        title="TEMPSEN",
        table="21.246",
        instance="TEMPSEN_REGS",
        base_macro="TEMPSEN_BASE",
        see="SG2002 技术参考手册 v1.02, 表 21.246-21.263。SG2002 TRM v1.02, Tables 21.246-21.263.",
    ),
    # One SDMMC controller instantiated three times; the TRM gives no base in
    # the caption, so the memory-map constants are used.
    "SDMMC": Spec(
        peripheral="SDMMC",
        type_name="SDMMC_Type",
        group="SG2002_SDMMC",
        table="21.102",
        instances=[("EMMC_REGS", "EMMC_BASE"), ("SD0_REGS", "SD0_BASE"),
                   ("SD1_REGS", "SD1_BASE")],
        see="SG2002 技术参考手册 v1.02, 表 21.102-21.140。SG2002 TRM v1.02, Tables 21.102-21.140.",
    ),
    # USB: the core, host and device banks are three tables at absolute offsets
    # 0x000, 0x400 and 0x800 from USB_BASE.
    "USBC": Spec(
        peripheral="USBC",
        type_name="USB_Type",
        group="SG2002_USB",
        title="USB core",
        table="21.154",
        instance="USB_REGS",
        base_macro="USB_BASE",
        see="SG2002 技术参考手册 v1.02, 表 21.154 起。SG2002 TRM v1.02, Table 21.154 onwards.",
    ),
    "Host": Spec(
        peripheral="USB host",
        type_name="USB_HOST_Type",
        group="SG2002_USB_HOST",
        title="USB host",
        table="21.197",
        instance="USB_HOST_REGS",
        base_macro="USB_BASE",
        base_offset=0x400,
        see="SG2002 技术参考手册 v1.02, 表 21.197 起。SG2002 TRM v1.02, Table 21.197 onwards.",
    ),
    "Device": Spec(
        peripheral="USB device",
        type_name="USB_DEVICE_Type",
        group="SG2002_USB_DEVICE",
        title="USB device",
        table="21.213",
        instance="USB_DEVICE_REGS",
        base_macro="USB_BASE",
        base_offset=0x800,
        see="SG2002 技术参考手册 v1.02, 表 21.213 起。SG2002 TRM v1.02, Table 21.213 onwards.",
    ),
    "CryptoDMA": Spec(
        peripheral="CryptoDMA",
        type_name="CRYPTO_DMA_Type",
        group="SG2002_CRYPTO_DMA",
        table="22.1",
        instance="CRYPTO_DMA_REGS",
        base_macro="CRYPTO_DMA_BASE",
        see="SG2002 技术参考手册 v1.02, 表 22.1-22.75。SG2002 TRM v1.02, Tables 22.1-22.75.",
    ),
    # The AIAO table is the I2S global block at 0x0410_8000, per its caption.
    "AIAO": Spec(
        peripheral="AIAO",
        type_name="I2S_GLOBAL_Type",
        group="SG2002_I2S_GLOBAL",
        title="I2S global (AIAO)",
        table="20.1",
        instance="I2S_GLOBAL_REGS",
        base_macro="I2S_GLOBAL_BASE",
        see="SG2002 技术参考手册 v1.02, 表 20.1-20.8。SG2002 TRM v1.02, Tables 20.1-20.8.",
    ),
    # One I2S/TDM block, four instances.
    "I2S_TDM": Spec(
        peripheral="I2S_TDM",
        type_name="I2S_Type",
        group="SG2002_I2S",
        title="I2S/TDM",
        table="20.2",
        instances=[("I2S0_REGS", "I2S0_BASE"), ("I2S1_REGS", "I2S1_BASE"),
                   ("I2S2_REGS", "I2S2_BASE"), ("I2S3_REGS", "I2S3_BASE")],
        see="SG2002 技术参考手册 v1.02, 表 20.2-20.30。SG2002 TRM v1.02, Tables 20.2-20.30.",
    ),
    # The TRM documents the codec registers but gives no base for them, and the
    # memory map has no codec entry, so no instance macro is emitted.
    "Audio DAC/ADC": Spec(
        peripheral="Audio codec",
        type_name="AUDIO_CODEC_Type",
        group="SG2002_AUDIO_CODEC",
        title="Audio codec",
        table="20.31",
        see="SG2002 技术参考手册 v1.02, 表 20.31 起。SG2002 TRM v1.02, Table 20.31 onwards.",
    ),
    "RTC_CORE_REG": Spec(
        peripheral="RTC_CORE",
        type_name="RTC_CORE_Type",
        group="SG2002_RTC_CORE",
        table="6.1",
        instance="RTC_CORE_REGS",
        base_macro="RTC_CORE_BASE",
        see="SG2002 技术参考手册 v1.02, 表 6.1 起。SG2002 TRM v1.02, Table 6.1 onwards.",
    ),
    "RTC_MACRO_REG": Spec(
        peripheral="RTC_MACRO",
        type_name="RTC_MACRO_Type",
        group="SG2002_RTC_MACRO",
        table="6.2",
        instance="RTC_MACRO_REGS",
        base_macro="RTC_MACRO_BASE",
        see="SG2002 技术参考手册 v1.02, 表 6.2。SG2002 TRM v1.02, Table 6.2.",
    ),
    # Group2 PLL, the counterpart of the already-migrated PLL_G6 block.
    "PLL_G2": Spec(
        peripheral="PLL_G2",
        type_name="PLL_G2_Type",
        group="SG2002_PLL_G2",
        table="8.5",
        instance="PLL_G2_REGS",
        base_macro="PLL_G2_BASE",
        see="SG2002 技术参考手册 v1.02, 表 8.5。SG2002 TRM v1.02, Table 8.5.",
    ),
}


def member_line(offset: int, name: str, description: str, indent: str = "    ") -> str:
    if name.startswith("RESERVED"):
        return (f"{indent}uint32_t {name};"
                f" ///< 偏移 0x{offset:03X}：未公开区域的占位空间。"
                f" Offset 0x{offset:03X}: unexposed padding.")
    # Three shapes, depending on what the TRM actually provides: bilingual when
    # the name yields a usable gloss, English-only when it does not, and the
    # bare offset when the TRM table gives no description at all.
    parts = []
    hint = chinese_hint(name)
    if hint:
        parts.append(f"偏移 0x{offset:03X}：{hint}。")
    if description:
        parts.append(f"Offset 0x{offset:03X}: {description}")
    return (f"{indent}volatile uint32_t {name}; ///< "
            + (" ".join(parts) if parts else f"Offset 0x{offset:03X}"))


def emit(overview: trm.Overview, spec: Spec) -> str:
    label = spec.title or spec.peripheral
    registers = sorted(overview.registers, key=lambda r: r.offset)
    for register in registers:
        register.name = screaming(spec.renames.get(register.offset, register.name))
        register.description = spec.descriptions.get(
            register.offset, clean_description(register.description))

    out: list[str] = []
    out.append("/**")
    out.append(f" * @defgroup {spec.group} {label} 寄存器 / {label} registers")
    out.append(" * @ingroup SG2002_DEVICE")
    out.append(f" * @see {spec.see}")
    out.append(" * @{")
    out.append(" */")
    out.append("")

    if spec.repeat:
        sub_type, count, stride, first, members_per = spec.repeat
        out.append("/**")
        out.append(f" * @brief {label} 单个通道的寄存器块。"
                   f" One {label} channel register block.")
        out.append(" */")
        out.append("typedef struct")
        out.append("{")
        for register in registers[:members_per]:
            out.append(member_line(register.offset - first, register.name, register.description))
        out.append(f"}} {sub_type};")
        out.append("")
        for register in registers[:members_per]:
            out.append(f"static_assert(offsetof({sub_type}, {register.name}) "
                       f"== 0x{register.offset - first:03X}U);")
        out.append("")

    out.append("/**")
    out.append(f" * @brief {label} 寄存器块。 {label} register block.")
    out.append(" */")
    out.append("typedef struct")
    out.append("{")
    if spec.repeat:
        sub_type, count, stride, first, members_per = spec.repeat
        end = first + stride * (count - 1) + 4 * members_per
        out.append(f"    {sub_type} CHANNEL[{count}];"
                   f" ///< 偏移 0x{first:03X}：通道寄存器数组，步长 0x{stride:X} 字节。"
                   f" Offset 0x{first:03X}: channel register array with a "
                   f"0x{stride:X}-byte stride.")
        rest = [r for r in registers if r.offset >= end]
        previous = end
    else:
        rest = registers
        previous = 0
    for register in rest:
        if register.offset > previous:
            words = (register.offset - previous) // 4
            if words:
                out.append(f"    uint32_t RESERVED_{previous:03X}[{words}];"
                           f" ///< 偏移 0x{previous:03X}：未公开区域的占位空间。"
                           f" Offset 0x{previous:03X}: unexposed padding.")
        out.append(member_line(register.offset, register.name, register.description))
        previous = register.offset + 4
    out.append(f"}} {spec.type_name};")
    out.append("")
    if spec.repeat:
        sub_type, count, stride, first, members_per = spec.repeat
        end = first + stride * (count - 1) + 4 * members_per
        rest = [r for r in registers if r.offset >= end]
    else:
        rest = registers
    for register in rest:
        out.append(f"static_assert(offsetof({spec.type_name}, {register.name}) "
                   f"== 0x{register.offset:03X}U);")
    out.append("")

    for macro, base in (spec.instances
                        or ([(spec.instance, spec.base_macro)]
                            if spec.instance and spec.base_macro else [])):
        out.append("/**")
        out.append(f" * @brief {macro} 寄存器实例指针。"
                   f" {macro} register-instance pointer.")
        out.append(" */")
        address = base if not spec.base_offset else f"({base} + 0x{spec.base_offset:X}UL)"
        out.append(f"#define {macro} (({spec.type_name} *)(uintptr_t){address})")
        out.append("")
    out.append("/** @} */")
    return "\n".join(out)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trm", type=Path, default=None)
    parser.add_argument("--peripheral", required=True)
    args = parser.parse_args()

    args.trm = args.trm or trm.default_trm()
    if not args.trm.is_file():
        parser.error(f"TRM text not found: {args.trm}; pass --trm or set SG2002_TRM")
    overviews = trm.find_overviews(trm.load(args.trm))
    spec = next((s for s in SPECS.values()
                 if args.peripheral.lower() in s.peripheral.lower()
                 or args.peripheral.lower() == (s.table or "")), None)
    if spec is None:
        parser.error(f"no structural spec for {args.peripheral!r}; add one to SPECS")
    if spec.table:
        match = next((o for o in overviews if o.table == spec.table), None)
    else:
        match = next((o for o in overviews
                      if args.peripheral.lower() in o.peripheral.lower()), None)
    if match is None:
        parser.error(f"no overview table matches {args.peripheral!r}")
    print(emit(match, spec))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
