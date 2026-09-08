#!/usr/bin/env python3
"""Create a portable-ish clangd database from the vendor GCC database.

The vendor SDK is built with a GCC cross compiler that clangd cannot use as a
frontend. This keeps all project flags and include directories, but changes the
driver to clang++, removes GCC-only options, adds the RISC-V target, and derives
the GCC C++ headers from the compiler path recorded by CMake.
"""

from __future__ import annotations

import json
import re
import shlex
import shutil
import subprocess
import sys
from pathlib import Path


def _compiler_path(command: str) -> Path:
    match = re.match(r'^\s*"([^"]+)"|^\s*([^\s]+)', command)
    if not match:
        raise ValueError(f"cannot find compiler in command: {command[:80]!r}")
    value = match.group(1) or match.group(2)
    compiler = Path(value)
    if not compiler.is_absolute():
        found = shutil.which(value)
        if found is None:
            raise ValueError(f"compiler is not on PATH: {value}")
        compiler = Path(found)
    return compiler.resolve()


def _version_key(path: Path) -> tuple[int, ...]:
    return tuple(int(part) for part in re.findall(r"\d+", path.name))


def _system_includes(compiler: Path, cxx: bool) -> list[Path]:
    root = compiler.parent.parent
    sysroot = root / "riscv-none-elf"
    gcc_versions = list((root / "lib" / "gcc" / "riscv-none-elf").glob("*"))
    if not gcc_versions:
        raise ValueError(f"GCC headers not found below {root / 'lib' / 'gcc'}")
    gcc = max(gcc_versions, key=_version_key)
    result: list[Path] = []
    if cxx:
        cxx_versions = list((sysroot / "include" / "c++").glob("*"))
        if not cxx_versions:
            raise ValueError(f"C++ headers not found below {sysroot}")
        cxx_headers = max(cxx_versions, key=_version_key)
        multilib = [
            path
            for path in cxx_headers.rglob("lp64d")
            if path.parent.name.startswith("rv64imafdc_zicsr")
        ]
        if len(multilib) != 1:
            raise ValueError(
                f"expected one RV64IMAFDC LP64D C++ include directory, got {multilib}"
            )
        result.append(cxx_headers)
        result.append(multilib[0])
        result.append(cxx_headers / "backward")
    result.extend([gcc / "include", gcc / "include-fixed", sysroot / "include"])
    missing = [path for path in result if not path.is_dir()]
    if missing:
        raise ValueError(f"missing compiler include directories: {missing}")
    return result


def _clang_command(command: str) -> str:
    compiler = _compiler_path(command)
    driver = "clang++" if "++" in compiler.name else "clang"
    includes = _system_includes(compiler, driver == "clang++")
    command = re.sub(
        r'^\s*(?:"[^"]+"|[^\s]+)', driver, command, count=1
    )
    command = re.sub(r"(?:^|\s)(?:-mcpu=thead-c906|--specs=nosys\.specs|-nostartfiles|-static|-Wl,--defsym=__dso_handle=0)(?=\s|$)", " ", command)
    command = f"{driver} --target=riscv64-unknown-elf" + command[len(driver) :]
    suffix = " ".join(f'-isystem "{path.as_posix()}"' for path in includes)
    return f"{command} {suffix}"


def main() -> int:
    if len(sys.argv) != 3:
        print(f"usage: {sys.argv[0]} INPUT OUTPUT", file=sys.stderr)
        return 2
    source, destination = map(Path, sys.argv[1:])
    entries = json.loads(source.read_text(encoding="utf-8"))
    for entry in entries:
        if "command" in entry:
            entry["command"] = _clang_command(entry["command"])
        elif "arguments" in entry:
            command = (
                subprocess.list2cmdline(entry["arguments"])
                if sys.platform == "win32"
                else shlex.join(entry["arguments"])
            )
            entry.pop("arguments")
            entry["command"] = _clang_command(command)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(entries, indent=2) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
