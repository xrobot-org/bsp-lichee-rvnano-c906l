# SG200x C906 BSP

This is a C906 FreeRTOS BSP with an MCU-style source layout. It deliberately
does not carry a second FreeRTOS RISC-V port. Firmware startup, trap handling,
PLIC support, linker script, FreeRTOS configuration, and the RTOS tick are
provided by the pinned Milk-V Duo SDK used by Sophgo's Debian integration.

```text
Core/                         BSP-wide non-application code
User/                         product application entry points
libxr/                        LibXR Git submodule
sg200x-c906-xr-driver/        SG200x LibXR platform-driver repository
scripts/                      build integration and runtime deployment tools
```

`sg200x-c906-xr-driver/` is intended to be a separate Git submodule. Its
remote URL is intentionally not guessed here; add it to `.gitmodules` once the
driver repository is published.

## Vendor SDK

`scripts/duo-sdk/sdk.env` pins the same Duo SDK revision used by
`D:/Projects/sophgo-sg200x-debian/components/sg2002-ipc`. The SDK owns the
C906 port, including `freertos/cvitek/arch/riscv64/src/start.S`, the T-Head
C906 FreeRTOS extension, and `cv181x_lscript.ld`.

For WSL, run the build against a dedicated worktree at that exact revision:

```sh
git submodule update --init --recursive
git clone https://github.com/milkv-duo/duo-buildroot-sdk-v2.git ~/duo-sdk-c906
cd ~/duo-sdk-c906
git checkout 6f8962c394dd0a05729abb089f0feb7d5cc4aa5e

cd /mnt/d/Dev/active/bsp_sophgo_c906
SG200X_DUO_SDK_DIR=~/duo-sdk-c906 \
  bash scripts/duo-sdk/build.sh
```

The build output is `build/firmware/c906-mcu.elf` and
`build/firmware/c906-mcu.bin`. The ELF is the development artifact: Linux
remoteproc loads it as `/lib/firmware/c906-mcu.elf`. The BIN is only for an
image builder that injects it into the FSBL/FIP boot path.

`build.sh` reverts and reapplies only its own Vendor patch before staging the
overlay. Keep the SDK worktree dedicated to this BSP and do not make unrelated
Vendor changes in it.

## Editor Support

Each `firmware` build exports the actual Vendor task build database to
`build/clangd/compile_commands.json`. Run clangd on the same host that produced
the database: WSL clangd for a WSL build, or Windows clangd for a native Windows
build. It reads the C906 flags, generated SDK headers, LibXR sources, and driver
sources from this database. The included VS Code setting lets clangd query the
RISC-V GCC driver for its system include paths. For WSL, install clangd and a
RISC-V GCC in the distribution used by VS Code:

```sh
sudo apt update
sudo apt install clangd gcc-riscv64-unknown-elf
```

The firmware build defaults to the pinned xPack GCC 15.2 toolchain, because
the current LibXR sources require C++20 features that the SDK's legacy GCC
10.2 cannot compile. Install it in WSL at:

```text
~/toolchains/xpack-riscv-none-elf-gcc-15.2.0-1/bin/riscv-none-elf-
```

The `SG2002 C906 (WSL, GCC 15)` configure preset and `firmware-gcc15` build
preset select that prefix without modifying `PATH`. `GCC 14 fallback` remains
available for bisecting toolchain regressions. To use another location,
set `SG200X_CROSS_COMPILE` when invoking `cmake` or edit the preset. The
distribution GCC is used only by clangd's `--query-driver` to discover the
compatible freestanding system headers.

In CMake Tools select the `SG2002 C906 (WSL, GCC 15)` configure preset, then
build the `firmware-gcc15` preset once. The root CMake project intentionally has `LANGUAGES
NONE`, so CMake Tools does not select a compiler: it starts the SDK build, and
the SDK selects the configured `riscv-none-elf-gcc/g++` prefix. The database is copied even
after a later compile failure, provided the SDK CMake configuration completed;
this keeps clangd usable while fixing source errors.

## Native Windows Build

Windows can build the C906 firmware from VS Code natively: CMake, Ninja,
clangd, the Vendor SDK scripts, and the compiler are Windows processes. Git
Bash provides the POSIX shell expected by the SDK. The installed WCH
`riscv-none-embed-gcc/g++` is suitable: it has a RV32 default, but it supports
and has been verified to emit RV64 ELF objects with
`-march=rv64imafdc -mabi=lp64d`.

The local `CMakeUserPresets.json` uses its absolute executable path, so adding
the toolchain to `PATH` is unnecessary. Its current location is:

```text
C:\MounRiver\MounRiver_Studio2\resources\app\resources\win32\components\WCH\Toolchain\RISC-V Embedded GCC\bin
```

Verify both commands exist and accept the target flags:

```powershell
'int main(void) { return 0; }' | riscv64-unknown-elf-gcc -march=rv64imafdc -mabi=lp64d -x c -S -o NUL -
'int main() { return 0; }' | riscv64-unknown-elf-g++ -march=rv64imafdc -mabi=lp64d -x c++ -S -o NUL -
```

The `SG2002 C906 (Windows)` preset selects the matching
`riscv-none-embed-` prefix. A toolchain with another prefix is supported by
changing `SG200X_CROSS_COMPILE`, including its trailing hyphen. Vendor sources
contain Windows-reserved `aux.*` paths, so the SDK worktree must live on the WSL
ext4 filesystem even though compilation uses Windows tools. Create it once:

```sh
wsl -d Ubuntu-26.04 bash -lc '
  git clone --depth 1 --filter=blob:none --sparse \
    https://github.com/milkv-duo/duo-buildroot-sdk-v2.git ~/duo-sdk-c906 &&
  git -C ~/duo-sdk-c906 sparse-checkout set build freertos/cvitek
'
```

`CMakeUserPresets.json` is intentionally Git-ignored and contains the local
absolute paths for Git Bash and the WCH compiler. Its SDK path is the Windows
UNC form `//wsl.localhost/Ubuntu-26.04/home/keruth/duo-sdk-c906`.
Select `SG2002 C906 (Windows Local)` in CMake Tools, or run:

```powershell
cmake --preset c906-windows-local
cmake --build --preset firmware-windows-local
```

This deliberately uses
`build/windows`, separate from the WSL `build` cache. clangd continues to read
`build/clangd/compile_commands.json`, which the firmware build exports from the
actual Vendor task project. Use the Windows clangd extension in this mode; its
configuration permits both Unix-style and `.exe` RISC-V GCC driver names.

## Runtime Deployment

The target must already run an image containing the C906 remoteproc device
tree, `cvitek_remoteproc`, `cvitek_mailbox`, and `/usr/bin/rtos-mode` from the
Sophgo Debian integration. No SD/eMMC reflash is needed for an iteration.

From WSL, configure once and then build/deploy over SSH:

```sh
cmake -S . -B build -DSG200X_DUO_SDK_DIR=~/duo-sdk-c906
cmake --build build --target firmware
SG200X_TARGET=root@192.168.x.x cmake --build build --target deploy
```

`deploy` uploads the ELF to a temporary board file, installs it as
`/lib/firmware/c906-mcu.elf`, and invokes `rtos-mode remoteproc`. This stops
and restarts the C906 remote processor and can interrupt the media stack.

## Timebase

The LibXR timebase reads the C906 `rdtime` CSR and uses the documented 25 MHz
timebase frequency. Do not read `0x7400BFF8` as a standard CLINT `mtime`: the
SG2002 TRM marks that address range reserved, the SDK undefines non-QEMU
`CLINT_MTIME`, and a board test showed that the MMIO access can hang. UART,
I2C, and SPI driver work is DMA-only as specified in
`sg200x-c906-xr-driver/README.md`.

The application keeps the LED task by default. Set `SG200X_TIMEBASE_TEST=1`
when building to run the 1024-sample CSR/API monotonicity test, or set
`SG200X_PREEMPT_TEST=1` to run the 4096-sample higher-priority notification
latency benchmark. Both modes publish their result through the existing
remoteproc boot-trace shared page.
