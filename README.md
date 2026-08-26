# SG200x C906L BSP

This is a C906 FreeRTOS BSP with an MCU-style source layout. It deliberately
does not carry a second FreeRTOS RISC-V port. Firmware startup, trap handling,
PLIC support, linker script, FreeRTOS configuration, and the RTOS tick are
provided by the pinned SG200x C906L SDK source used by Sophgo's Debian integration.

```text
Core/                         BSP-wide non-application code
User/                         product application entry points
libxr/                        LibXR Git submodule
driver/                       SG200x C906L platform drivers (part of this repo)
scripts/                      build integration and runtime deployment tools
sdk/sg200x/                   SDK pin, patches, overlay, and board config
```

`driver/` contains the C906L coprocessor platform drivers directly in this
repository; no separate driver submodule is required.

## Vendor SDK

`sdk/sg200x/sdk.env` pins the same SG200x SDK revision used by
`D:/Projects/sophgo-sg200x-debian/components/sg2002-ipc`. The SDK owns the
C906 port, including `freertos/cvitek/arch/riscv64/src/start.S`, the T-Head
C906 FreeRTOS extension, and `cv181x_lscript.ld`. The upstream repository is
maintained by Milk-V, but this BSP does not select a Milk-V board configuration:
it builds only the SG2002 C906L RTOS port with the LicheeRV Nano memory map.

The SDK directory is a read-only source input. The build extracts the pinned
tracked blobs into `build/sdk-worktree`, applies the BSP patches there, stages
the `xrobot` overlay there, and keeps all generated SDK build/install files in
that project-local directory. The source SDK is never used as a build output
directory and may be shared with other projects.

Provide a worktree at that exact revision on a normal Windows filesystem:

```powershell
git submodule update --init --recursive
git clone https://github.com/milkv-duo/duo-buildroot-sdk-v2.git C:\src\sg200x-c906l-sdk
cd C:\src\sg200x-c906l-sdk
git checkout 6f8962c394dd0a05729abb089f0feb7d5cc4aa5e
```

The build output is `build/firmware/c906-mcu.elf` and
`build/firmware/c906-mcu.bin`. The ELF is the development artifact: Linux
remoteproc loads it as `/lib/firmware/c906-mcu.elf`. The BIN is only for an
image builder that injects it into the FSBL/FIP boot path.

The project-owned changes applied during staging are limited to the patch
files under `sdk/sg200x/patches/` and the overlay under
`sdk/sg200x/overlay/`. Board configuration lives under
`sdk/sg200x/licheervnano/`. Do not edit generated files under
`build/sdk-worktree`; change the corresponding project patch or overlay
instead.

## Toolchain

Scoop is sufficient; MSYS2 is optional. Install CMake, Ninja, Python, Git,
MinGW (for `make`), and OpenSSH:

```powershell
scoop install cmake ninja python git mingw openssh
```

Scoop Git provides the Bash/POSIX commands used by the Vendor RTOS scripts.
Use `usr\bin\bash.exe` from that Git installation, not
`C:\Windows\System32\bash.exe`. Install the official Windows xPack GCC 15.2
package:

<https://github.com/xpack-dev-tools/riscv-none-elf-gcc-xpack/releases/download/v15.2.0-1/xpack-riscv-none-elf-gcc-15.2.0-1-win32-x64.zip>

The compiler must accept `-mcpu=thead-c906`; older vendor GCC packages do not.
Put the xPack `bin` directory on `PATH`, or set the compiler prefix
explicitly. The native build exports
`build/clangd/compile_commands.json` directly, so clangd uses the same Windows
paths and flags as the firmware build.

`CMakeUserPresets.json` is intentionally Git-ignored and contains this
workstation's SDK, Bash, compiler, and board paths. Configure and build from
PowerShell or the CMake Tools panel:

```powershell
cmake --preset c906
cmake --build --preset firmware
```

The `deploy` target builds the ELF and sends it over
Windows OpenSSH, then invokes `/usr/bin/rtos-mode` on the board. No additional
CLI or Linux compatibility layer is required.

## Runtime Deployment

The target must already run an image containing the C906 remoteproc device
tree, `cvitek_remoteproc`, `cvitek_mailbox`, and `/usr/bin/rtos-mode` from the
Sophgo Debian integration. No SD/eMMC reflash is needed for an iteration.

Configure once and build/deploy over Windows OpenSSH:

```powershell
cmake --preset c906
cmake --build --preset firmware
cmake --build --preset deploy
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
`driver/README.md`.

The application keeps the LED task by default. Set `SG200X_TIMEBASE_TEST=1`
when building to run the 1024-sample CSR/API monotonicity test, or set
`SG200X_PREEMPT_TEST=1` to run the 4096-sample higher-priority notification
latency benchmark. Both modes publish their result through the existing
remoteproc boot-trace shared page.

Set `SG200X_DMA_TEST=1` to build an isolated DMAC memory-copy test. It does
not access I2C or SPI pins; completion, data-integrity, and timeout status are
written to boot-trace words 16 through 21.

Set `SG200X_I2C_DMA_TEST=1` to build the board-specific I2C4 DMA smoke test.
It performs a one-byte read from the camera-board probe address `0x3d` and
publishes its result through the same boot trace. It is intentionally not a
generic device probe and does not write any I2C register.

## Atomic Operations

The C906 toolchain target includes the RISC-V A extension. A firmware build
with `SG200X_ATOMIC_TEST=1` runs two same-priority LibXR threads which contend
on a lock-free `std::atomic<uint32_t>`, and separately executes direct
`amoadd.w.aqrl` instructions. The boot-trace result is zero only when the
two contended counters reach `600000` and `65536`, respectively.

For the Linux C906 core, build and run the independent scheduler-preemption
stress test on the board:

```sh
gcc -O2 -std=c11 -pthread tools/sg200x_atomic_stress.c -o atomic-stress
./atomic-stress 4 2000000
```

This demonstrates local C906 atomic RMW behavior. It does not make a
cross-processor atomicity claim for C906L and Linux shared DDR: the SG2002
shared-memory transport deliberately uses an uncached Linux mapping and C906L
cache invalidate/clean operations, so that region is not hardware cache
coherent. Use single-writer counters plus cache maintenance and release/acquire
barriers for that IPC protocol; do not use a C++ atomic RMW as its lock.
