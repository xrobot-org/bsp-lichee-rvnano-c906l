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

The platform drivers depend on the C23 `sg200x-ll` submodule. Initialize both
`libxr` and `sg200x-ll` with `git submodule update --init --recursive`. Peripheral
register definitions and low-level operations belong to SG200X LL; LibXR adapters
and clock/resource policy belong to `driver/`.

The SDK overlay accepts an application-owned `User/CMakeLists.txt` that adds
sources to the existing `xrobot` target. Without that manifest, it builds the
unchanged `User/main.cpp` and `User/platform.cpp` template entry points. Product
firmware can maintain its application in a separate checkout while sharing
these drivers and SDK integration.

## Vendor SDK

`sdk/sg200x/sdk.env` pins the SG200x SDK revision used by this BSP. The SDK
owns the C906 port, including `freertos/cvitek/arch/riscv64/src/start.S`, the
T-Head C906 FreeRTOS extension, and `cv181x_lscript.ld`. The upstream repository
is maintained by Milk-V, but this BSP does not select a Milk-V board
configuration: it builds only the SG2002 C906L RTOS port with the LicheeRV Nano
memory map.

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

The `deploy` target builds the ELF and sends it over SSH. The reference image
documents `debian`/`rv`, so that target uses `sshpass` with an environment-only
password and supplies the same sudo credential via standard input. Credentials
never appear in the SSH command line. Other targets may use normal SSH keys;
set `SG200X_SSH_PASSWORD` and `SG200X_SUDO_PASSWORD` explicitly when they use
password authentication.

## Runtime Deployment

The target must already run an image containing the C906 remoteproc device
tree, `cvitek_remoteproc`, `cvitek_mailbox`, and `/usr/bin/rtos-mode` from the
Sophgo Debian integration. No SD/eMMC reflash is needed for an iteration.

The reference board used during bring-up is reachable as `debian@10.42.0.1`:

```powershell
sshpass -p rv ssh debian@10.42.0.1
```

The deploy script passes its password through `SSHPASS`, not the `-p` argument
shown for this interactive login example.

Configure once and build/deploy over Windows OpenSSH:

```powershell
cmake --preset c906
cmake --build --preset firmware
cmake --build --preset deploy
```

`deploy` uploads the ELF to a temporary board file, installs it as
`/lib/firmware/c906-mcu.elf`, then restarts the C906 processor. On the
reference image the C906 remote processor
appears as `/sys/class/remoteproc/remoteproc0` with name `cv181x-c906_1`.
Before C906L starts, the deployment applies the board ownership declarations
from `sdk/sg200x/licheervnano/deploy.conf`. I2C1 (`4010000.i2c`) is unbound
from Linux there because Linux and C906L cannot safely service interrupts from
the same DesignWare controller. Other boards can select a different file with
`SG200X_BOARD_DEPLOY_CONFIG`; individual deployments may override
`SG200X_EXCLUSIVE_PLATFORM_DEVICES` with whitespace-separated
`driver:device` entries.
Set `SG200X_DMA_INT_MUX=0x0007FC00` when restoring the reference image after
an SDMA routing experiment; the deployment sequence stops C906, restores that
TOP routing register, replaces the firmware, and starts C906 again.
To restart that instance explicitly, use an interactive root shell:

```sh
cat /sys/class/remoteproc/remoteproc0/name
cat /sys/class/remoteproc/remoteproc0/state
echo stop > /sys/class/remoteproc/remoteproc0/state
echo start > /sys/class/remoteproc/remoteproc0/state
cat /sys/class/remoteproc/remoteproc0/state
```

The generic `/usr/bin/rtos-mode remoteproc` helper is image-dependent and was
not reliable on the reference image; the direct `remoteproc0` sysfs sequence
was the verified recovery path. Stopping and restarting C906 interrupts any
running media or mailbox users.

## Timebase

The LibXR timebase reads the C906 `rdtime` CSR and uses the documented 25 MHz
timebase frequency. Do not read `0x7400BFF8` as a standard CLINT `mtime`: the
SG2002 TRM marks that address range reserved, the SDK undefines non-QEMU
`CLINT_MTIME`, and a board test showed that the MMIO access can hang. UART,
I2C, and SPI driver work is DMA-only as specified in `driver/README.md`.

The C906L `DEFAULT_C906L_CLOCK_PLAN` configures `CLK_SPI` from the board's
1.5 GHz FPLL input with divider 8, yielding a 187.5 MHz SSI input. All SPI0-3
share that root clock. `driver/sg200x_clock_tree.hpp` keeps the graph and
static correctness checks; its value-parameter planner also accepts runtime
rates. `SG200XRCC` applies plans and controls gates at runtime while preserving
the boot-owned PLL/CPU clock contract. Explicit rate changes survive later
peripheral preparation.

## DMA Ownership

The SDMA controller at `0x04330000` is a single hardware instance shared by
Linux and C906L. `SG200XDMAC` owns channels 4 through 7 only; Linux retains
channels 0 through 3. First initialization never pulses the shared reset. It
retires stale C906L channels, removes only those channel interrupt routes from
the other CPUs, and routes them to the C906L CPU2 domain while preserving the
Linux partition. Both sides must keep that channel partition and must not
rewrite the other side's remap, enable, or interrupt bits.

The main branch contains only the LED blink application. Board stress and
validation firmware is retained separately in the `codex/stress-tests` branch.
