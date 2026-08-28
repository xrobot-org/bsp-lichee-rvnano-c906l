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
For bring-up and repeated SPI/DMA tests, restart that instance explicitly from
an interactive root shell:

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

## RCC Validation

Set `SG200X_RCC_TEST=1` while building to run the SG200x C906L clock/reset
self-test. It checks invalid API values without MMIO side effects, live root
and planned rates, all managed mux/divider/gate fields, I2C1 reset release,
an SPI2 reset pulse, idempotent plan application, repair after deliberate SPI
divider/bypass corruption, and runtime-only C906 clock immutability. Two tasks
then make 20,000 RCC calls and require an observed low-to-high-priority
preemption while an RCC read or write transaction is active. Contending writes
return `BUSY`; the test blocks briefly before retrying so the preempted writer
can finish.

The test is intended for the board ownership used here: I2C1 and SPI2 belong
to C906L, and SPI2 has no active external client. Do not run its reset-pulse
stage while Linux or another firmware client owns SPI2. A successful Linux
`boot_trace` read reports `valid=1` and `event_arg=0x00000000`. Raw trace words
40 through 55 contain magic `0x52434331`, the error bitmap, completed-call
count, decoded rates, live RCC registers, the iteration at which the
higher-priority task preempted the lower-priority task, and the observed
contention count.

The final test firmware was verified on `maixcam-sc035hgs` on 2026-08-28 over
five consecutive remoteproc stop/start cycles. Every cycle completed 20,000
calls with no error bits; FPLL/AXI4/AXI6/SPI/I2C decoded as 1.5 GHz, 300 MHz,
100 MHz, 187.5 MHz, and 100 MHz respectively. The lock-free transaction test
observed 5 to 10 read/write conflicts per run while preserving those final
rates, proving that the high-priority task could preempt an in-flight RCC call.

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
written to boot-trace words 16 through 21. On a callback timeout, it also
captures the DMAC channel, TOP interrupt mux, PLIC, and machine-interrupt CSR
state in raw trace words 40 through 53 before clearing the channel; this makes
the external-interrupt path diagnosable without changing normal DMA behavior.
The timeout event's low 24 bits also expose a compact snapshot through the
Linux `boot_trace` sysfs attribute: bits 0..1 are `DMAC_CFG`, 2..9 are global
DMA interrupt status, 10..17 are channel interrupt status, and 18..23 are the
first pending C906L PLIC source. A value of 63 in bits 18..23 means none of the
port's 62 PLIC sources latched while the DMA completion remained asserted.

Set `SG200X_I2C_DMA_TEST=1` to build the board-specific I2C4 DMA smoke test.
It performs a one-byte read from the camera-board probe address `0x3d` and
publishes its result through the same boot trace. It is intentionally not a
generic device probe and does not write any I2C register.

For the SPI2 DMA loopback smoke test, connect the board header pins P22 (MOSI)
to P21 (MISO), and keep P23 (SCK) and P18 (CS) available to the SPI2
controller. Build with `SG200X_SPI2_TEST=1`; the test sends 64 bytes through
the DMA TX and RX channels and waits for the asynchronous completion callback
before comparing the buffers. A successful boot trace has
`event_name=irq` and `event_arg=0x00000040` (result 0, all 64 bytes matched).
An argument with the high byte set is a transfer or callback error; a low
24-bit value below 64 identifies the first mismatching byte. The test was
verified on the `maixcam-sc035hgs` image via `debian@10.42.0.1` on 2026-08-26.
The C906L `DEFAULT_C906L_CLOCK_PLAN` configures `CLK_SPI` from the board's
1.5 GHz FPLL input with divider 8, yielding a 187.5 MHz SSI input. All SPI0-3
share that root clock; the plan is defined and compile-time checked in
`driver/sg200x_clock_tree.hpp`, rather than being hidden in the SPI constructor
or test application.
After collecting the trace, restore the previous `/lib/firmware/c906-mcu.elf`
and perform a full system reboot so the media services are not left on the
smoke-test firmware.

For a repeatable SPI2 DMA stress run, use the same MOSI-to-MISO connection and
build with `SG200X_SPI2_STRESS_TEST=1`. The default is 4096 back-to-back DMA
transfers; set `SG200X_SPI2_STRESS_ITERATIONS` to select another positive
count. In PowerShell, for example:

```powershell
$env:SG200X_SPI2_STRESS_TEST = "1"
$env:SG200X_SPI2_STRESS_ITERATIONS = "10000"
cmake --build --preset firmware
```

Before the sustained loop, the test exercises all four CPOL/CPHA combinations,
1/3/8/17/64/127/512-byte frames, `ReadAndWrite`, zero-copy `Transfer`,
`Read`, `Write`, `MemRead`, `MemWrite`, BLOCK completion, invalid arguments,
and `BUSY` while an operation is active. It also runs a 64-byte Circular DMA
ring for the configured iteration count, validates every RX block callback,
stops the ring from its final ISR callback, and immediately runs a Normal DMA
transfer to prove both channels are reusable. An intentionally unaligned caller
buffer verifies that ordinary operations use the configured DMA staging
buffers, while `Transfer` directly exercises those internal buffers. The test
validates every loopback byte and the callback completion. A final `event_arg`
with high byte zero reports the completed iteration count. Result codes `1`,
`2`, `4`, and `5` mean data mismatch, driver/DMA error, timeout, and
API-contract/configuration failure, respectively. On failure the low 24 bits
pack the test stage, the low 10 bits of the iteration, and the byte/error
offset.

The C906L IRQ path was verified on `maixcam-sc035hgs` via
`debian@10.42.0.1` on 2026-08-27: the DMA memory-copy test completed through
PLIC IRQ 25 with `event_arg=0x00000100`, and the SPI2 loopback stress test
completed 4096 Circular blocks followed by the Normal functional suite and
4096-transfer stress loop with `event_arg=0x00001000`.

The SDMA controller at `0x04330000` is a single hardware instance shared by
Linux and C906L. `SG200XDMAC` owns channels 4 through 7 only; Linux retains
channels 0 through 3. First initialization never pulses the shared reset. It
retires stale C906L channels, removes only those channel interrupt routes from
the other CPUs, and routes them to the C906L CPU2 domain while preserving the
Linux partition. Both sides must keep that channel partition and must not
rewrite the other side's remap, enable, or interrupt bits.

## Atomic Operations

The C906 toolchain target includes the RISC-V A extension. A firmware build
with `SG200X_ATOMIC_TEST=1` runs two same-priority LibXR threads which contend
on a lock-free `std::atomic<uint32_t>`, and separately executes direct
`amoadd.w.aqrl` instructions. The boot-trace result is zero only when the
two contended counters reach `600000` and `65536`, respectively.

This does not make a cross-processor atomicity claim for C906L and Linux
shared DDR: the SG2002
shared-memory transport deliberately uses an uncached Linux mapping and C906L
cache invalidate/clean operations, so that region is not hardware cache
coherent. Use single-writer counters plus cache maintenance and release/acquire
barriers for that IPC protocol; do not use a C++ atomic RMW as its lock.
