# SG200x C906L LibXR Driver

`driver/` contains the SG200x C906L platform drivers as part of this BSP
repository. Keeping the sources in-tree makes firmware and driver changes
versioned and reviewed together.

These sources provide the SG200x-specific implementation of the LibXR hardware
interfaces. They are not a fork of `libxr`.

## Scope

The driver owns C906-side support for the peripherals needed by LibXR, such as:

- GPIO and pin configuration required by the board
- UART
- timer and timebase
- I2C
- SPI
- PWM
- SARADC
- SG200x system DMA integration

## Reference Layers

The register definitions and behavior in this repository are cross-checked
against these public implementations:

- [Milk-V duo-buildroot-sdk](https://github.com/milkv-duo/duo-buildroot-sdk/tree/develop/freertos/cvitek):
  CVITEK FreeRTOS `hal/cv181x` implementations for SARADC, DesignWare I2C,
  PWM, SPI, UART, and pinmux, plus the small `driver` wrappers exposed to RTOS
  tasks.
- [Sophgo SG200x Debian BSP](https://github.com/Fishwaldo/sophgo-sg200x-debian):
  Linux clock/reset, IRQ, pinctrl, and peripheral drivers. These are useful for
  cross-checking hardware ownership, but are not directly linkable from C906L
  FreeRTOS.
- [sg200x-bare](https://github.com/0x754C/sg200x-bare): independent C bare-metal
  startup, GPIO, UART, and register definitions.
- [sg200x-bsp](https://github.com/yfblock/sg200x-bsp): community Rust `no_std`
  BSP used to compare instance maps and higher-level I2C/PWM/DMA semantics.

The C906L driver deliberately keeps its own LibXR-facing API. The vendor HAL
is a useful register-level reference, but it is not a complete installable
SG200x driver library: some modules are present in the source tree without
being enabled by the default RISC-V CMake lists, and its SPI/I2C paths do not
provide the DMA and asynchronous completion guarantees required here.

Each driver keeps the SoC addresses, register offsets, and bit definitions it
uses in its own header, so the hardware contract is visible beside the code
that consumes it. `sg200x_mmio.hpp` only centralizes the volatile MMIO access
cast; it does not hide chip-specific register knowledge.

DMA is the only I2C/SPI data path in this platform driver. `SG200XDMAC` uses a
static partition of the SG200x eight-channel DesignWare AXI DMA controller at
`0x04330000`: Linux owns channels 0-3 and C906L owns channels 4-7. It controls
only its partition's Top interrupt-mux fields, peripheral remap fields, channel
registers, descriptors, and C906 D-cache maintenance. The Linux kernel/DT must
also restrict `dw_dmac` allocation to channels 0-3; interrupt routing alone
does not prevent a Linux client from programming an unreserved channel.

`sg200x_clock_tree.hpp` is the compile-time model of the C906-visible portion
of the actual CV181x/SG200x TOP provider: the 25 MHz oscillator, PLL roots,
C906 cores, AXI4/AXI6 buses, RTC/ADC/timer branches, and DMA/I2C/PWM/SPI
leaves. Its `ClockId` values match the DT binding; its descriptors retain the
real gate, bypass, mux, divider, and reset-controller fields. `MakeRatePlan`
is a `consteval` solver for a C906L-configurable mux/divider branch. The
reference `DEFAULT_C906L_CLOCK_PLAN` fixes the board's FPLL input at 1.5 GHz
and derives AXI4 300 MHz, AXI6/I2C 100 MHz, 1 MHz, and SPI 187.5 MHz from it.
`SG200XRCC` validates that FPLL contract from the hardware CSR before applying
only the branches required by a prepared peripheral. It never retunes a PLL or
the clock of the running C906L core. A rate that traverses an as-yet undecoded
fractional G2 PLL still returns zero rather than a plausible-looking guess.
Clock/reset updates use a lock-free, fail-fast writer transaction and return
`BUSY` on overlap; they are startup/thread control-plane calls, not ISR APIs.
Live rate decoding never waits and uses a sequence check to reject an
intermediate hardware state with a zero result. No RCC operation disables
machine interrupts.
SPI is only one consumer of this provider; DMA and SARADC use the same resource
mapping already.
Each submitted transfer claims a C906L-owned DMA channel and programs its
documented peripheral request ID. First initialization gracefully disables or
aborts stale channels 4-7 without resetting the shared controller or touching
Linux channels 0-3. Completion and error signals for channels 4-7 are removed
from the other CPU routes and enabled through the TOP SDMA CPU2 mux field at
`0x03000298` to the C906L PLIC source 25. The driver
enables controller and per-channel interrupt signaling, registers that source
through the SDK `request_irq()` API, and enables `mie.MEIE` only after the
handler is installed. The handler consumes only latched DMA interrupt status;
there is no completion polling fallback. `Mode::NORMAL` detaches its callback
record after one completion. `Mode::CIRCULAR` uses a self-linked LLI, keeps the
record armed, invalidates RX memory, and dispatches one callback for every
completed block until explicit abort. Stop waits for hardware `CHEN` to clear
before returning the channel to the allocation pool. Calls that need to claim
DMA resources are rejected from ISR context.
GPIO, timer, and PWM do not use DMA, but they still share the platform clock,
reset, pinmux, and interrupt requirements.

## Watchdog

`SG200XWatchdog` uses the DesignWare watchdog instances documented at
`0x03010000`, `0x03011000`, and `0x03012000`. C906L firmware should use
`Instance::WDT2`; its documented interrupt is PLIC IRQ 39, although the driver
selects a CPU-reset response by default, so a timeout resets the C906L CPU while
Linux remains running. `ResetTarget::SYSTEM` can be selected when a full system
reset is required. `ResponseMode::INTERRUPT_THEN_RESET` enables the DesignWare
first-timeout interrupt; an ISR must clear/feed the watchdog or the next timeout
will perform the configured reset. The timeout is quantized to the first hardware TOP value whose
`2^(16 + TOP)` clock periods are at least the requested timeout. The default
clock is the 25 MHz XTAL; pass `32768u` to use the alternate 32 kHz watchdog
clock.

As with STM32 IWDG, starting the watchdog is one-way until the relevant CPU or
system reset.
`Stop()` only disables LibXR's automatic feed flag and returns
`ErrorCode::NOT_SUPPORT`; it cannot clear the hardware enable bit. Keep the
watchdog out of default bring-up firmware until the feed path and reset
handling have been deliberately tested.

## SARADC

`SG200XADC` implements the SG200x 12-bit SARADC in both hardware domains:
logical channels 1 through 3 use `0x030F0000`, while channels 4 through 6
(`PWR_ADC1` through `PWR_ADC3`) use `0x0502C000` and are remapped to local
channels 1 through 3. Each `ReadRaw()` operation selects one channel, triggers
a single conversion, checks the result-valid bit, and polls with a bounded
timeout. A local atomic transaction guard prevents concurrent tasks from
reprogramming the shared SARADC. The peripheral has no documented DMA sample
stream, so this driver intentionally uses polling rather than the SG200x DMA
controller. The default timing uses a conservative divider of 15 from the
25 MHz XTAL and the internal reference; pass `Reference::EXTERNAL_VDD18A` and
the board's measured reference voltage when the external reference is used.

## I2C and SPI Bring-up

`SG200XI2C` implements I2C0 through I2C4 as a DesignWare APB I2C master. Its
controller ID selects the RCC peripheral resources; the constructor prepares
the gates/reset and reads the live `CLK_I2C` rate from `SG200XRCC`. Standard-
and fast-mode timing counts are derived from that rate using the DesignWare
compensation formula and validated against the register widths. It supports
7-bit and 10-bit targets and repeated-start register reads. The constructor
requires caller-owned, cache-line-aligned DMA command and receive staging
buffers whose capacities are whole cache-line multiples: `IC_DATA_CMD`
includes read, restart, and stop command bits,
so raw user bytes cannot be written directly by DMA. STOP_DET or TX_ABRT IRQ
is combined with DMA completion before the LibXR operation completes. Invalid
arguments, zero-length operations, active-operation `BUSY`, block timeouts,
NACK cleanup/recovery, and BLOCK/CALLBACK/POLLING completion follow the same
LibXR operation contract used by the STM32 drivers. The submitted operation is
copied into the driver before hardware can complete, so asynchronous calls do
not retain a pointer to the caller's operation wrapper. One live object may
own each controller; the permanent SDK IRQ entry dispatches through a
lifecycle-managed per-controller instance table rather than retaining an
object address. The dispatch pointer is published and removed atomically, so
normal transfers, IRQ registration, and teardown do not disable machine
interrupts. A claim left in service by the previous remoteproc image is retired
after the initialized instance has been published, so an immediately arriving
IRQ can use the normal lock-free dispatch path. I2C transfers always use Normal
DMA. Circular DMA is not an
I2C transaction mode in STM32I2C and
cannot autonomously repeat address, START/STOP, NACK, or arbitration handling;
continuous sensor acquisition should schedule repeated Normal transactions.

`SG200XSPI` implements SPI0 through SPI3 as a DesignWare SSI master using
8-bit Motorola SPI frames. Its constructor accepts a controller ID rather than
a clock rate; it prepares the modelled SPI/APB gates and reads the actual
configured `CLK_SPI` rate from `SG200XRCC`, so the LibXR maximum is never a
hard-coded 187.5 MHz assumption. BAUDR only
accepts an even divisor between 2 and 65534, so the driver reports
`ssi_clk / 2` as the LibXR maximum and maps `DIV_1` to that fastest legal
hardware setting. The driver owns one hardware slave-select bit per instance.
Like `STM32SPI`, its constructor takes cache-line-aligned RX/TX DMA buffers
whose capacities are whole cache-line multiples.
`ReadAndWrite`, `Read`, `Write`, `MemRead`, and `MemWrite` stage arbitrary
caller buffers through that storage; `Transfer(size)` submits the active
internal buffers directly and switches them on successful completion.
`MemRead` and
`MemWrite` follow LibXR's established 8-bit register convention (read: bit 7
set; write: bit 7 clear); devices with a different wire protocol should use
`ReadAndWrite` directly. Zero-length memory accesses still place their command
byte on the bus, matching the STM32SPI abstraction.

SPI transfers use paired RX/TX DMA channels for every non-empty operation,
including read-only and write-only calls, so the controller FIFO cannot stall.
The instance reports `BUSY` while a transfer or its completion cleanup owns
the hardware. DMA start failures, block-operation timeouts, and SSI errors
disable the request paths and release both DMA channels before the instance is
made available again. Successful DMA completion also waits for SSI `BUSY` to
drop before deasserting chip select, preserving the final frame and CS hold
time. DMA ownership is task-only; calls with `in_isr=true` return
`ErrorCode::NOT_SUPPORT`.

`StartCircularTransfer(size, callback)` is the STM32-style Circular extension
for the active internal DMA buffers. It is full duplex and zero copy, accepts
callback operations only, and reports each completed RX block from ISR
context. The callback may call `StopCircularTransfer(true)`; stop retires TX
before RX while SSI handshakes are still live, then disables SSI and releases
both channels. BLOCK and POLLING Circular starts return `NOT_SUPPORT`, and a
second start or any ordinary SPI operation returns `BUSY` until the ring is
stopped. DMA or SSI errors shut down the ring and report one failed callback.

The implementation uses the SG200x TRM for register layout and interrupt
numbers. Board pin assignments, clock/reset ownership, and firmware loading
are supplied by the board firmware integration.

The timebase uses the C906 `rdtime` CSR. The SG200x firmware SDK explicitly
undefines any non-QEMU `CLINT_MTIME` MMIO address, and the TRM marks the
`0x30000000-0x7fffffff` range containing `0x74000000` as reserved. The board
DTS's generic `riscv,clint0` node must therefore not be interpreted as proof
that the standard SiFive `mtime` register exists at `0x7400BFF8`. The firmware
device tree documents `timebase-frequency = 25000000`; the timebase constructor
accepts only that CSR frequency and does not expose an unusable CLINT address.
The documented SG200x peripheral timer block at
`0x030A0000` remains available for scheduler or application timer interrupts,
but is not used as the timestamp source.

## GPIO

`SG200XGPIO` implements the four active-domain DesignWare GPIO controllers:
GPIO0 through GPIO3 at `0x03020000` through `0x03023000`. Each object owns one
bit of a 32-bit GPIO port. The normal C++ interface uses a strongly typed pin
descriptor owned by `SG200XGPIO`:

```cpp
SG200XGPIO led(SG200XGPIO::Bank::A, 14u);
```

`Bank::A`, `Bank::B`, `Bank::C`, and `Bank::D` map to GPIO0, GPIO1, GPIO2,
and GPIO3. The driver applies the SG200x GPIO function for its known
SoC-level pad mappings, including `GPIOA_14` at PINMUX offset `0x38`. Any
board-specific peripheral muxing remains startup-firmware state.

The driver supports input, push-pull output, software-emulated open-drain
output, and rising or falling edge interrupts. The C906 SDK PLIC IRQ numbers
are 41 through 44 for GPIO0 through GPIO3. `EnableInterrupt()` registers the
controller interrupt with the SDK `request_irq()` API and dispatches registered
LibXR callbacks for all pending pins in that controller. A pin has unique
object ownership; destruction masks its interrupt and removes it from the
dispatch table. Instance publication, interrupt-enabled state, and first IRQ
registration are lock-free atomics; a concurrent registration returns `BUSY`
instead of waiting, and the ISR never takes a lock.

The public SG200x C906 pinmux register description does not define pad-bias
fields. To avoid silently applying an incorrect electrical configuration,
`Pull::UP` and `Pull::DOWN` return `ErrorCode::NOT_SUPPORT`. The DesignWare
GPIO block provides one edge-polarity bit per pin, so
`FALL_RISING_INTERRUPT` likewise returns `ErrorCode::NOT_SUPPORT`.

## PWM

`SG200XPWM` maps the four SG200x PWM controllers at `0x03060000`,
`0x03061000`, `0x03062000`, and `0x03063000` to global channels `PWM0` through
`PWM15`. Each controller has four channels. The implementation follows the
TRM register sequence: `PERIOD` must be greater than `HLPERIOD`, continuous
output is started by clearing and then setting `PWMSTART[n]`, and an active
waveform update writes `PWMUPDATE[n]` as a one-then-zero strobe. `PWM_OE[n]`
is disabled before stopping a channel so a stopped output is not left driven.

The default clock argument is 100 MHz, matching the 100 MHz PWM operating
case documented in the TRM. The clock tree can also produce the documented
148.5 MHz case, so board code must pass the actual `clk_pwm` rate when it is
different from the default. The driver rejects frequencies that cannot be
represented by the 30-bit period counter and clamps the two exact duty-cycle
endpoints to one clock tick because the TRM requires `PERIOD > HLPERIOD`.
`SetPolarity()` exposes the TRM `POLARITY[n]` bit and must be called while the
channel is stopped.

Pin multiplexing is intentionally explicit. The SDK documents
`PWM0_BUCK` at pinmux offset `0xEC`, function `0`; it can be supplied as
`SG200XPWM::PWM0_BUCK_PINMUX`. Other PWM functions are available on board-
dependent SD, UART, ADC, camera, and Ethernet pads and must be selected by
the board integration. `PWM0_BUCK` is a power-regulator control net on the
 reference package, not a motor-power output.

```cpp
// The board startup code has already muxed the selected pad to PWM8.
SG200XPWM pwm(SG200XPWM::Channel::PWM8, 100000000u);
pwm.SetConfig({20000u});
pwm.SetDutyCycle(0.5f);
pwm.Enable();
```

The PWM block provides continuous output, finite pulse-count mode, and a
four-channel phase-shift mode. It does not expose complementary outputs,
programmable dead time, shoot-through protection, fault shutdown, current
limiting, or gate-drive power. Therefore `SG200XPWM` is suitable for feeding
an external motor/half-bridge driver logic input; it must not be connected
directly to a motor or used as a substitute for a protected motor-control
timer.
