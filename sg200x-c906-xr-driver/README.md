# SG200x C906 LibXR Driver

`sg200x-c906-xr-driver` is an independently maintained driver repository for
the SG200x C906 core. It is intended to be checked out in the parent BSP as a
Git submodule at this path.

This repository provides the SG200x-specific implementation of the LibXR
hardware interfaces. It is not a fork of `libxr`, and it is not a subtree of
the SG200x firmware repository.

## Scope

The driver owns C906-side support for the peripherals needed by LibXR, such as:

- GPIO and pin configuration required by the board
- UART
- timer and timebase
- I2C
- SPI
- PWM
- SG200x system DMA integration

DMA is a required part of this platform driver, not an optional acceleration
path. UART RX/TX and I2C/SPI data transfers are designed around DMA channels,
DMA completion interrupts, and DMA-safe buffers. Implementations must not add a
polling-only fallback merely for short transfers; the transfer state machine,
error handling, and callback behavior should remain the same for all lengths.

The DMA layer must define SG200x channel/request routing, buffer ownership,
alignment constraints, and cache clean/invalidate operations before an I2C, SPI,
or UART driver is considered complete. GPIO, timer, and PWM do not use DMA, but
they still share the platform clock, reset, pinmux, and interrupt requirements.

The implementation uses the SG200x TRM for register layout and interrupt
numbers. Board pin assignments, clock/reset ownership, and firmware loading
are supplied by the board firmware integration.

The timebase uses the C906 `rdtime` CSR. The SG200x firmware SDK explicitly
undefines any non-QEMU `CLINT_MTIME` MMIO address, and the TRM marks the
`0x30000000-0x7fffffff` range containing `0x74000000` as reserved. The board
DTS's generic `riscv,clint0` node must therefore not be interpreted as proof
that the standard SiFive `mtime` register exists at `0x7400BFF8`. The firmware
device tree documents `timebase-frequency = 25000000`; the constructor accepts
that CSR timebase frequency. The documented SG200x peripheral timer block at
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
LibXR callbacks for all pending pins in that controller.

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
