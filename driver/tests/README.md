# SG200x driver tests

Run from the BSP root:

```sh
cmake -S driver/tests -B /tmp/sg200x-clock-tests -G Ninja \
  -DCMAKE_C_FLAGS="-O2 -Wall -Wextra -Werror" \
  -DCMAKE_CXX_FLAGS="-O2 -Wall -Wextra -Werror"
cmake --build /tmp/sg200x-clock-tests
ctest --test-dir /tmp/sg200x-clock-tests --output-on-failure
```

The tests compile the C23 SG200X LL library and the C++20 RCC controller. Planner
tests pass runtime values and cover rate policies, hardware ranges, bypass
semantics, gate-leaf resolution, and static topology/field/resource checks.
Controller tests map anonymous register pages and verify real register writes,
parent-rate validation, rejection without mutation, rollback after a dependency
failure, dependency-aware disable, default-profile preservation, shared SPI
rates, and PLL/CPU/system-bus ownership restrictions. They do not validate
physical clock waveforms or PLL lock behavior on hardware.

`peripheral_driver_test` links the real LibXR host backend and the production
GPIO, PWM, ADC, watchdog, I2C, SPI, and DMA adapters to SG200X LL. It checks GPIO
ownership and IRQ callbacks, caller-provided PWM clock arithmetic, ADC domain
mapping and failure results, watchdog timeout rounding, I2C command streams and
STOP/DMA completion in different orders, NACK recovery, SPI RX completion without
a TX completion IRQ, and normal/circular descriptor reuse and channel release.

Only the tests replace C906 cache/NOP operations and emulate I2C enable
acknowledgments. The real LL polling implementation still runs; SG200X LL's own tests
check enable timeouts and RV64 cache instructions separately. Interrupt injection
updates the anonymous register image to model completion and clear side effects.
No test accesses a physical board or starts DMA hardware.
