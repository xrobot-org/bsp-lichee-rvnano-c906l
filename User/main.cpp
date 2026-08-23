#include "main.h"

#include <cstdint>

#include "sg200x_gpio.hpp"
#include "timer.hpp"

namespace
{
// The stock LicheeRV Nano remoteproc driver requires a valid resource-table
// section even when this firmware only runs the LED task.
struct ResourceTable
{
  uint32_t version;
  uint32_t num;
  uint32_t reserved[2];
};

[[gnu::used, gnu::section(".resource_table"), gnu::aligned(8)]]
const ResourceTable RESOURCE_TABLE{1u, 0u, {0u, 0u}};

constexpr uint32_t LED_PERIOD_MS = 1000u;

struct LedState
{
  LibXR::SG200XGPIO* led;
  bool level;
};

void BlinkLed(LedState* state)
{
  if (state == nullptr || state->led == nullptr)
  {
    return;
  }

  state->level = !state->level;
  state->led->Write(state->level);
}
}  // namespace

extern "C" void CreateDefaultTask(void)
{
  // GPIOA_14 is the active-low user LED on LicheeRV Nano.
  static LibXR::SG200XGPIO led(LibXR::SG200XGPIO::Bank::A, 14u);
  const auto config_result =
      led.SetConfig({LibXR::GPIO::Direction::OUTPUT_PUSH_PULL, LibXR::GPIO::Pull::NONE});
  if (config_result != LibXR::ErrorCode::OK)
  {
    return;
  }

  led.Write(false);

  static LedState state{&led, false};
  const auto timer = LibXR::Timer::CreateTask<LedState*>(BlinkLed, &state, LED_PERIOD_MS);
  if (timer == nullptr)
  {
    return;
  }
  LibXR::Timer::Add(timer);
  LibXR::Timer::Start(timer);
}

extern "C" void DefaultTask(void* argument)
{
  BlinkLed(static_cast<LedState*>(argument));
}
