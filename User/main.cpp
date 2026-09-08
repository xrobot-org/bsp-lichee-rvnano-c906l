#include "main.h"

#include <cstdint>

#include "sg200x_gpio.hpp"
#include "timer.hpp"

namespace
{
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
  const auto result =
      led.SetConfig({LibXR::GPIO::Direction::OUTPUT_PUSH_PULL, LibXR::GPIO::Pull::NONE});
  if (result != LibXR::ErrorCode::OK)
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
