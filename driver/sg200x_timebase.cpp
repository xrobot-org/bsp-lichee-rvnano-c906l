#include "sg200x_timebase.hpp"

#include <limits>

namespace LibXR
{
using InitFunction = void (*)();

extern "C"
{
extern InitFunction __preinit_array_start[];
extern InitFunction __preinit_array_end[];
extern InitFunction __init_array_start[];
extern InitFunction __init_array_end[];
extern InitFunction __CTOR_LIST__[];
extern InitFunction __CTOR_END__[];
}

namespace
{
uint32_t g_clock_hz = 0u;

uint64_t read_time_csr()
{
  uint64_t value = 0;
  asm volatile("rdtime %0" : "=r"(value));
  return value;
}

uint64_t ticks_to_microseconds(uint64_t ticks)
{
  if (g_clock_hz == 0u)
  {
    return 0u;
  }

  // Split the multiplication to avoid overflowing for the extended counter.
  const uint64_t whole = ticks / g_clock_hz;
  const uint64_t remainder = ticks % g_clock_hz;
  return whole * 1000000ULL + (remainder * 1000000ULL) / g_clock_hz;
}

}  // namespace

// The vendor C906 start-up object is linked with -nostartfiles and therefore
// does not provide the usual C++ runtime constructor walk. Keep the walk in
// the XRobot library so the platform start-up can invoke it before main().
extern "C" void sg200x_run_global_constructors()
{
  for (auto* function = __preinit_array_start; function != __preinit_array_end; ++function)
  {
    (*function)();
  }
  for (auto* function = __init_array_start; function != __init_array_end; ++function)
  {
    (*function)();
  }
  for (auto* function = __CTOR_LIST__; function != __CTOR_END__; ++function)
  {
    if (*function != nullptr && *function != reinterpret_cast<InitFunction>(-1))
    {
      (*function)();
    }
  }
}

namespace
{
SG200XTimebase g_sg200x_timebase;
}  // namespace

SG200XTimebase::SG200XTimebase(uintptr_t clint_base, uint32_t clock_hz)
{
  if (clint_base == 0u || clock_hz == 0u)
  {
    g_clock_hz = 0u;
    ConfigureWrapRange(0u, 0u);
    SetReady(false);
    return;
  }

  (void)clint_base;
  g_clock_hz = clock_hz;

  // LibXR's public timestamp values are 32-bit milliseconds and microseconds
  // with the standard unsigned wrap semantics.
  ConfigureWrapRange(static_cast<uint64_t>(UINT32_MAX) * 1000ULL + 999ULL,
                     std::numeric_limits<uint32_t>::max());
  SetReady(true);
}

uint32_t SG200XTimebase::ClockHz() noexcept { return g_clock_hz; }

MicrosecondTimestamp Timebase::GetMicroseconds()
{
  if (g_clock_hz == 0u)
  {
    return MicrosecondTimestamp(0u);
  }
  const uint64_t ticks = read_time_csr();
  return MicrosecondTimestamp(ticks_to_microseconds(ticks));
}

MillisecondTimestamp Timebase::GetMilliseconds()
{
  if (g_clock_hz == 0u)
  {
    return MillisecondTimestamp(0u);
  }
  const uint64_t ticks = read_time_csr();
  return MillisecondTimestamp(ticks_to_microseconds(ticks) / 1000ULL);
}

}  // namespace LibXR
