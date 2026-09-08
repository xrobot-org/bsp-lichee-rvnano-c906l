#include "sg200x_timebase.hpp"

#include <limits>

#include "sg200x_ll_csr.h"

namespace LibXR
{
namespace
{
uint32_t g_clock_hz = 0u;

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

SG200XTimebase g_sg200x_timebase;
}  // namespace

SG200XTimebase::SG200XTimebase(uint32_t clock_hz)
{
  if (clock_hz == 0u)
  {
    g_clock_hz = 0u;
    ConfigureWrapRange(0u, 0u);
    SetReady(false);
    return;
  }

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
  const uint64_t ticks = sgll_csr_time_read();
  return MicrosecondTimestamp(ticks_to_microseconds(ticks));
}

MillisecondTimestamp Timebase::GetMilliseconds()
{
  if (g_clock_hz == 0u)
  {
    return MillisecondTimestamp(0u);
  }
  const uint64_t ticks = sgll_csr_time_read();
  return MillisecondTimestamp(ticks_to_microseconds(ticks) / 1000ULL);
}

}  // namespace LibXR
