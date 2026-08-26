#pragma once

#include <cstdint>

namespace LibXR
{

/**
 * @brief Volatile 32-bit MMIO access used by the SG200x peripheral drivers.
 *
 * Keeping the cast in one place makes the register-level code readable while
 * preserving the exact access semantics required by the memory-mapped blocks.
 */
inline volatile uint32_t& Register32(uintptr_t address) noexcept
{
  // MMIO addresses are physical constants; this is the sole integer-to-pointer boundary.
  // NOLINTNEXTLINE(performance-no-int-to-ptr)
  return *reinterpret_cast<volatile uint32_t*>(address);
}

inline volatile uint32_t& Register32(uintptr_t base, uint32_t offset) noexcept
{
  return Register32(base + static_cast<uintptr_t>(offset));
}

constexpr uint32_t Bit(uint32_t position) noexcept
{
  return static_cast<uint32_t>(1u << position);
}

inline void SetBits(uintptr_t address, uint32_t mask) noexcept
{
  Register32(address) |= mask;
}

inline void ClearBits(uintptr_t address, uint32_t mask) noexcept
{
  Register32(address) &= ~mask;
}

}  // namespace LibXR
