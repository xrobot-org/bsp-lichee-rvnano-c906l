#pragma once

#include <cstddef>
#include <cstdint>

#if defined(__riscv)
#error "Host peripheral tests must not replace real C906 core operations"
#endif

// The host has coherent memory. These test-only hooks record maintenance calls;
// the SG200X LL RV64/QEMU suite verifies the real cache instructions separately.
extern "C"
{
  void sg200x_ll_csr_dcache_clean_range(uintptr_t address, size_t size);
  void sg200x_ll_csr_dcache_invalidate_range(uintptr_t address, size_t size);
  void sg200x_ll_csr_dcache_clean_invalidate_range(uintptr_t address, size_t size);
  void sg200x_ll_csr_delay_nops(uint32_t iterations);
}
