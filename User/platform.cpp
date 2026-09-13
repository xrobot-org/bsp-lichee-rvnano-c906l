#include <cstdint>
#include "sg200x_ll_csr.h"

extern "C"
{
#include "arch_helpers.h"
}

namespace
{
struct ResourceTable
{
  uint32_t version;
  uint32_t num;
  uint32_t reserved[2];
};

// The stock remoteproc driver requires the section even without RPMsg resources.
[[gnu::used, gnu::section(".resource_table"), gnu::aligned(8)]]
const ResourceTable RESOURCE_TABLE{1u, 0u, {0u, 0u}};

constexpr uintptr_t BOOT_TRACE_ADDRESS = 0x8FFFF000u;
}  // namespace

extern "C" void sg200x_config_assert(const char* file, unsigned long line)
{
  uint32_t file_tag = 2166136261u;
  while (*file != '\0')
  {
    file_tag = (file_tag ^ static_cast<uint8_t>(*file++)) * 16777619u;
  }

  auto* const trace = reinterpret_cast<volatile uint32_t*>(BOOT_TRACE_ADDRESS);
  trace[4] = 0xC906A55Eu;
  trace[5] = ~trace[4];
  trace[6] = static_cast<uint32_t>(line);
  trace[7] = ~trace[6];
  trace[8] = 0xA55E0001u;
  trace[9] = ~trace[8];
  trace[10] = file_tag;
  trace[11] = ~trace[10];
  sg200x_ll_csr_dcache_clean_invalidate_range(BOOT_TRACE_ADDRESS + 16u, 32u);

  for (;;)
  {
    asm volatile("wfi");
  }
}
