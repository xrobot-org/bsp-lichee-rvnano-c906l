#include "sg200x_rcc.hpp"

#include "sg200x_mmio.hpp"

namespace LibXR
{
namespace
{
constexpr uintptr_t CLOCK_GEN_BASE = 0x03002000u;
constexpr uintptr_t RESET_CTRL_BASE = 0x03003000u;
constexpr uint8_t MAX_CLOCK_DEPTH = 8u;

[[nodiscard]] constexpr uint32_t Mask(uint8_t width) noexcept
{
  return width == 0u ? 0u
                     : (width >= 32u ? UINT32_MAX : (static_cast<uint32_t>(1u) << width) - 1u);
}

[[nodiscard]] uint32_t ReadField(const SG200XClockTree::RegisterField& field) noexcept
{
  if (!field.Exists() || field.width == 0u)
  {
    return 0u;
  }
  return (Register32(CLOCK_GEN_BASE + field.offset) >> field.shift) & Mask(field.width);
}

void SetField(const SG200XClockTree::RegisterField& field, bool enabled) noexcept
{
  if (!field.Exists() || field.width != 1u)
  {
    return;
  }
  auto& value = Register32(CLOCK_GEN_BASE + field.offset);
  const uint32_t bit = static_cast<uint32_t>(1u) << field.shift;
  value = enabled ? value | bit : value & ~bit;
}

void WriteField(const SG200XClockTree::RegisterField& field, uint32_t value) noexcept
{
  if (!field.Exists() || field.width == 0u)
  {
    return;
  }
  auto& reg = Register32(CLOCK_GEN_BASE + field.offset);
  const uint32_t field_mask = Mask(field.width) << field.shift;
  reg = (reg & ~field_mask) | ((value & Mask(field.width)) << field.shift);
}

[[nodiscard]] uint32_t DividerValue(const SG200XClockTree::Divider& divider) noexcept
{
  if (!divider.Exists())
  {
    return 1u;
  }
  const uint32_t value = Register32(CLOCK_GEN_BASE + divider.field.offset);
  // All described CV181x mux/divider branches use bit 3 as their valid flag.
  // Before Linux has programmed a branch, its provider reports the documented
  // reset value rather than the uninitialised divider field.
  if (divider.reset_value != 0u && (value & Bit(3u)) == 0u)
  {
    return divider.reset_value;
  }
  return (value >> divider.field.shift) & Mask(divider.field.width);
}

[[nodiscard]] uint32_t G6PllRate(uint16_t csr) noexcept
{
  const uint32_t value = Register32(CLOCK_GEN_BASE + csr);
  const uint32_t pre_divider = value & 0x7Fu;
  const uint32_t post_divider = (value >> 8u) & 0x7Fu;
  const uint32_t multiplier = (value >> 17u) & 0x7Fu;
  if (pre_divider == 0u || post_divider == 0u || multiplier == 0u)
  {
    return 0u;
  }
  const uint64_t numerator =
      static_cast<uint64_t>(SG200XClockTree::OSCILLATOR_HZ) * multiplier;
  return static_cast<uint32_t>(numerator / (pre_divider * post_divider));
}

[[nodiscard]] uint8_t ActiveParentIndex(const SG200XClockTree::ClockNode& node) noexcept
{
  using NodeKind = SG200XClockTree::NodeKind;
  if (node.kind == NodeKind::Gate)
  {
    return 0u;
  }
  if (node.kind != NodeKind::MuxDividerGate)
  {
    return 0xFFu;
  }
  if (node.bypass.Exists() && ReadField(node.bypass) != 0u)
  {
    return 0u;
  }
  if (node.path_select.Exists())
  {
    // The vendor driver's get_clk_sel() inverts the register bit.
    return ReadField(node.path_select) == 0u
               ? 1u
               : static_cast<uint8_t>(2u + ReadField(node.source_select));
  }
  if (node.source_select.Exists())
  {
    return static_cast<uint8_t>((node.bypass.Exists() ? 1u : 0u) +
                                ReadField(node.source_select));
  }
  return node.bypass.Exists() ? 1u : 0u;
}
}  // namespace

SG200XRCC SG200XRCC::instance_;

SG200XRCC& SG200XRCC::Instance() noexcept { return instance_; }

void SG200XRCC::Lock() noexcept
{
  while (lock_.test_and_set(std::memory_order_acquire))
  {
  }
}

void SG200XRCC::Unlock() noexcept { lock_.clear(std::memory_order_release); }

ErrorCode SG200XRCC::EnableClock(ClockId clock) noexcept
{
  Lock();
  const ErrorCode result = EnableClockPathLocked(clock, 0u);
  Unlock();
  return result;
}

ErrorCode SG200XRCC::EnableClockPathLocked(ClockId clock, uint8_t depth) noexcept
{
  if (depth >= MAX_CLOCK_DEPTH)
  {
    return ErrorCode::STATE_ERR;
  }
  const SG200XClockTree::ClockNode* node = SG200XClockTree::Find(clock);
  if (node == nullptr)
  {
    return ErrorCode::ARG_ERR;
  }
  const ErrorCode plan_result = ApplyC906LClockPlanLocked(clock, depth);
  if (plan_result != ErrorCode::OK)
  {
    return plan_result;
  }
  if (node->ownership == SG200XClockTree::Ownership::RuntimeOnly)
  {
    return node->gate.Exists() ? ErrorCode::NOT_SUPPORT : ErrorCode::OK;
  }

  const uint8_t parent_index = ActiveParentIndex(*node);
  if (parent_index >= node->parents.count)
  {
    return ErrorCode::STATE_ERR;
  }
  const ErrorCode result = EnableClockPathLocked(
      node->parents.ids[parent_index], static_cast<uint8_t>(depth + 1u));
  if (result != ErrorCode::OK)
  {
    return result;
  }
  if (!node->gate.Exists())
  {
    return ErrorCode::OK;
  }
  SetField(node->gate, true);
  return ErrorCode::OK;
}

ErrorCode SG200XRCC::ApplyC906LClockPlanLocked(ClockId clock, uint8_t depth) noexcept
{
  const SG200XClockTree::ClockRatePlan* plan =
      SG200XClockTree::DEFAULT_C906L_CLOCK_PLAN.Find(clock);
  if (plan == nullptr)
  {
    return ErrorCode::OK;
  }
  if (depth >= MAX_CLOCK_DEPTH)
  {
    return ErrorCode::STATE_ERR;
  }

  const uint8_t plan_index = static_cast<uint8_t>(
      plan - SG200XClockTree::DEFAULT_C906L_CLOCK_PLAN.clocks.data());
  const uint8_t plan_bit = static_cast<uint8_t>(1u << plan_index);
  if ((applied_clock_plan_mask_ & plan_bit) != 0u)
  {
    return ErrorCode::OK;
  }

  const SG200XClockTree::ClockNode* node = SG200XClockTree::Find(clock);
  if (node == nullptr || node->ownership != SG200XClockTree::Ownership::C906LManaged ||
      node->kind != SG200XClockTree::NodeKind::MuxDividerGate ||
      node->path_select.Exists() || !node->divider0.Exists() ||
      plan->parent_index >= node->parents.count ||
      node->parents.ids[plan->parent_index] != plan->parent)
  {
    return ErrorCode::STATE_ERR;
  }

  const ErrorCode parent_result =
      ApplyC906LClockPlanLocked(plan->parent, static_cast<uint8_t>(depth + 1u));
  if (parent_result != ErrorCode::OK)
  {
    return parent_result;
  }
  // A generated plan is valid only for the exact root rate it was solved
  // against. This catches a board image with a different PLL contract before
  // its C906L peripheral routes are modified.
  if (ClockRateRecursive(plan->parent, static_cast<uint8_t>(depth + 1u)) !=
      plan->parent_rate_hz)
  {
    return ErrorCode::STATE_ERR;
  }

  WriteField(node->divider0.field, plan->divider);
  // CV181x divider registers report their reset-value until this valid bit is
  // set. This mirrors the vendor clock provider's set_rate path.
  if (node->divider0.reset_value != 0u)
  {
    Register32(CLOCK_GEN_BASE + node->divider0.field.offset) |= Bit(3u);
  }

  if (plan->parent_index == 0u)
  {
    if (node->bypass.Exists())
    {
      SetField(node->bypass, true);
    }
  }
  else
  {
    if (node->bypass.Exists())
    {
      SetField(node->bypass, false);
    }
    if (node->source_select.Exists())
    {
      const uint8_t source_index = static_cast<uint8_t>(
          plan->parent_index - (node->bypass.Exists() ? 1u : 0u));
      WriteField(node->source_select, source_index);
    }
  }

  if (ClockRateRecursive(clock, depth) != plan->actual_rate_hz)
  {
    return ErrorCode::STATE_ERR;
  }
  applied_clock_plan_mask_ |= plan_bit;
  return ErrorCode::OK;
}

ErrorCode SG200XRCC::ReleaseReset(ResetId reset) noexcept
{
  Lock();
  ReleaseResetLocked(reset);
  Unlock();
  return ErrorCode::OK;
}

void SG200XRCC::ReleaseResetLocked(ResetId reset) noexcept
{
  // The cvitek reset controller deassert operation writes a one: reset lines
  // are active low and indexed linearly across 32-bit registers.
  const uint16_t id = static_cast<uint16_t>(reset);
  const uintptr_t address = RESET_CTRL_BASE + static_cast<uintptr_t>(id / 32u) * 4u;
  const uint32_t bit = static_cast<uint32_t>(1u) << (id % 32u);
  Register32(address) |= bit;
}

void SG200XRCC::PulseResetLocked(ResetId reset) noexcept
{
  const uint16_t id = static_cast<uint16_t>(reset);
  const uintptr_t address = RESET_CTRL_BASE + static_cast<uintptr_t>(id / 32u) * 4u;
  const uint32_t bit = static_cast<uint32_t>(1u) << (id % 32u);
  auto& reset_register = Register32(address);

  // TOP reset lines are active low. Read back both writes so the reset pulse
  // reaches the peripheral before any subsequent MMIO initialization.
  reset_register &= ~bit;
  asm volatile("fence iorw, iorw" ::: "memory");
  const uint32_t asserted_state = reset_register;
  (void)asserted_state;
  reset_register |= bit;
  asm volatile("fence iorw, iorw" ::: "memory");
  const uint32_t released_state = reset_register;
  (void)released_state;
}

ErrorCode SG200XRCC::PreparePeripheral(PeripheralId peripheral) noexcept
{
  const SG200XClockTree::PeripheralResources* resource = SG200XClockTree::Find(peripheral);
  if (resource == nullptr)
  {
    return ErrorCode::ARG_ERR;
  }

  Lock();
  for (uint8_t index = 0u; index < resource->clock_count; ++index)
  {
    const ErrorCode result = EnableClockPathLocked(resource->clocks[index], 0u);
    if (result != ErrorCode::OK)
    {
      Unlock();
      return result;
    }
  }

  ReleaseResetLocked(resource->reset);
  Unlock();
  return ErrorCode::OK;
}

ErrorCode SG200XRCC::ResetPeripheral(PeripheralId peripheral) noexcept
{
  const SG200XClockTree::PeripheralResources* resource = SG200XClockTree::Find(peripheral);
  if (resource == nullptr)
  {
    return ErrorCode::ARG_ERR;
  }

  Lock();
  for (uint8_t index = 0u; index < resource->clock_count; ++index)
  {
    const ErrorCode result = EnableClockPathLocked(resource->clocks[index], 0u);
    if (result != ErrorCode::OK)
    {
      Unlock();
      return result;
    }
  }
  PulseResetLocked(resource->reset);
  Unlock();
  return ErrorCode::OK;
}

uint32_t SG200XRCC::ClockRate(ClockId clock) const noexcept
{
  return ClockRateRecursive(clock, 0u);
}

uint32_t SG200XRCC::ClockRateRecursive(ClockId clock, uint8_t depth) const noexcept
{
  if (depth >= MAX_CLOCK_DEPTH)
  {
    return 0u;
  }

  const SG200XClockTree::ClockNode* node = SG200XClockTree::Find(clock);
  if (node == nullptr)
  {
    return 0u;
  }
  if (node->kind == SG200XClockTree::NodeKind::Fixed)
  {
    return node->id == ClockId::Oscillator ? SG200XClockTree::OSCILLATOR_HZ : 0u;
  }
  if (node->kind == SG200XClockTree::NodeKind::G6Pll)
  {
    return G6PllRate(node->pll_csr);
  }
  if (node->kind == SG200XClockTree::NodeKind::G2Pll)
  {
    // G2 PLLs may run fractional synthesis. Do not manufacture a rate until
    // their complete SSC/fractional decoder is modelled and board-validated.
    return 0u;
  }
  if (node->kind == SG200XClockTree::NodeKind::Gate)
  {
    return node->parents.count == 1u
               ? ClockRateRecursive(node->parents.ids[0], static_cast<uint8_t>(depth + 1u))
               : 0u;
  }

  uint8_t parent_index = 0u;
  const SG200XClockTree::Divider* divider = &node->divider0;
  if (node->bypass.Exists() && ReadField(node->bypass) != 0u)
  {
    parent_index = 0u;
    divider = nullptr;
  }
  else if (node->path_select.Exists())
  {
    // The vendor driver's get_clk_sel() inverts this register bit.
    const bool uses_divider1 = ReadField(node->path_select) == 0u;
    if (uses_divider1)
    {
      parent_index = 1u;
      divider = &node->divider1;
    }
    else
    {
      parent_index = static_cast<uint8_t>(2u + ReadField(node->source_select));
    }
  }
  else
  {
    parent_index = node->source_select.Exists()
                       ? static_cast<uint8_t>(1u + ReadField(node->source_select))
                       : static_cast<uint8_t>(node->parents.count == 1u ? 0u : 1u);
  }

  if (parent_index >= node->parents.count)
  {
    return 0u;
  }
  const uint32_t parent_rate =
      ClockRateRecursive(node->parents.ids[parent_index], static_cast<uint8_t>(depth + 1u));
  if (parent_rate == 0u || divider == nullptr)
  {
    return parent_rate;
  }
  const uint32_t divisor = DividerValue(*divider);
  return divisor == 0u ? 0u : parent_rate / divisor;
}

}  // namespace LibXR
