#include "sg200x_rcc.hpp"

#include <array>

#include "sg200x_mmio.hpp"

namespace LibXR
{
namespace
{
constexpr uintptr_t CLOCK_GEN_BASE = 0x03002000u;
constexpr uintptr_t RESET_CTRL_BASE = 0x03003000u;

void IoFence() noexcept { asm volatile("fence iorw, iorw" ::: "memory"); }

[[nodiscard]] constexpr uint32_t Mask(uint8_t width) noexcept
{
  return width == 0u
             ? 0u
             : (width >= 32u ? UINT32_MAX
                             : (static_cast<uint32_t>(1u) << width) - 1u);
}

[[nodiscard]] uint32_t ReadField(
    const SG200XClockTree::RegisterField& field) noexcept
{
  if (!SG200XClockTree::IsFieldWellFormed(field) || !field.Exists())
  {
    return 0u;
  }
  return (Register32(CLOCK_GEN_BASE + field.offset) >> field.shift) &
         Mask(field.width);
}

[[nodiscard]] bool SetField(const SG200XClockTree::RegisterField& field,
                            bool enabled) noexcept
{
  if (!SG200XClockTree::IsFieldWellFormed(field) || !field.Exists() ||
      field.width != 1u)
  {
    return false;
  }
  auto& value = Register32(CLOCK_GEN_BASE + field.offset);
  const uint32_t bit = static_cast<uint32_t>(1u) << field.shift;
  value = enabled ? value | bit : value & ~bit;
  IoFence();
  return (value & bit) == (enabled ? bit : 0u);
}

[[nodiscard]] bool WriteField(const SG200XClockTree::RegisterField& field,
                              uint32_t value) noexcept
{
  if (!SG200XClockTree::IsFieldWellFormed(field) || !field.Exists() ||
      value > Mask(field.width))
  {
    return false;
  }
  auto& reg = Register32(CLOCK_GEN_BASE + field.offset);
  const uint32_t field_mask = Mask(field.width) << field.shift;
  reg = (reg & ~field_mask) | (value << field.shift);
  IoFence();
  return ((reg & field_mask) >> field.shift) == value;
}

[[nodiscard]] uint32_t DividerValue(
    const SG200XClockTree::Divider& divider) noexcept
{
  if (!divider.Exists())
  {
    return 1u;
  }
  const uint32_t value = Register32(CLOCK_GEN_BASE + divider.field.offset);
  // All described CV181x mux/divider branches use bit 3 as their valid flag.
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
  const uint64_t denominator =
      static_cast<uint64_t>(pre_divider) * post_divider;
  return static_cast<uint32_t>(numerator / denominator);
}

[[nodiscard]] uint8_t ActiveParentIndex(
    const SG200XClockTree::ClockNode& node) noexcept
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

struct RegisterSnapshot
{
  std::array<uintptr_t, 3u> addresses{};
  std::array<uint32_t, 3u> values{};
  uint8_t count = 0u;

  void Add(const SG200XClockTree::RegisterField& field) noexcept
  {
    if (!field.Exists())
    {
      return;
    }
    const uintptr_t address = CLOCK_GEN_BASE + field.offset;
    for (uint8_t index = 0u; index < count; ++index)
    {
      if (addresses[index] == address)
      {
        return;
      }
    }
    if (count < addresses.size())
    {
      addresses[count] = address;
      values[count] = Register32(address);
      ++count;
    }
  }

  [[nodiscard]] bool Restore() const noexcept
  {
    for (uint8_t index = 0u; index < count; ++index)
    {
      Register32(addresses[index]) = values[index];
    }
    IoFence();
    for (uint8_t index = 0u; index < count; ++index)
    {
      if (Register32(addresses[index]) != values[index])
      {
        return false;
      }
    }
    return true;
  }
};

[[nodiscard]] bool IsPlanRouteActive(
    const SG200XClockTree::ClockNode& node,
    const SG200XClockTree::ClockRatePlan& plan) noexcept
{
  if (ReadField(node.divider0.field) != plan.divider ||
      (node.divider0.reset_value != 0u &&
       (Register32(CLOCK_GEN_BASE + node.divider0.field.offset) & Bit(3u)) == 0u))
  {
    return false;
  }
  if (node.bypass.Exists() &&
      ReadField(node.bypass) != (plan.parent_index == 0u ? 1u : 0u))
  {
    return false;
  }
  if (plan.parent_index != 0u && node.source_select.Exists())
  {
    const uint8_t expected = static_cast<uint8_t>(
        plan.parent_index - (node.bypass.Exists() ? 1u : 0u));
    if (ReadField(node.source_select) != expected)
    {
      return false;
    }
  }
  return true;
}

[[nodiscard]] bool ResetAddressAndBit(SG200XClockTree::ResetId reset,
                                      uintptr_t& address,
                                      uint32_t& bit) noexcept
{
  if (!SG200XClockTree::IsKnownReset(reset))
  {
    return false;
  }
  const uint16_t id = static_cast<uint16_t>(reset);
  address = RESET_CTRL_BASE + static_cast<uintptr_t>(id / 32u) * 4u;
  bit = static_cast<uint32_t>(1u) << (id % 32u);
  return true;
}
}  // namespace

SG200XRCC SG200XRCC::instance_;

SG200XRCC& SG200XRCC::Instance() noexcept { return instance_; }

bool SG200XRCC::TryBeginWrite() noexcept
{
  if (write_busy_.test_and_set(std::memory_order_acquire))
  {
    return false;
  }
  write_sequence_.fetch_add(1u, std::memory_order_acq_rel);
  return true;
}

void SG200XRCC::EndWrite() noexcept
{
  // Publish all device writes before readers can observe an even sequence.
  IoFence();
  write_sequence_.fetch_add(1u, std::memory_order_release);
  write_busy_.clear(std::memory_order_release);
}

ErrorCode SG200XRCC::EnableClock(ClockId clock) noexcept
{
  if (SG200XClockTree::Find(clock) == nullptr)
  {
    return ErrorCode::ARG_ERR;
  }
  if (!TryBeginWrite())
  {
    return ErrorCode::BUSY;
  }
  const ErrorCode result = EnableClockPathLocked(clock, 0u);
  EndWrite();
  return result;
}

ErrorCode SG200XRCC::EnableClockPathLocked(ClockId clock, uint8_t depth) noexcept
{
  if (depth >= SG200XClockTree::MAX_CLOCK_DEPTH)
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
  return SetField(node->gate, true) ? ErrorCode::OK : ErrorCode::CHECK_ERR;
}

ErrorCode SG200XRCC::ApplyC906LClockPlanLocked(ClockId clock,
                                               uint8_t depth) noexcept
{
  const SG200XClockTree::ClockRatePlan* plan =
      SG200XClockTree::DEFAULT_C906L_CLOCK_PLAN.Find(clock);
  if (plan == nullptr)
  {
    return ErrorCode::OK;
  }
  if (depth >= SG200XClockTree::MAX_CLOCK_DEPTH)
  {
    return ErrorCode::STATE_ERR;
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
  if (ClockRateRecursive(plan->parent, static_cast<uint8_t>(depth + 1u)) !=
      plan->parent_rate_hz)
  {
    return ErrorCode::STATE_ERR;
  }

  // Do not trust software history: remote resets or another owner may have
  // changed a managed branch since the last call.
  if (IsPlanRouteActive(*node, *plan) &&
      ClockRateRecursive(clock, depth) == plan->actual_rate_hz)
  {
    return ErrorCode::OK;
  }

  RegisterSnapshot original;
  original.Add(node->divider0.field);
  original.Add(node->bypass);
  original.Add(node->source_select);

  bool written = WriteField(node->divider0.field, plan->divider);
  if (written && node->divider0.reset_value != 0u)
  {
    auto& divider_register =
        Register32(CLOCK_GEN_BASE + node->divider0.field.offset);
    divider_register |= Bit(3u);
    IoFence();
    written = (divider_register & Bit(3u)) != 0u;
  }

  if (written && plan->parent_index == 0u && node->bypass.Exists())
  {
    written = SetField(node->bypass, true);
  }
  else if (written && plan->parent_index != 0u)
  {
    if (node->source_select.Exists())
    {
      const uint8_t source_index = static_cast<uint8_t>(
          plan->parent_index - (node->bypass.Exists() ? 1u : 0u));
      written = WriteField(node->source_select, source_index);
    }
    if (written && node->bypass.Exists())
    {
      written = SetField(node->bypass, false);
    }
  }

  if (!written || !IsPlanRouteActive(*node, *plan) ||
      ClockRateRecursive(clock, depth) != plan->actual_rate_hz)
  {
    return original.Restore() ? ErrorCode::CHECK_ERR : ErrorCode::FAILED;
  }
  return ErrorCode::OK;
}

ErrorCode SG200XRCC::ReleaseReset(ResetId reset) noexcept
{
  if (!SG200XClockTree::IsKnownReset(reset))
  {
    return ErrorCode::ARG_ERR;
  }
  if (!TryBeginWrite())
  {
    return ErrorCode::BUSY;
  }
  const ErrorCode result = ReleaseResetLocked(reset);
  EndWrite();
  return result;
}

ErrorCode SG200XRCC::ReleaseResetLocked(ResetId reset) noexcept
{
  uintptr_t address = 0u;
  uint32_t bit = 0u;
  if (!ResetAddressAndBit(reset, address, bit))
  {
    return ErrorCode::ARG_ERR;
  }
  auto& reset_register = Register32(address);
  reset_register |= bit;
  IoFence();
  return (reset_register & bit) != 0u ? ErrorCode::OK : ErrorCode::CHECK_ERR;
}

ErrorCode SG200XRCC::PulseResetLocked(ResetId reset) noexcept
{
  uintptr_t address = 0u;
  uint32_t bit = 0u;
  if (!ResetAddressAndBit(reset, address, bit))
  {
    return ErrorCode::ARG_ERR;
  }
  auto& reset_register = Register32(address);

  reset_register &= ~bit;
  IoFence();
  const bool asserted = (reset_register & bit) == 0u;

  // Always attempt to release the peripheral, including after a failed
  // assertion readback, so an error does not intentionally leave it in reset.
  reset_register |= bit;
  IoFence();
  const bool released = (reset_register & bit) != 0u;
  return asserted && released ? ErrorCode::OK : ErrorCode::CHECK_ERR;
}

ErrorCode SG200XRCC::PreparePeripheral(PeripheralId peripheral) noexcept
{
  const SG200XClockTree::PeripheralResources* resource =
      SG200XClockTree::Find(peripheral);
  if (resource == nullptr)
  {
    return ErrorCode::ARG_ERR;
  }

  if (!TryBeginWrite())
  {
    return ErrorCode::BUSY;
  }
  ErrorCode result = ErrorCode::OK;
  for (uint8_t index = 0u; index < resource->clock_count; ++index)
  {
    result = EnableClockPathLocked(resource->clocks[index], 0u);
    if (result != ErrorCode::OK)
    {
      break;
    }
  }
  if (result == ErrorCode::OK)
  {
    result = ReleaseResetLocked(resource->reset);
  }
  EndWrite();
  return result;
}

ErrorCode SG200XRCC::ResetPeripheral(PeripheralId peripheral) noexcept
{
  const SG200XClockTree::PeripheralResources* resource =
      SG200XClockTree::Find(peripheral);
  if (resource == nullptr)
  {
    return ErrorCode::ARG_ERR;
  }

  if (!TryBeginWrite())
  {
    return ErrorCode::BUSY;
  }
  ErrorCode result = ErrorCode::OK;
  for (uint8_t index = 0u; index < resource->clock_count; ++index)
  {
    result = EnableClockPathLocked(resource->clocks[index], 0u);
    if (result != ErrorCode::OK)
    {
      break;
    }
  }
  if (result == ErrorCode::OK)
  {
    result = PulseResetLocked(resource->reset);
  }
  EndWrite();
  return result;
}

uint32_t SG200XRCC::ClockRate(ClockId clock) const noexcept
{
  const uint32_t begin = write_sequence_.load(std::memory_order_acquire);
  if ((begin & 1u) != 0u)
  {
    return 0u;
  }
  const uint32_t rate = ClockRateRecursive(clock, 0u);
  IoFence();
  const uint32_t end = write_sequence_.load(std::memory_order_acquire);
  return begin == end && (end & 1u) == 0u ? rate : 0u;
}

uint32_t SG200XRCC::ClockRateRecursive(ClockId clock, uint8_t depth) const noexcept
{
  if (depth >= SG200XClockTree::MAX_CLOCK_DEPTH)
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
               ? ClockRateRecursive(node->parents.ids[0],
                                    static_cast<uint8_t>(depth + 1u))
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
  const uint32_t parent_rate = ClockRateRecursive(
      node->parents.ids[parent_index], static_cast<uint8_t>(depth + 1u));
  if (parent_rate == 0u || divider == nullptr)
  {
    return parent_rate;
  }
  const uint32_t divisor = DividerValue(*divider);
  return divisor == 0u ? 0u : parent_rate / divisor;
}

}  // namespace LibXR
