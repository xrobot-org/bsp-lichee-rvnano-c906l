#include "sg200x_rcc.hpp"

#include "sg200x_ll_rcc.h"

namespace LibXR
{
namespace Tree = SG200XClockTree;

namespace detail
{
inline uint32_t RccField(const SG200XClockTree::RegisterField& field) noexcept
{
  return field.Exists() ? sgll_rcc_field_read(field.offset, field.shift, field.width)
                        : 0u;
}

inline uint32_t RccDividerValue(const SG200XClockTree::Divider& divider) noexcept
{
  if (!divider.Exists())
  {
    return 1u;
  }
  if (divider.reset_value != 0u &&
      !sgll_rcc_div_uses_register_factor(divider.field.offset))
  {
    return divider.reset_value;
  }
  return sgll_rcc_div_factor_raw_get(divider.field.offset, divider.field.width);
}

inline uint8_t RccParentIndex(const SG200XClockTree::ClockNode& node) noexcept
{
  if (node.kind == SG200XClockTree::NodeKind::Gate)
  {
    return 0u;
  }
  if (node.bypass.Exists() && RccField(node.bypass) != 0u)
  {
    return 0u;
  }
  if (node.path_select.Exists())
  {
    return RccField(node.path_select) == 0u
               ? 1u
               : static_cast<uint8_t>(2u + RccField(node.source_select));
  }
  if (node.source_select.Exists())
  {
    return static_cast<uint8_t>((node.bypass.Exists() ? 1u : 0u) +
                                RccField(node.source_select));
  }
  return node.bypass.Exists() ? 1u : 0u;
}

inline uint32_t RccG6PllRate(uint16_t csr) noexcept
{
  const uint32_t value = sgll_rcc_g6_pll_read(csr);
  const uint32_t pre = sgll_field_get(value, PLL_G6_PREDIV_SHIFT, PLL_G6_DIVIDER_WIDTH);
  const uint32_t post = sgll_field_get(value, PLL_G6_POSTDIV_SHIFT, PLL_G6_DIVIDER_WIDTH);
  const uint32_t multiplier =
      sgll_field_get(value, PLL_G6_MULTIPLIER_SHIFT, PLL_G6_DIVIDER_WIDTH);
  if (pre == 0u || post == 0u || multiplier == 0u)
  {
    return 0u;
  }
  return static_cast<uint32_t>(
      (static_cast<uint64_t>(SG200XClockTree::OSCILLATOR_HZ) * multiplier) /
      (static_cast<uint64_t>(pre) * post));
}
}  // namespace detail

SG200XRCC SG200XRCC::instance_{};

struct SG200XRCC::Transaction
{
  struct Change
  {
    Tree::RegisterField field{};
    uint32_t previous = 0u;
  };
  SG200XRCC& owner;
  std::array<Change, Tree::MAX_CLOCK_DEPTH * 3u + 8u> changes{};
  std::array<bool, Tree::NODES.size()> configured;
  std::size_t count = 0u;
  bool committed = false;

  explicit Transaction(SG200XRCC& controller) noexcept
      : owner(controller), configured(controller.configured_)
  {
  }

  static bool Store(const Tree::RegisterField& field, uint32_t value) noexcept
  {
    if (!Tree::IsFieldWellFormed(field) || !field.Exists() ||
        static_cast<uint64_t>(value) >= (uint64_t{1u} << field.width))
      return false;
    return sgll_rcc_field_write(field.offset, field.shift, field.width, value);
  }

  bool Write(const Tree::RegisterField& field, uint32_t value) noexcept
  {
    if (!field.Exists() || !Tree::IsFieldWellFormed(field) ||
        static_cast<uint64_t>(value) >= (uint64_t{1u} << field.width))
      return false;
    const uint32_t previous = detail::RccField(field);
    if (previous == value) return true;
    if (count == changes.size()) return false;
    changes[count++] = {field, previous};
    return Store(field, value);
  }

  void Commit() noexcept
  {
    owner.configured_ = configured;
    committed = true;
  }

  ~Transaction()
  {
    if (!committed)
    {
      while (count != 0u)
      {
        const Change& change = changes[--count];
        (void)Store(change.field, change.previous);
      }
    }
    owner.EndWrite();
  }
};

namespace
{
ErrorCode PlanError(Tree::PlanStatus status) noexcept
{
  switch (status)
  {
    case Tree::PlanStatus::Ok:
      return ErrorCode::OK;
    case Tree::PlanStatus::Busy:
      return ErrorCode::BUSY;
    case Tree::PlanStatus::ReadOnlyClock:
    case Tree::PlanStatus::UnattainableRate:
      return ErrorCode::NOT_SUPPORT;
    case Tree::PlanStatus::UnknownParentRate:
      return ErrorCode::STATE_ERR;
    default:
      return ErrorCode::ARG_ERR;
  }
}

uint64_t PlanErrorNumerator(const Tree::ClockRatePlan& plan) noexcept
{
  const uint64_t desired = static_cast<uint64_t>(plan.target_rate_hz) * plan.divider;
  return desired >= plan.parent_rate_hz ? desired - plan.parent_rate_hz
                                        : plan.parent_rate_hz - desired;
}
}  // namespace

SG200XRCC& SG200XRCC::Instance() noexcept { return instance_; }

bool SG200XRCC::TryBeginWrite() noexcept
{
  if (write_busy_.test_and_set(std::memory_order_acquire)) return false;
  write_sequence_.fetch_add(1u, std::memory_order_acq_rel);
  return true;
}

void SG200XRCC::EndWrite() noexcept
{
  sgll_csr_fence_io();
  write_sequence_.fetch_add(1u, std::memory_order_release);
  write_busy_.clear(std::memory_order_release);
}

SG200XRCC::ClockRatePlan SG200XRCC::PlanRateLocked(ClockId clock, uint32_t target_rate_hz,
                                                   RatePolicy policy) const noexcept
{
  ClockRatePlan best{};
  best.clock = clock;
  best.target_rate_hz = target_rate_hz;
  best.policy = policy;
  const auto* node = Tree::RateControlNode(clock);
  if (node == nullptr) return best;
  if (target_rate_hz == 0u ||
      static_cast<uint8_t>(policy) > static_cast<uint8_t>(RatePolicy::Exact))
  {
    best.status = Tree::PlanStatus::InvalidRate;
    return best;
  }
  if (node->ownership != Tree::Ownership::C906LManaged ||
      node->kind != Tree::NodeKind::MuxDividerGate || node->path_select.Exists())
  {
    best.status = Tree::PlanStatus::ReadOnlyClock;
    return best;
  }
  best.status = Tree::PlanStatus::UnknownParentRate;
  const uint8_t current_parent = detail::RccParentIndex(*node);
  for (uint8_t index = 0u; index < node->parents.count; ++index)
  {
    const ClockId parent = node->parents.ids[index];
    const uint32_t parent_rate = ClockRateRecursive(parent, 0u);
    if (parent_rate == 0u) continue;
    if (!best.IsValid()) best.status = Tree::PlanStatus::UnattainableRate;
    const ClockRatePlan candidate =
        Tree::MakeRatePlan(clock, parent, parent_rate, target_rate_hz, policy);
    if (!candidate.IsValid()) continue;
    const uint64_t candidate_error = PlanErrorNumerator(candidate) * best.divider;
    const uint64_t best_error = PlanErrorNumerator(best) * candidate.divider;
    if (!best.IsValid() || candidate_error < best_error ||
        (candidate_error == best_error && index == current_parent &&
         best.parent_index != current_parent))
    {
      best = candidate;
    }
  }
  return best;
}

SG200XRCC::ClockRatePlan SG200XRCC::PlanRate(ClockId clock, uint32_t target_rate_hz,
                                             RatePolicy policy) const noexcept
{
  const uint32_t before = write_sequence_.load(std::memory_order_acquire);
  ClockRatePlan plan{};
  plan.clock = clock;
  plan.target_rate_hz = target_rate_hz;
  if ((before & 1u) == 0u) plan = PlanRateLocked(clock, target_rate_hz, policy);
  sgll_csr_fence_io();
  if ((before & 1u) != 0u || before != write_sequence_.load(std::memory_order_acquire))
    plan.status = Tree::PlanStatus::Busy;
  return plan;
}

SG200XRCC::ClockRatePlan SG200XRCC::PlanRate(PeripheralId peripheral,
                                             uint32_t target_rate_hz,
                                             RatePolicy policy) const noexcept
{
  const auto* resource = Tree::Find(peripheral);
  return PlanRate(resource == nullptr ? ClockId::None : resource->rate_clock,
                  target_rate_hz, policy);
}

bool SG200XRCC::PlanMatchesHardware(const ClockRatePlan& plan) const noexcept
{
  const auto* node = Tree::Find(plan.rate_clock);
  if (node == nullptr || detail::RccParentIndex(*node) != plan.parent_index) return false;
  if (node->bypass.Exists() && plan.parent_index == 0u) return true;
  return sgll_rcc_div_reset_is_deasserted(node->divider0.field.offset) &&
         detail::RccDividerValue(node->divider0) == plan.divider;
}

bool SG200XRCC::PathContains(ClockId clock, ClockId ancestor,
                             uint8_t depth) const noexcept
{
  if (clock == ancestor) return true;
  if (depth >= Tree::MAX_CLOCK_DEPTH) return false;
  const auto* node = Tree::Find(clock);
  if (node == nullptr || node->parents.count == 0u) return false;
  const uint8_t parent = node->parents.count == 1u ? 0u : detail::RccParentIndex(*node);
  return parent < node->parents.count &&
         PathContains(node->parents.ids[parent], ancestor, depth + 1u);
}

bool SG200XRCC::HasEnabledDependent(ClockId clock) const noexcept
{
  for (const auto& node : Tree::NODES)
  {
    if (node.id != clock && node.gate.Exists() && detail::RccField(node.gate) != 0u &&
        PathContains(node.id, clock, 0u))
      return true;
  }
  return false;
}

void SG200XRCC::RememberPath(ClockId clock, Transaction& transaction,
                             uint8_t depth) noexcept
{
  if (depth >= Tree::MAX_CLOCK_DEPTH) return;
  const auto* node = Tree::Find(clock);
  const std::size_t index = Tree::NodeIndex(clock);
  if (node == nullptr || index == Tree::NODES.size()) return;
  transaction.configured[index] = true;
  if (node->parents.count == 0u) return;
  const uint8_t parent = node->parents.count == 1u ? 0u : detail::RccParentIndex(*node);
  if (parent < node->parents.count)
    RememberPath(node->parents.ids[parent], transaction, depth + 1u);
}

ErrorCode SG200XRCC::ApplyClockPlanLocked(const ClockRatePlan& plan,
                                          Transaction& transaction, bool startup) noexcept
{
  if (!Tree::IsClockPlanWellFormed(plan)) return ErrorCode::ARG_ERR;
  const auto* node = Tree::Find(plan.rate_clock);
  if (ClockRateRecursive(plan.parent, 0u) != plan.parent_rate_hz)
    return ErrorCode::CHECK_ERR;
  const bool matches = PlanMatchesHardware(plan);
  const bool system_bus = Tree::IsSystemBusClock(node->id);
  const bool was_enabled = detail::RccField(node->gate) != 0u;
  if (system_bus && !matches && (!startup || was_enabled)) return ErrorCode::NOT_SUPPORT;
  if (!matches && was_enabled && !system_bus && HasEnabledDependent(node->id))
    return ErrorCode::BUSY;

  ErrorCode result = EnableClockPathLocked(plan.parent, transaction, 0u, false);
  if (result != ErrorCode::OK) return result;
  if (!matches)
  {
    if (was_enabled && !system_bus && !transaction.Write(node->gate, 0u))
      return ErrorCode::CHECK_ERR;
    if (node->bypass.Exists() && plan.parent_index == 0u)
    {
      if (!transaction.Write(node->bypass, 1u)) return ErrorCode::CHECK_ERR;
    }
    else
    {
      if (!transaction.Write(node->divider0.field, plan.divider) ||
          !transaction.Write(Tree::Field(node->divider0.field.offset, 3u), 1u) ||
          !transaction.Write(Tree::Field(node->divider0.field.offset, 0u), 1u))
        return ErrorCode::CHECK_ERR;
      const uint32_t selection = plan.parent_index - (node->bypass.Exists() ? 1u : 0u);
      if (node->source_select.Exists() &&
          !transaction.Write(node->source_select, selection))
        return ErrorCode::CHECK_ERR;
      if (!node->source_select.Exists() && selection != 0u) return ErrorCode::STATE_ERR;
      if (node->bypass.Exists() && !transaction.Write(node->bypass, 0u))
        return ErrorCode::CHECK_ERR;
    }
    if (was_enabled && !system_bus && !transaction.Write(node->gate, 1u))
      return ErrorCode::CHECK_ERR;
  }
  if (ClockRateRecursive(plan.clock, 0u) != plan.actual_rate_hz ||
      ClockRateRecursive(plan.parent, 0u) != plan.parent_rate_hz)
    return ErrorCode::CHECK_ERR;
  RememberPath(plan.clock, transaction, 0u);
  return ErrorCode::OK;
}

ErrorCode SG200XRCC::ApplyClockPlan(const ClockRatePlan& plan) noexcept
{
  if (!Tree::IsClockPlanWellFormed(plan)) return ErrorCode::ARG_ERR;
  if (!TryBeginWrite()) return ErrorCode::BUSY;
  Transaction transaction(*this);
  const ErrorCode result = ApplyClockPlanLocked(plan, transaction, false);
  if (result == ErrorCode::OK) transaction.Commit();
  return result;
}

ErrorCode SG200XRCC::SetRate(ClockId clock, uint32_t target_rate_hz,
                             RatePolicy policy) noexcept
{
  if (!TryBeginWrite()) return ErrorCode::BUSY;
  Transaction transaction(*this);
  const ClockRatePlan plan = PlanRateLocked(clock, target_rate_hz, policy);
  const ErrorCode result = plan.IsValid() ? ApplyClockPlanLocked(plan, transaction, false)
                                          : PlanError(plan.status);
  if (result == ErrorCode::OK) transaction.Commit();
  return result;
}

ErrorCode SG200XRCC::SetRate(PeripheralId peripheral, uint32_t target_rate_hz,
                             RatePolicy policy) noexcept
{
  const auto* resource = Tree::Find(peripheral);
  return resource == nullptr ? ErrorCode::ARG_ERR
                             : SetRate(resource->rate_clock, target_rate_hz, policy);
}

ErrorCode SG200XRCC::InitializeClockLocked(ClockId clock, Transaction& transaction,
                                           uint8_t depth) noexcept
{
  if (depth >= Tree::MAX_CLOCK_DEPTH) return ErrorCode::STATE_ERR;
  const std::size_t index = Tree::NodeIndex(clock);
  if (index == Tree::NODES.size()) return ErrorCode::ARG_ERR;
  if (transaction.configured[index]) return ErrorCode::OK;
  const auto* plan = Tree::DEFAULT_C906L_CLOCK_PLAN.Find(clock);
  if (plan == nullptr) return ErrorCode::OK;
  const ErrorCode result = InitializeClockLocked(plan->parent, transaction, depth + 1u);
  return result == ErrorCode::OK ? ApplyClockPlanLocked(*plan, transaction, true)
                                 : result;
}

ErrorCode SG200XRCC::EnableClockPathLocked(ClockId clock, Transaction& transaction,
                                           uint8_t depth, bool defaults) noexcept
{
  if (depth >= Tree::MAX_CLOCK_DEPTH) return ErrorCode::STATE_ERR;
  const auto* node = Tree::Find(clock);
  if (node == nullptr) return ErrorCode::ARG_ERR;
  if (defaults)
  {
    const ErrorCode result = InitializeClockLocked(clock, transaction, depth);
    if (result != ErrorCode::OK) return result;
  }
  if (node->kind == Tree::NodeKind::G2Pll) return ErrorCode::NOT_SUPPORT;
  if (node->kind == Tree::NodeKind::Fixed || node->kind == Tree::NodeKind::G6Pll)
    return ClockRateRecursive(clock, 0u) != 0u ? ErrorCode::OK : ErrorCode::STATE_ERR;
  if (node->ownership == Tree::Ownership::RuntimeOnly)
    return IsClockEnabledRecursive(clock, 0u) ? ErrorCode::OK : ErrorCode::NOT_SUPPORT;
  const uint8_t parent = detail::RccParentIndex(*node);
  if (parent >= node->parents.count) return ErrorCode::STATE_ERR;
  const ErrorCode result =
      EnableClockPathLocked(node->parents.ids[parent], transaction, depth + 1u, defaults);
  if (result != ErrorCode::OK) return result;
  if (node->divider0.Exists() && !(node->bypass.Exists() && parent == 0u) &&
      !transaction.Write(Tree::Field(node->divider0.field.offset, 0u), 1u))
    return ErrorCode::CHECK_ERR;
  return transaction.Write(node->gate, 1u) ? ErrorCode::OK : ErrorCode::CHECK_ERR;
}

ErrorCode SG200XRCC::EnableClock(ClockId clock) noexcept
{
  if (Tree::Find(clock) == nullptr) return ErrorCode::ARG_ERR;
  if (!TryBeginWrite()) return ErrorCode::BUSY;
  Transaction transaction(*this);
  const ErrorCode result = EnableClockPathLocked(clock, transaction, 0u, true);
  if (result == ErrorCode::OK) transaction.Commit();
  return result;
}

ErrorCode SG200XRCC::DisableClock(ClockId clock) noexcept
{
  const auto* node = Tree::Find(clock);
  if (node == nullptr) return ErrorCode::ARG_ERR;
  if (node->ownership != Tree::Ownership::C906LManaged || !node->gate.Exists() ||
      Tree::IsSystemBusClock(clock))
    return ErrorCode::NOT_SUPPORT;
  if (!TryBeginWrite()) return ErrorCode::BUSY;
  Transaction transaction(*this);
  if (HasEnabledDependent(clock)) return ErrorCode::BUSY;
  if (!transaction.Write(node->gate, 0u)) return ErrorCode::CHECK_ERR;
  RememberPath(clock, transaction, 0u);
  transaction.Commit();
  return ErrorCode::OK;
}

bool SG200XRCC::IsClockEnabledRecursive(ClockId clock, uint8_t depth) const noexcept
{
  if (depth >= Tree::MAX_CLOCK_DEPTH) return false;
  const auto* node = Tree::Find(clock);
  if (node == nullptr || (node->gate.Exists() && detail::RccField(node->gate) == 0u))
    return false;
  if (node->kind == Tree::NodeKind::Fixed || node->kind == Tree::NodeKind::G6Pll)
    return ClockRateRecursive(clock, 0u) != 0u;
  if (node->kind == Tree::NodeKind::G2Pll) return false;
  const uint8_t parent = detail::RccParentIndex(*node);
  if (parent >= node->parents.count) return false;
  const Tree::Divider& divider =
      node->path_select.Exists() && detail::RccField(node->path_select) == 0u
          ? node->divider1
          : node->divider0;
  if (divider.Exists() && !(node->bypass.Exists() && parent == 0u) &&
      !sgll_rcc_div_reset_is_deasserted(divider.field.offset))
    return false;
  return IsClockEnabledRecursive(node->parents.ids[parent], depth + 1u);
}

bool SG200XRCC::IsClockEnabled(ClockId clock) const noexcept
{
  const uint32_t before = write_sequence_.load(std::memory_order_acquire);
  if ((before & 1u) != 0u) return false;
  const bool enabled = IsClockEnabledRecursive(clock, 0u);
  sgll_csr_fence_io();
  return before == write_sequence_.load(std::memory_order_acquire) && enabled;
}

ErrorCode SG200XRCC::ReleaseReset(ResetId reset) noexcept
{
  if (!Tree::IsKnownReset(reset)) return ErrorCode::ARG_ERR;
  if (!TryBeginWrite()) return ErrorCode::BUSY;
  Transaction transaction(*this);
  const ErrorCode result = ReleaseResetLocked(reset);
  if (result == ErrorCode::OK) transaction.Commit();
  return result;
}

ErrorCode SG200XRCC::PreparePeripheral(PeripheralId peripheral) noexcept
{
  const auto* resource = Tree::Find(peripheral);
  if (resource == nullptr) return ErrorCode::ARG_ERR;
  if (!TryBeginWrite()) return ErrorCode::BUSY;
  Transaction transaction(*this);
  for (uint8_t index = 0u; index < resource->clock_count; ++index)
  {
    const ErrorCode result =
        EnableClockPathLocked(resource->clocks[index], transaction, 0u, true);
    if (result != ErrorCode::OK) return result;
  }
  const ErrorCode result = ReleaseResetLocked(resource->reset);
  if (result == ErrorCode::OK) transaction.Commit();
  return result;
}

ErrorCode SG200XRCC::ResetPeripheral(PeripheralId peripheral) noexcept
{
  const auto* resource = Tree::Find(peripheral);
  if (resource == nullptr) return ErrorCode::ARG_ERR;
  if (!TryBeginWrite()) return ErrorCode::BUSY;
  Transaction transaction(*this);
  for (uint8_t index = 0u; index < resource->clock_count; ++index)
  {
    const ErrorCode result =
        EnableClockPathLocked(resource->clocks[index], transaction, 0u, true);
    if (result != ErrorCode::OK) return result;
  }
  const ErrorCode result = PulseResetLocked(resource->reset);
  if (result == ErrorCode::OK) transaction.Commit();
  return result;
}

uint32_t SG200XRCC::ClockRate(ClockId clock) const noexcept
{
  const uint32_t before = write_sequence_.load(std::memory_order_acquire);
  if ((before & 1u) != 0u) return 0u;
  const uint32_t rate = ClockRateRecursive(clock, 0u);
  sgll_csr_fence_io();
  return before == write_sequence_.load(std::memory_order_acquire) ? rate : 0u;
}

uint32_t SG200XRCC::ClockRate(PeripheralId peripheral) const noexcept
{
  const auto* resource = Tree::Find(peripheral);
  return resource == nullptr ? 0u : ClockRate(resource->rate_clock);
}

ErrorCode SG200XRCC::ReleaseResetLocked(ResetId reset) noexcept
{
  const auto target = Tree::DeviceResetTarget(reset);
  if (target == RESET_NONE) return ErrorCode::ARG_ERR;
  sgll_rcc_reset_release(target);
  sgll_csr_fence_io();
  return sgll_rcc_reset_is_released(target) ? ErrorCode::OK : ErrorCode::CHECK_ERR;
}

ErrorCode SG200XRCC::PulseResetLocked(ResetId reset) noexcept
{
  const auto target = Tree::DeviceResetTarget(reset);
  if (target == RESET_NONE) return ErrorCode::ARG_ERR;
  sgll_rcc_reset_assert(target);
  sgll_csr_fence_io();
  const bool asserted = !sgll_rcc_reset_is_released(target);
  sgll_rcc_reset_release(target);
  sgll_csr_fence_io();
  return asserted && sgll_rcc_reset_is_released(target) ? ErrorCode::OK
                                                        : ErrorCode::CHECK_ERR;
}

uint32_t SG200XRCC::ClockRateRecursive(ClockId clock, uint8_t depth) const noexcept
{
  if (depth >= SG200XClockTree::MAX_CLOCK_DEPTH)
  {
    return 0u;
  }
  const auto* node = SG200XClockTree::Find(clock);
  if (node == nullptr)
  {
    return 0u;
  }
  using NodeKind = SG200XClockTree::NodeKind;
  if (node->kind == NodeKind::Fixed)
  {
    return node->id == ClockId::Oscillator ? SG200XClockTree::OSCILLATOR_HZ : 0u;
  }
  if (node->kind == NodeKind::G6Pll)
  {
    return detail::RccG6PllRate(node->pll_csr);
  }
  if (node->kind == NodeKind::G2Pll)
  {
    return 0u;
  }
  if (node->kind == NodeKind::Gate)
  {
    return node->parents.count == 1u
               ? ClockRateRecursive(node->parents.ids[0], depth + 1u)
               : 0u;
  }
  uint8_t parent = 0u;
  const SG200XClockTree::Divider* divider = &node->divider0;
  if (node->bypass.Exists() && detail::RccField(node->bypass) != 0u)
  {
    parent = 0u;
    divider = nullptr;
  }
  else if (node->path_select.Exists() && detail::RccField(node->path_select) == 0u)
  {
    parent = 1u;
    divider = &node->divider1;
  }
  else if (node->path_select.Exists())
  {
    parent = static_cast<uint8_t>(2u + detail::RccField(node->source_select));
  }
  else
  {
    parent = node->source_select.Exists()
                 ? static_cast<uint8_t>((node->bypass.Exists() ? 1u : 0u) +
                                        detail::RccField(node->source_select))
                 : static_cast<uint8_t>(node->parents.count == 1u ? 0u : 1u);
  }
  if (parent >= node->parents.count)
  {
    return 0u;
  }
  const uint32_t parent_rate = ClockRateRecursive(node->parents.ids[parent], depth + 1u);
  if (parent_rate == 0u || divider == nullptr)
  {
    return parent_rate;
  }
  const uint32_t divisor = detail::RccDividerValue(*divider);
  return divisor == 0u ? 0u : parent_rate / divisor;
}

}  // namespace LibXR
