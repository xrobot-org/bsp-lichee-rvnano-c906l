#pragma once

#include <array>
#include <cstdint>

namespace LibXR::SG200XClockTree
{

// This is the C906-visible part of the CV181x/SG200x TOP clock provider.
// Numeric IDs intentionally match include/dt-bindings/clock/cv181x-clock.h.
enum class ClockId : uint16_t
{
  None = 0xFFFFu,
  Oscillator = 0xFFFEu,
  Mpll = 0u,
  Tpll = 1u,
  Fpll = 2u,
  MipiMpll = 3u,
  A0Pll = 4u,
  DisplayPll = 5u,
  CpuAxi0 = 12u,
  Rtc25M = 19u,
  TemperatureSensor = 20u,
  SarAdc = 21u,
  XtalMisc = 25u,
  SdmaAxi = 44u,
  ApbI2c = 49u,
  ApbWatchdog = 50u,
  Pwm = 51u,
  ApbSpi0 = 52u,
  ApbSpi1 = 53u,
  ApbSpi2 = 54u,
  ApbSpi3 = 55u,
  Axi4 = 76u,
  Axi6 = 77u,
  Clock1M = 112u,
  Spi = 113u,
  I2c = 114u,
  Timer0 = 116u,
  Timer1 = 117u,
  Timer2 = 118u,
  Timer3 = 119u,
  Timer4 = 120u,
  Timer5 = 121u,
  Timer6 = 122u,
  Timer7 = 123u,
  ApbI2c0 = 124u,
  ApbI2c1 = 125u,
  ApbI2c2 = 126u,
  ApbI2c3 = 127u,
  ApbI2c4 = 128u,
  PwmSource = 143u,
  C906_0 = 152u,
  C906_1 = 153u,
};

enum class NodeKind : uint8_t
{
  Fixed,
  G6Pll,
  G2Pll,
  Gate,
  MuxDividerGate,
};

enum class Ownership : uint8_t
{
  // The source is fixed by the board contract or cannot safely be changed by
  // the running C906L core (PLL roots and C906 core clocks).
  RuntimeOnly,
  // C906L may select its parent, divider, gate, and peripheral reset.
  C906LManaged,
};

enum class PeripheralId : uint8_t
{
  Sdma,
  Spi0,
  Spi1,
  Spi2,
  Spi3,
  I2c0,
  I2c1,
  I2c2,
  I2c3,
  I2c4,
  Pwm0,
  Pwm1,
  Pwm2,
  Pwm3,
  SarAdc,
  Watchdog,
  Count,
};

// Reset IDs use the linear active-low IDs from cv181x-resets.h. They are
// deliberately separate from ClockId: reset lines are resource controls, not
// clock-tree edges.
enum class ResetId : uint16_t
{
  Sdma = 18u,
  I2c0 = 27u,
  I2c1 = 28u,
  I2c2 = 29u,
  I2c3 = 30u,
  I2c4 = 31u,
  Pwm0 = 32u,
  Pwm1 = 33u,
  Pwm2 = 34u,
  Pwm3 = 35u,
  Spi0 = 40u,
  Spi1 = 41u,
  Spi2 = 42u,
  Spi3 = 43u,
  Watchdog = 48u,
  SarAdc = 52u,
};

inline constexpr uint32_t OSCILLATOR_HZ = 25000000u;
// This is the C906L board contract for the reference SG200x image. It is an
// input to the consteval planner, not a request to retune the running PLL.
inline constexpr uint32_t C906L_FPLL_HZ = 1500000000u;
inline constexpr uint16_t NO_REGISTER = 0xFFFFu;
inline constexpr uint16_t CLOCK_GEN_REGISTER_BYTES = 0x1000u;
inline constexpr uint16_t RESET_LINE_COUNT = 64u;
inline constexpr uint8_t MAX_CLOCK_DEPTH = 8u;

struct RegisterField
{
  uint16_t offset = NO_REGISTER;
  uint8_t shift = 0u;
  uint8_t width = 0u;

  [[nodiscard]] constexpr bool Exists() const noexcept { return offset != NO_REGISTER; }
};

struct Divider
{
  RegisterField field{};
  // The Linux provider uses this value until bit 3 of the divider register is
  // set. It is a hardware reset/default contract, not a requested firmware
  // rate.
  uint8_t reset_value = 0u;

  [[nodiscard]] constexpr bool Exists() const noexcept { return field.Exists(); }
};

struct ParentSet
{
  std::array<ClockId, 6u> ids{ClockId::None, ClockId::None, ClockId::None,
                              ClockId::None, ClockId::None, ClockId::None};
  uint8_t count = 0u;
};

struct ClockNode
{
  ClockId id = ClockId::None;
  NodeKind kind = NodeKind::Fixed;
  const char* name = "";
  ParentSet parents{};
  RegisterField gate{};
  RegisterField bypass{};
  // On nodes with two divider paths (the C906 cores), a clear hardware bit
  // selects divider1/parent1; a set bit selects divider0/source parents.
  RegisterField path_select{};
  RegisterField source_select{};
  Divider divider0{};
  Divider divider1{};
  uint16_t pll_csr = NO_REGISTER;
  Ownership ownership = Ownership::RuntimeOnly;
};

struct PeripheralResources
{
  PeripheralId peripheral{};
  std::array<ClockId, 2u> clocks{ClockId::None, ClockId::None};
  uint8_t clock_count = 0u;
  ResetId reset{};
};

[[nodiscard]] constexpr RegisterField Field(uint16_t offset, uint8_t shift,
                                            uint8_t width = 1u) noexcept
{
  return {offset, shift, width};
}

[[nodiscard]] constexpr Divider Divide(uint16_t offset, uint8_t shift, uint8_t width,
                                        uint8_t reset_value) noexcept
{
  return {{offset, shift, width}, reset_value};
}

[[nodiscard]] constexpr ParentSet Parents(ClockId parent0 = ClockId::None,
                                          ClockId parent1 = ClockId::None,
                                          ClockId parent2 = ClockId::None,
                                          ClockId parent3 = ClockId::None,
                                          ClockId parent4 = ClockId::None,
                                          ClockId parent5 = ClockId::None) noexcept
{
  const std::array<ClockId, 6u> parents{parent0, parent1, parent2,
                                         parent3, parent4, parent5};
  uint8_t count = 0u;
  for (const ClockId parent : parents)
  {
    if (parent != ClockId::None)
    {
      ++count;
    }
  }
  return {parents, count};
}

[[nodiscard]] constexpr ClockNode Fixed(ClockId id, const char* name,
                                        RegisterField gate = {}) noexcept
{
  return {id, NodeKind::Fixed, name, {}, gate, {}, {}, {}, {}, {}, NO_REGISTER,
          Ownership::RuntimeOnly};
}

[[nodiscard]] constexpr ClockNode Gate(ClockId id, const char* name, ClockId parent,
                                       RegisterField gate) noexcept
{
  return {id, NodeKind::Gate, name, Parents(parent), gate, {}, {}, {}, {}, {},
          NO_REGISTER, Ownership::C906LManaged};
}

[[nodiscard]] constexpr ClockNode G6Pll(ClockId id, const char* name,
                                        uint16_t csr) noexcept
{
  return {id, NodeKind::G6Pll, name, Parents(ClockId::Oscillator), {}, {}, {}, {},
          {}, {}, csr, Ownership::RuntimeOnly};
}

[[nodiscard]] constexpr ClockNode G2Pll(ClockId id, const char* name,
                                        ClockId parent) noexcept
{
  return {id, NodeKind::G2Pll, name, Parents(parent), {}, {}, {}, {},
          {}, {}, NO_REGISTER, Ownership::RuntimeOnly};
}

[[nodiscard]] constexpr ClockNode MuxDividerGate(
    ClockId id, const char* name, ParentSet parents, RegisterField gate,
    RegisterField bypass, RegisterField source_select, Divider divider0) noexcept
{
  return {id, NodeKind::MuxDividerGate, name, parents, gate, bypass, {}, source_select,
          divider0, {}, NO_REGISTER, Ownership::C906LManaged};
}

[[nodiscard]] constexpr ClockNode DualPathMuxDividerGate(
    ClockId id, const char* name, ParentSet parents, RegisterField gate,
    RegisterField bypass, RegisterField path_select, RegisterField source_select,
    Divider divider0, Divider divider1) noexcept
{
  return {id, NodeKind::MuxDividerGate, name, parents, gate, bypass, path_select,
          source_select, divider0, divider1, NO_REGISTER, Ownership::RuntimeOnly};
}

// This table is transcribed from the CV181x clock provider used by the SG200x
// reference image family. It covers every TOP branch exposed to current C906
// drivers and the roots needed to calculate their rates.
inline constexpr std::array<ClockNode, 41u> NODES{
    Fixed(ClockId::Oscillator, "osc"),
    G6Pll(ClockId::Mpll, "clk_mpll", 0x908u),
    G6Pll(ClockId::Tpll, "clk_tpll", 0x90Cu),
    G6Pll(ClockId::Fpll, "clk_fpll", 0x910u),
    G2Pll(ClockId::MipiMpll, "clk_mipimpll", ClockId::Oscillator),
    G2Pll(ClockId::A0Pll, "clk_a0pll", ClockId::MipiMpll),
    G2Pll(ClockId::DisplayPll, "clk_disppll", ClockId::MipiMpll),
    MuxDividerGate(ClockId::CpuAxi0, "clk_cpu_axi0",
                    Parents(ClockId::Oscillator, ClockId::Fpll, ClockId::DisplayPll),
                    Field(0x000u, 1u), Field(0x030u, 1u), Field(0x048u, 8u, 2u),
                    Divide(0x048u, 16u, 4u, 3u)),
    MuxDividerGate(ClockId::Axi4, "clk_axi4",
                    Parents(ClockId::Oscillator, ClockId::Fpll, ClockId::DisplayPll),
                    Field(0x008u, 1u), Field(0x030u, 19u), Field(0x0B8u, 8u, 2u),
                    Divide(0x0B8u, 16u, 4u, 5u)),
    MuxDividerGate(ClockId::Axi6, "clk_axi6",
                    Parents(ClockId::Oscillator, ClockId::Fpll), Field(0x008u, 2u),
                    Field(0x030u, 20u), {}, Divide(0x0BCu, 16u, 4u, 15u)),
    Gate(ClockId::Rtc25M, "clk_rtc_25m", ClockId::Oscillator, Field(0x000u, 8u)),
    Gate(ClockId::TemperatureSensor, "clk_tempsen", ClockId::Oscillator,
         Field(0x000u, 9u)),
    Gate(ClockId::SarAdc, "clk_saradc", ClockId::Oscillator, Field(0x000u, 10u)),
    Gate(ClockId::XtalMisc, "clk_xtal_misc", ClockId::Oscillator, Field(0x000u, 14u)),
    Gate(ClockId::SdmaAxi, "clk_sdma_axi", ClockId::Axi4, Field(0x004u, 1u)),
    Gate(ClockId::ApbI2c, "clk_apb_i2c", ClockId::Axi4, Field(0x004u, 6u)),
    Gate(ClockId::ApbWatchdog, "clk_apb_wdt", ClockId::Oscillator,
         Field(0x004u, 7u)),
    MuxDividerGate(ClockId::PwmSource, "clk_pwm_src",
                    Parents(ClockId::Oscillator, ClockId::Fpll, ClockId::DisplayPll),
                    Field(0x010u, 4u), Field(0x030u, 15u), Field(0x120u, 8u, 2u),
                    Divide(0x120u, 16u, 6u, 10u)),
    Gate(ClockId::Pwm, "clk_pwm", ClockId::PwmSource, Field(0x004u, 8u)),
    Gate(ClockId::ApbSpi0, "clk_apb_spi0", ClockId::Axi4, Field(0x004u, 9u)),
    Gate(ClockId::ApbSpi1, "clk_apb_spi1", ClockId::Axi4, Field(0x004u, 10u)),
    Gate(ClockId::ApbSpi2, "clk_apb_spi2", ClockId::Axi4, Field(0x004u, 11u)),
    Gate(ClockId::ApbSpi3, "clk_apb_spi3", ClockId::Axi4, Field(0x004u, 12u)),
    MuxDividerGate(ClockId::Clock1M, "clk_1m", Parents(ClockId::Oscillator),
                    Field(0x00Cu, 5u), {}, {}, Divide(0x0FCu, 16u, 6u, 25u)),
    MuxDividerGate(ClockId::Spi, "clk_spi", Parents(ClockId::Oscillator, ClockId::Fpll),
                    Field(0x00Cu, 6u), Field(0x030u, 30u), {},
                    Divide(0x100u, 16u, 6u, 8u)),
    MuxDividerGate(ClockId::I2c, "clk_i2c", Parents(ClockId::Oscillator, ClockId::Axi6),
                    Field(0x00Cu, 7u), Field(0x030u, 31u), {},
                    Divide(0x104u, 16u, 4u, 1u)),
    Gate(ClockId::Timer0, "clk_timer0", ClockId::XtalMisc, Field(0x00Cu, 9u)),
    Gate(ClockId::Timer1, "clk_timer1", ClockId::XtalMisc, Field(0x00Cu, 10u)),
    Gate(ClockId::Timer2, "clk_timer2", ClockId::XtalMisc, Field(0x00Cu, 11u)),
    Gate(ClockId::Timer3, "clk_timer3", ClockId::XtalMisc, Field(0x00Cu, 12u)),
    Gate(ClockId::Timer4, "clk_timer4", ClockId::XtalMisc, Field(0x00Cu, 13u)),
    Gate(ClockId::Timer5, "clk_timer5", ClockId::XtalMisc, Field(0x00Cu, 14u)),
    Gate(ClockId::Timer6, "clk_timer6", ClockId::XtalMisc, Field(0x00Cu, 15u)),
    Gate(ClockId::Timer7, "clk_timer7", ClockId::XtalMisc, Field(0x00Cu, 16u)),
    Gate(ClockId::ApbI2c0, "clk_apb_i2c0", ClockId::Axi4, Field(0x00Cu, 17u)),
    Gate(ClockId::ApbI2c1, "clk_apb_i2c1", ClockId::Axi4, Field(0x00Cu, 18u)),
    Gate(ClockId::ApbI2c2, "clk_apb_i2c2", ClockId::Axi4, Field(0x00Cu, 19u)),
    Gate(ClockId::ApbI2c3, "clk_apb_i2c3", ClockId::Axi4, Field(0x00Cu, 20u)),
    Gate(ClockId::ApbI2c4, "clk_apb_i2c4", ClockId::Axi4, Field(0x00Cu, 21u)),
    DualPathMuxDividerGate(
        ClockId::C906_0, "clk_c906_0",
        Parents(ClockId::Oscillator, ClockId::Fpll, ClockId::Tpll, ClockId::A0Pll,
                ClockId::MipiMpll, ClockId::Mpll),
        Field(0x010u, 13u), Field(0x034u, 6u), Field(0x020u, 23u),
        Field(0x130u, 8u, 2u), Divide(0x130u, 16u, 4u, 1u),
        Divide(0x134u, 16u, 4u, 2u)),
    DualPathMuxDividerGate(
        ClockId::C906_1, "clk_c906_1",
        Parents(ClockId::Oscillator, ClockId::Fpll, ClockId::Tpll, ClockId::A0Pll,
                ClockId::DisplayPll, ClockId::Mpll),
        Field(0x010u, 14u), Field(0x034u, 7u), Field(0x020u, 24u),
        Field(0x138u, 8u, 2u), Divide(0x138u, 16u, 4u, 2u),
        Divide(0x13Cu, 16u, 4u, 2u)),
};

inline constexpr std::array<PeripheralResources, 16u> PERIPHERALS{
    PeripheralResources{PeripheralId::Sdma, {ClockId::SdmaAxi, ClockId::None}, 1u,
                        ResetId::Sdma},
    PeripheralResources{PeripheralId::Spi0, {ClockId::Spi, ClockId::ApbSpi0}, 2u,
                        ResetId::Spi0},
    PeripheralResources{PeripheralId::Spi1, {ClockId::Spi, ClockId::ApbSpi1}, 2u,
                        ResetId::Spi1},
    PeripheralResources{PeripheralId::Spi2, {ClockId::Spi, ClockId::ApbSpi2}, 2u,
                        ResetId::Spi2},
    PeripheralResources{PeripheralId::Spi3, {ClockId::Spi, ClockId::ApbSpi3}, 2u,
                        ResetId::Spi3},
    PeripheralResources{PeripheralId::I2c0, {ClockId::I2c, ClockId::ApbI2c0}, 2u,
                        ResetId::I2c0},
    PeripheralResources{PeripheralId::I2c1, {ClockId::I2c, ClockId::ApbI2c1}, 2u,
                        ResetId::I2c1},
    PeripheralResources{PeripheralId::I2c2, {ClockId::I2c, ClockId::ApbI2c2}, 2u,
                        ResetId::I2c2},
    PeripheralResources{PeripheralId::I2c3, {ClockId::I2c, ClockId::ApbI2c3}, 2u,
                        ResetId::I2c3},
    PeripheralResources{PeripheralId::I2c4, {ClockId::I2c, ClockId::ApbI2c4}, 2u,
                        ResetId::I2c4},
    PeripheralResources{PeripheralId::Pwm0, {ClockId::Pwm, ClockId::None}, 1u,
                        ResetId::Pwm0},
    PeripheralResources{PeripheralId::Pwm1, {ClockId::Pwm, ClockId::None}, 1u,
                        ResetId::Pwm1},
    PeripheralResources{PeripheralId::Pwm2, {ClockId::Pwm, ClockId::None}, 1u,
                        ResetId::Pwm2},
    PeripheralResources{PeripheralId::Pwm3, {ClockId::Pwm, ClockId::None}, 1u,
                        ResetId::Pwm3},
    PeripheralResources{PeripheralId::SarAdc, {ClockId::SarAdc, ClockId::None}, 1u,
                        ResetId::SarAdc},
    PeripheralResources{PeripheralId::Watchdog, {ClockId::ApbWatchdog, ClockId::None},
                        1u, ResetId::Watchdog},
};

[[nodiscard]] constexpr const ClockNode* Find(ClockId id) noexcept
{
  for (const ClockNode& node : NODES)
  {
    if (node.id == id)
    {
      return &node;
    }
  }
  return nullptr;
}

[[nodiscard]] constexpr const PeripheralResources* Find(PeripheralId peripheral) noexcept
{
  for (const PeripheralResources& resource : PERIPHERALS)
  {
    if (resource.peripheral == peripheral)
    {
      return &resource;
    }
  }
  return nullptr;
}

[[nodiscard]] constexpr bool IsKnownReset(ResetId reset) noexcept
{
  for (const PeripheralResources& resource : PERIPHERALS)
  {
    if (resource.reset == reset)
    {
      return true;
    }
  }
  return false;
}

struct ClockRatePlan
{
  ClockId clock = ClockId::None;
  ClockId parent = ClockId::None;
  uint32_t parent_rate_hz = 0u;
  uint32_t target_rate_hz = 0u;
  uint32_t actual_rate_hz = 0u;
  uint8_t parent_index = 0u;
  uint8_t divider = 0u;
};

[[nodiscard]] constexpr uint8_t ParentIndex(const ClockNode& node, ClockId parent) noexcept
{
  for (uint8_t index = 0u; index < node.parents.count; ++index)
  {
    if (node.parents.ids[index] == parent)
    {
      return index;
    }
  }
  return 0xFFu;
}

[[nodiscard]] constexpr uint32_t DividerMaximum(const Divider& divider) noexcept
{
  return divider.field.width == 0u
             ? 0u
             : (static_cast<uint32_t>(1u) << divider.field.width) - 1u;
}

[[nodiscard]] constexpr uint64_t RateError(uint32_t actual, uint32_t target) noexcept
{
  return actual >= target ? static_cast<uint64_t>(actual - target)
                          : static_cast<uint64_t>(target - actual);
}

/**
 * Build a C906L-owned mux/divider setting at compile time.
 *
 * The vendor provider uses one-based divider fields. `ParentRateHz` is a
 * board contract, which deliberately makes a rate change reviewable at the
 * declaration site instead of hiding it in an SPI constructor.
 */
template <ClockId Clock, ClockId Parent, uint32_t ParentRateHz, uint32_t TargetRateHz>
[[nodiscard]] consteval ClockRatePlan MakeRatePlan()
{
  static_assert(ParentRateHz != 0u && TargetRateHz != 0u && TargetRateHz <= ParentRateHz,
                "SG200x clock-rate request is invalid");
  if constexpr (ParentRateHz == 0u || TargetRateHz == 0u || TargetRateHz > ParentRateHz)
  {
    return {};
  }
  else
  {
    constexpr const ClockNode* node = Find(Clock);
    static_assert(node != nullptr && node->kind == NodeKind::MuxDividerGate &&
                      !node->path_select.Exists() && node->divider0.Exists(),
                  "SG200x clock is not a C906L-configurable single-path divider");
    constexpr uint8_t parent_index = ParentIndex(*node, Parent);
    static_assert(parent_index != 0xFFu, "SG200x clock request names an invalid parent");
    static_assert((node->bypass.Exists() || parent_index == 0u ||
                   node->source_select.Exists()) &&
                      (node->source_select.Exists() || !node->bypass.Exists() ||
                       parent_index <= 1u),
                  "SG200x clock has no hardware route for the requested parent");
    constexpr uint32_t divider_maximum = DividerMaximum(node->divider0);
    static_assert(divider_maximum != 0u, "SG200x clock has an invalid divider field");

    uint32_t lower = ParentRateHz / TargetRateHz;
    if (lower == 0u)
    {
      lower = 1u;
    }
    if (lower > divider_maximum)
    {
      lower = divider_maximum;
    }
    uint32_t upper = lower;
    if (ParentRateHz / lower > TargetRateHz && lower < divider_maximum)
    {
      upper = lower + 1u;
    }

    const uint32_t lower_rate = ParentRateHz / lower;
    const uint32_t upper_rate = ParentRateHz / upper;
    const uint32_t divider = RateError(upper_rate, TargetRateHz) <
                                     RateError(lower_rate, TargetRateHz)
                                 ? upper
                                 : lower;
    return {Clock, Parent, ParentRateHz, TargetRateHz, ParentRateHz / divider,
            parent_index, static_cast<uint8_t>(divider)};
  }
}

struct C906LClockPlan
{
  uint32_t fpll_rate_hz = 0u;
  std::array<ClockRatePlan, 5u> clocks{};

  [[nodiscard]] constexpr const ClockRatePlan* Find(ClockId clock) const noexcept
  {
    for (const ClockRatePlan& plan : clocks)
    {
      if (plan.clock == clock)
      {
        return &plan;
      }
    }
    return nullptr;
  }
};

inline constexpr ClockRatePlan C906L_AXI4_PLAN =
    MakeRatePlan<ClockId::Axi4, ClockId::Fpll, C906L_FPLL_HZ, 300000000u>();
inline constexpr ClockRatePlan C906L_AXI6_PLAN =
    MakeRatePlan<ClockId::Axi6, ClockId::Fpll, C906L_FPLL_HZ, 100000000u>();
inline constexpr ClockRatePlan C906L_1M_PLAN =
    MakeRatePlan<ClockId::Clock1M, ClockId::Oscillator, OSCILLATOR_HZ, 1000000u>();
inline constexpr ClockRatePlan C906L_SPI_PLAN =
    MakeRatePlan<ClockId::Spi, ClockId::Fpll, C906L_FPLL_HZ, 187500000u>();
inline constexpr ClockRatePlan C906L_I2C_PLAN =
    MakeRatePlan<ClockId::I2c, ClockId::Axi6, C906L_AXI6_PLAN.actual_rate_hz, 100000000u>();

// The whole C906L peripheral domain is visible here. The RCC applies only a
// setting that is needed by the peripheral being prepared, recursively
// applying planned C906L parents first.
inline constexpr C906LClockPlan DEFAULT_C906L_CLOCK_PLAN{
    C906L_FPLL_HZ,
    {C906L_AXI4_PLAN, C906L_AXI6_PLAN, C906L_1M_PLAN, C906L_SPI_PLAN,
     C906L_I2C_PLAN},
};

static_assert(C906L_SPI_PLAN.divider == 8u &&
                  C906L_SPI_PLAN.actual_rate_hz == 187500000u,
              "C906L SPI profile must produce the documented 187.5 MHz SSI clock");
static_assert(C906L_I2C_PLAN.divider == 1u &&
                  C906L_I2C_PLAN.actual_rate_hz == 100000000u,
              "C906L I2C profile must retain its documented 100 MHz input clock");

[[nodiscard]] constexpr bool IsFieldWellFormed(const RegisterField& field) noexcept
{
  if (!field.Exists())
  {
    return field.shift == 0u && field.width == 0u;
  }
  return field.offset < CLOCK_GEN_REGISTER_BYTES &&
         (field.offset % sizeof(uint32_t)) == 0u && field.width != 0u &&
         field.width <= 32u && field.shift < 32u &&
         static_cast<uint16_t>(field.shift) + field.width <= 32u;
}

[[nodiscard]] constexpr bool IsParentSetWellFormed(const ParentSet& parents) noexcept
{
  if (parents.count > parents.ids.size())
  {
    return false;
  }
  for (uint8_t index = 0u; index < parents.ids.size(); ++index)
  {
    if ((index < parents.count) != (parents.ids[index] != ClockId::None))
    {
      return false;
    }
    for (uint8_t previous = 0u; previous < index; ++previous)
    {
      if (parents.ids[index] != ClockId::None &&
          parents.ids[index] == parents.ids[previous])
      {
        return false;
      }
    }
  }
  return true;
}

[[nodiscard]] constexpr bool IsNodeWellFormed(const ClockNode& node) noexcept
{
  if (node.id == ClockId::None || node.name == nullptr || node.name[0] == '\0' ||
      !IsParentSetWellFormed(node.parents) || !IsFieldWellFormed(node.gate) ||
      !IsFieldWellFormed(node.bypass) || !IsFieldWellFormed(node.path_select) ||
      !IsFieldWellFormed(node.source_select) ||
      !IsFieldWellFormed(node.divider0.field) ||
      !IsFieldWellFormed(node.divider1.field) ||
      (node.gate.Exists() && node.gate.width != 1u) ||
      (node.bypass.Exists() && node.bypass.width != 1u) ||
      (node.path_select.Exists() && node.path_select.width != 1u) ||
      (node.divider0.reset_value > DividerMaximum(node.divider0)) ||
      (node.divider1.reset_value > DividerMaximum(node.divider1)) ||
      (node.divider0.reset_value != 0u && node.divider0.field.shift <= 3u &&
       static_cast<uint16_t>(node.divider0.field.shift) +
               node.divider0.field.width >
           3u) ||
      (node.divider1.reset_value != 0u && node.divider1.field.shift <= 3u &&
       static_cast<uint16_t>(node.divider1.field.shift) +
               node.divider1.field.width >
           3u))
  {
    return false;
  }

  switch (node.kind)
  {
    case NodeKind::Fixed:
      return node.parents.count == 0u && !node.gate.Exists() &&
             !node.bypass.Exists() && !node.path_select.Exists() &&
             !node.source_select.Exists() &&
             node.pll_csr == NO_REGISTER && !node.divider0.Exists() &&
             !node.divider1.Exists() && node.ownership == Ownership::RuntimeOnly;
    case NodeKind::G6Pll:
      return node.parents.count == 1u &&
             node.parents.ids[0] == ClockId::Oscillator && !node.gate.Exists() &&
             !node.bypass.Exists() && !node.path_select.Exists() &&
             !node.source_select.Exists() && node.pll_csr != NO_REGISTER &&
             node.pll_csr < CLOCK_GEN_REGISTER_BYTES &&
             (node.pll_csr % sizeof(uint32_t)) == 0u && !node.divider0.Exists() &&
             !node.divider1.Exists() && node.ownership == Ownership::RuntimeOnly;
    case NodeKind::G2Pll:
      return node.parents.count == 1u && !node.gate.Exists() &&
             !node.bypass.Exists() && !node.path_select.Exists() &&
             !node.source_select.Exists() &&
             node.pll_csr == NO_REGISTER && !node.divider0.Exists() &&
             !node.divider1.Exists() && node.ownership == Ownership::RuntimeOnly;
    case NodeKind::Gate:
      return node.parents.count == 1u && node.gate.Exists() &&
             node.pll_csr == NO_REGISTER && !node.bypass.Exists() &&
             !node.path_select.Exists() && !node.source_select.Exists() &&
             !node.divider0.Exists() && !node.divider1.Exists() &&
             node.ownership == Ownership::C906LManaged;
    case NodeKind::MuxDividerGate:
    {
      const uint32_t single_path_capacity =
          (node.bypass.Exists() ? 1u : 0u) +
          (node.source_select.Exists()
               ? (static_cast<uint32_t>(1u) << node.source_select.width)
               : 1u);
      const uint32_t dual_path_capacity =
          2u + (node.source_select.Exists()
                    ? (static_cast<uint32_t>(1u) << node.source_select.width)
                    : 0u);
      return node.parents.count != 0u && node.gate.Exists() &&
             node.pll_csr == NO_REGISTER && node.divider0.Exists() &&
             (!node.path_select.Exists() ||
              (node.parents.count >= 2u && node.bypass.Exists() &&
               node.source_select.Exists() && node.divider1.Exists() &&
               node.parents.count <= dual_path_capacity)) &&
             (node.path_select.Exists() ||
              (!node.divider1.Exists() &&
               node.parents.count <= single_path_capacity)) &&
             node.ownership == (node.path_select.Exists()
                                    ? Ownership::RuntimeOnly
                                    : Ownership::C906LManaged);
    }
  }
  return false;
}

[[nodiscard]] constexpr bool ReachesClock(ClockId current, ClockId target,
                                          uint8_t depth) noexcept
{
  if (depth >= NODES.size())
  {
    return true;
  }
  const ClockNode* node = Find(current);
  if (node == nullptr)
  {
    return false;
  }
  for (uint8_t index = 0u; index < node->parents.count; ++index)
  {
    if (node->parents.ids[index] == target ||
        ReachesClock(node->parents.ids[index], target,
                     static_cast<uint8_t>(depth + 1u)))
    {
      return true;
    }
  }
  return false;
}

[[nodiscard]] constexpr bool FitsRuntimeDepth(ClockId current,
                                              uint8_t depth) noexcept
{
  if (depth >= MAX_CLOCK_DEPTH)
  {
    return false;
  }
  const ClockNode* node = Find(current);
  if (node == nullptr)
  {
    return false;
  }
  for (uint8_t index = 0u; index < node->parents.count; ++index)
  {
    if (!FitsRuntimeDepth(node->parents.ids[index],
                          static_cast<uint8_t>(depth + 1u)))
    {
      return false;
    }
  }
  return true;
}

[[nodiscard]] constexpr bool IsTopologyWellFormed() noexcept
{
  for (std::size_t node_index = 0u; node_index < NODES.size(); ++node_index)
  {
    const ClockNode& node = NODES[node_index];
    if (!IsNodeWellFormed(node) || !FitsRuntimeDepth(node.id, 0u))
    {
      return false;
    }
    for (std::size_t previous = 0u; previous < node_index; ++previous)
    {
      if (NODES[previous].id == node.id)
      {
        return false;
      }
    }
    for (uint8_t index = 0u; index < node.parents.count; ++index)
    {
      if (Find(node.parents.ids[index]) == nullptr ||
          node.parents.ids[index] == node.id ||
          ReachesClock(node.parents.ids[index], node.id, 0u))
      {
        return false;
      }
    }
  }
  return true;
}

[[nodiscard]] constexpr bool ArePeripheralResourcesWellFormed() noexcept
{
  if (PERIPHERALS.size() != static_cast<std::size_t>(PeripheralId::Count))
  {
    return false;
  }
  for (std::size_t index = 0u; index < PERIPHERALS.size(); ++index)
  {
    const PeripheralResources& resource = PERIPHERALS[index];
    if (static_cast<std::size_t>(resource.peripheral) != index ||
        resource.clock_count == 0u ||
        resource.clock_count > resource.clocks.size() ||
        static_cast<uint16_t>(resource.reset) >= RESET_LINE_COUNT)
    {
      return false;
    }
    for (uint8_t clock_index = 0u; clock_index < resource.clocks.size(); ++clock_index)
    {
      if ((clock_index < resource.clock_count) !=
              (resource.clocks[clock_index] != ClockId::None) ||
          (clock_index < resource.clock_count &&
           Find(resource.clocks[clock_index]) == nullptr))
      {
        return false;
      }
      for (uint8_t previous = 0u; previous < clock_index; ++previous)
      {
        if (resource.clocks[clock_index] != ClockId::None &&
            resource.clocks[clock_index] == resource.clocks[previous])
        {
          return false;
        }
      }
    }
    for (std::size_t previous = 0u; previous < index; ++previous)
    {
      if (PERIPHERALS[previous].peripheral == resource.peripheral ||
          PERIPHERALS[previous].reset == resource.reset)
      {
        return false;
      }
    }
  }
  return true;
}

[[nodiscard]] constexpr bool IsClockPlanWellFormed() noexcept
{
  if (DEFAULT_C906L_CLOCK_PLAN.fpll_rate_hz != C906L_FPLL_HZ)
  {
    return false;
  }
  for (std::size_t index = 0u; index < DEFAULT_C906L_CLOCK_PLAN.clocks.size(); ++index)
  {
    const ClockRatePlan& plan = DEFAULT_C906L_CLOCK_PLAN.clocks[index];
    const ClockNode* node = Find(plan.clock);
    if (node == nullptr || node->ownership != Ownership::C906LManaged ||
        node->kind != NodeKind::MuxDividerGate || node->path_select.Exists() ||
        !node->divider0.Exists() || plan.parent_index >= node->parents.count ||
        node->parents.ids[plan.parent_index] != plan.parent || plan.parent_rate_hz == 0u ||
        plan.target_rate_hz == 0u || plan.target_rate_hz > plan.parent_rate_hz ||
        plan.divider == 0u || plan.divider > DividerMaximum(node->divider0) ||
        plan.actual_rate_hz != plan.parent_rate_hz / plan.divider)
    {
      return false;
    }
    for (std::size_t previous = 0u; previous < index; ++previous)
    {
      if (DEFAULT_C906L_CLOCK_PLAN.clocks[previous].clock == plan.clock)
      {
        return false;
      }
    }

    const ClockRatePlan* parent_plan = DEFAULT_C906L_CLOCK_PLAN.Find(plan.parent);
    if (parent_plan == nullptr && plan.parent != ClockId::Oscillator &&
        plan.parent != ClockId::Fpll)
    {
      return false;
    }
    if (parent_plan != nullptr && parent_plan->actual_rate_hz != plan.parent_rate_hz)
    {
      return false;
    }
    if (plan.parent == ClockId::Oscillator && plan.parent_rate_hz != OSCILLATOR_HZ)
    {
      return false;
    }
    if (plan.parent == ClockId::Fpll &&
        plan.parent_rate_hz != DEFAULT_C906L_CLOCK_PLAN.fpll_rate_hz)
    {
      return false;
    }
  }
  return true;
}

static_assert(IsTopologyWellFormed(), "SG200x clock topology is malformed");
static_assert(ArePeripheralResourcesWellFormed(),
              "SG200x peripheral clock/reset resources are malformed");
static_assert(IsClockPlanWellFormed(), "SG200x C906L clock plan is malformed");

}  // namespace LibXR::SG200XClockTree
