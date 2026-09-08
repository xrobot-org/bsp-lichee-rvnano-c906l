#pragma once

#include <array>
#include <cstdint>

#include "sg2002.h"

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
  Axi4Sd0 = 29u,
  Sd0 = 30u,
  Clock100kSd0 = 31u,
  Axi4Sd1 = 32u,
  Sd1 = 33u,
  Clock100kSd1 = 34u,
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
  Sdio0,
  Sdio1,
  Count,
};

// Derive linear reset IDs from the device map while preserving the public
// cv181x-resets.h numbering used by existing C++ callers.
[[nodiscard]] constexpr uint8_t ResetLine(rstgen_reset_target_t target) noexcept
{
  switch (target)
  {
#define SG200X_RESET_LINE(name, bank, bit)       \
  case RESET_##name:                             \
    return bank == RSTGEN_RESET_LOCATION_INVALID \
               ? 0u                              \
               : static_cast<uint8_t>(bank * 32u + bit);
    SG2002_RESET_MAP(SG200X_RESET_LINE)
#undef SG200X_RESET_LINE
    default:
      return 0u;
  }
}

// Reset IDs use the linear active-low IDs from cv181x-resets.h. They are
// deliberately separate from ClockId: reset lines are resource controls, not
// clock-tree edges.
enum class ResetId : uint8_t
{
  None = ResetLine(RESET_NONE),
  Sdma = ResetLine(RESET_SDMA),
  I2c0 = ResetLine(RESET_I2C0),
  I2c1 = ResetLine(RESET_I2C1),
  I2c2 = ResetLine(RESET_I2C2),
  I2c3 = ResetLine(RESET_I2C3),
  I2c4 = ResetLine(RESET_I2C4),
  Pwm0 = ResetLine(RESET_PWM0),
  Pwm1 = ResetLine(RESET_PWM1),
  Pwm2 = ResetLine(RESET_PWM2),
  Pwm3 = ResetLine(RESET_PWM3),
  Spi0 = ResetLine(RESET_SPI0),
  Spi1 = ResetLine(RESET_SPI1),
  Spi2 = ResetLine(RESET_SPI2),
  Spi3 = ResetLine(RESET_SPI3),
  Sd0 = ResetLine(RESET_SD0),
  Sd1 = ResetLine(RESET_SD1),
  Watchdog = ResetLine(RESET_WDT0),
  SarAdc = ResetLine(RESET_SARADC),
};

[[nodiscard]] constexpr rstgen_reset_target_t DeviceResetTarget(ResetId reset) noexcept
{
  switch (reset)
  {
    case ResetId::None:
      return RESET_NONE;
    case ResetId::Sdma:
      return RESET_SDMA;
    case ResetId::I2c0:
      return RESET_I2C0;
    case ResetId::I2c1:
      return RESET_I2C1;
    case ResetId::I2c2:
      return RESET_I2C2;
    case ResetId::I2c3:
      return RESET_I2C3;
    case ResetId::I2c4:
      return RESET_I2C4;
    case ResetId::Pwm0:
      return RESET_PWM0;
    case ResetId::Pwm1:
      return RESET_PWM1;
    case ResetId::Pwm2:
      return RESET_PWM2;
    case ResetId::Pwm3:
      return RESET_PWM3;
    case ResetId::Spi0:
      return RESET_SPI0;
    case ResetId::Spi1:
      return RESET_SPI1;
    case ResetId::Spi2:
      return RESET_SPI2;
    case ResetId::Spi3:
      return RESET_SPI3;
    case ResetId::Sd0:
      return RESET_SD0;
    case ResetId::Sd1:
      return RESET_SD1;
    case ResetId::Watchdog:
      return RESET_WDT0;
    case ResetId::SarAdc:
      return RESET_SARADC;
    default:
      return RESET_NONE;
  }
}

inline constexpr uint32_t OSCILLATOR_HZ = XTAL_FREQ_HZ;
// This is the C906L board contract for the reference SG200x image. It is an
// input to the rate planner, not a request to retune the running PLL.
inline constexpr uint32_t C906L_FPLL_HZ = 1500000000u;
inline constexpr uint16_t NO_REGISTER = 0xFFFFu;
inline constexpr uint16_t CLOCK_GEN_REGISTER_BYTES = CLKGEN_REGISTER_BYTES;
inline constexpr uint16_t RESET_LINE_COUNT = RSTGEN_SOFT_RSTN_COUNT * 32u;
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
  ResetId reset = ResetId::None;
  ClockId rate_clock = ClockId::None;
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
  return {id,          NodeKind::Fixed,       name, {}, gate, {}, {}, {}, {}, {},
          NO_REGISTER, Ownership::RuntimeOnly};
}

[[nodiscard]] constexpr ClockNode Gate(ClockId id, const char* name, ClockId parent,
                                       RegisterField gate) noexcept
{
  return {id,          NodeKind::Gate,
          name,        Parents(parent),
          gate,        {},
          {},          {},
          {},          {},
          NO_REGISTER, Ownership::C906LManaged};
}

[[nodiscard]] constexpr ClockNode G6Pll(ClockId id, const char* name,
                                        uint16_t csr) noexcept
{
  return {id,   NodeKind::G6Pll,
          name, Parents(ClockId::Oscillator),
          {},   {},
          {},   {},
          {},   {},
          csr,  Ownership::RuntimeOnly};
}

[[nodiscard]] constexpr ClockNode G2Pll(ClockId id, const char* name,
                                        ClockId parent) noexcept
{
  return {
      id,          NodeKind::G2Pll,       name, Parents(parent), {}, {}, {}, {}, {}, {},
      NO_REGISTER, Ownership::RuntimeOnly};
}

[[nodiscard]] constexpr ClockNode MuxDividerGate(ClockId id, const char* name,
                                                 ParentSet parents, RegisterField gate,
                                                 RegisterField bypass,
                                                 RegisterField source_select,
                                                 Divider divider0) noexcept
{
  return {id,          NodeKind::MuxDividerGate,
          name,        parents,
          gate,        bypass,
          {},          source_select,
          divider0,    {},
          NO_REGISTER, Ownership::C906LManaged};
}

[[nodiscard]] constexpr ClockNode DualPathMuxDividerGate(
    ClockId id, const char* name, ParentSet parents, RegisterField gate,
    RegisterField bypass, RegisterField path_select, RegisterField source_select,
    Divider divider0, Divider divider1) noexcept
{
  return {id,          NodeKind::MuxDividerGate,
          name,        parents,
          gate,        bypass,
          path_select, source_select,
          divider0,    divider1,
          NO_REGISTER, Ownership::RuntimeOnly};
}

// This table is transcribed from the CV181x clock provider used by the SG200x
// reference image family. It covers every TOP branch exposed to current C906
// drivers and the roots needed to calculate their rates.
inline constexpr std::array<ClockNode, 47u> NODES{
    Fixed(ClockId::Oscillator, "osc"),
    G6Pll(ClockId::Mpll, "clk_mpll", PLL_G6_MPLL_OFFSET),
    G6Pll(ClockId::Tpll, "clk_tpll", PLL_G6_TPLL_OFFSET),
    G6Pll(ClockId::Fpll, "clk_fpll", PLL_G6_FPLL_OFFSET),
    G2Pll(ClockId::MipiMpll, "clk_mipimpll", ClockId::Oscillator),
    G2Pll(ClockId::A0Pll, "clk_a0pll", ClockId::MipiMpll),
    G2Pll(ClockId::DisplayPll, "clk_disppll", ClockId::MipiMpll),
    MuxDividerGate(
        ClockId::CpuAxi0, "clk_cpu_axi0",
        Parents(ClockId::Oscillator, ClockId::Fpll, ClockId::DisplayPll),
        Field(CLKGEN_CLK_EN0_OFFSET, 1u), Field(CLKGEN_CLK_BYP0_OFFSET, 1u),
        Field(CLKGEN_DIV_OFF_CPU_AXI0, CLKGEN_DIV_SRC_SHIFT, CLKGEN_DIV_SRC_WIDTH),
        Divide(CLKGEN_DIV_OFF_CPU_AXI0, CLKGEN_DIV_FACTOR_SHIFT,
               CLKGEN_DIV_CPU_AXI0_FACTOR_WIDTH, CLKGEN_CPU_AXI0_INITIAL_FACTOR)),
    MuxDividerGate(ClockId::Axi4, "clk_axi4",
                   Parents(ClockId::Oscillator, ClockId::Fpll, ClockId::DisplayPll),
                   Field(CLKGEN_GATE_AXI4), Field(CLKGEN_BYPASS_AXI4),
                   Field(CLKGEN_DIV_OFF_AXI4, CLKGEN_DIV_SRC_SHIFT, CLKGEN_DIV_SRC_WIDTH),
                   Divide(CLKGEN_DIV_OFF_AXI4, CLKGEN_DIV_FACTOR_SHIFT,
                          CLKGEN_DIV_AXI4_FACTOR_WIDTH, CLKGEN_AXI4_INITIAL_FACTOR)),
    MuxDividerGate(ClockId::Axi6, "clk_axi6", Parents(ClockId::Oscillator, ClockId::Fpll),
                   Field(CLKGEN_GATE_AXI6), Field(CLKGEN_BYPASS_AXI6), {},
                   Divide(CLKGEN_DIV_OFF_AXI6, CLKGEN_DIV_FACTOR_SHIFT,
                          CLKGEN_DIV_AXI6_FACTOR_WIDTH, CLKGEN_AXI6_INITIAL_FACTOR)),
    Gate(ClockId::Rtc25M, "clk_rtc_25m", ClockId::Oscillator, Field(CLKGEN_GATE_RTC_25M)),
    Gate(ClockId::TemperatureSensor, "clk_tempsen", ClockId::Oscillator,
         Field(CLKGEN_GATE_TEMPSEN)),
    Gate(ClockId::SarAdc, "clk_saradc", ClockId::Oscillator, Field(CLKGEN_GATE_SARADC)),
    Gate(ClockId::XtalMisc, "clk_xtal_misc", ClockId::Oscillator,
         Field(CLKGEN_GATE_XTAL_MISC)),
    Gate(ClockId::Axi4Sd0, "clk_axi4_sd0", ClockId::Axi4,
         Field(CLKGEN_CLK_EN0_OFFSET, 18u)),
    MuxDividerGate(ClockId::Sd0, "clk_sd0",
                   Parents(ClockId::Oscillator, ClockId::Fpll, ClockId::DisplayPll),
                   Field(CLKGEN_CLK_EN0_OFFSET, 19u), Field(CLKGEN_CLK_BYP0_OFFSET, 6u),
                   Field(CLKGEN_DIV_OFF_SD0, CLKGEN_DIV_SRC_SHIFT, CLKGEN_DIV_SRC_WIDTH),
                   Divide(CLKGEN_DIV_OFF_SD0, CLKGEN_DIV_FACTOR_SHIFT,
                          CLKGEN_DIV_SD_FACTOR_WIDTH, CLKGEN_SD_INITIAL_FACTOR)),
    MuxDividerGate(
        ClockId::Clock100kSd0, "clk_100k_sd0", Parents(ClockId::Clock1M),
        Field(CLKGEN_CLK_EN0_OFFSET, 20u), {}, {},
        Divide(CLKGEN_DIV_OFF_100K_SD0, CLKGEN_DIV_FACTOR_SHIFT,
               CLKGEN_DIV_SD_100K_FACTOR_WIDTH, CLKGEN_SD_100K_INITIAL_FACTOR)),
    Gate(ClockId::Axi4Sd1, "clk_axi4_sd1", ClockId::Axi4,
         Field(CLKGEN_CLK_EN0_OFFSET, 21u)),
    MuxDividerGate(ClockId::Sd1, "clk_sd1",
                   Parents(ClockId::Oscillator, ClockId::Fpll, ClockId::DisplayPll),
                   Field(CLKGEN_CLK_EN0_OFFSET, 22u), Field(CLKGEN_CLK_BYP0_OFFSET, 7u),
                   Field(CLKGEN_DIV_OFF_SD1, CLKGEN_DIV_SRC_SHIFT, CLKGEN_DIV_SRC_WIDTH),
                   Divide(CLKGEN_DIV_OFF_SD1, CLKGEN_DIV_FACTOR_SHIFT,
                          CLKGEN_DIV_SD_FACTOR_WIDTH, CLKGEN_SD_INITIAL_FACTOR)),
    MuxDividerGate(
        ClockId::Clock100kSd1, "clk_100k_sd1", Parents(ClockId::Clock1M),
        Field(CLKGEN_CLK_EN0_OFFSET, 23u), {}, {},
        Divide(CLKGEN_DIV_OFF_100K_SD1, CLKGEN_DIV_FACTOR_SHIFT,
               CLKGEN_DIV_SD_100K_FACTOR_WIDTH, CLKGEN_SD_100K_INITIAL_FACTOR)),
    Gate(ClockId::SdmaAxi, "clk_sdma_axi", ClockId::Axi4, Field(CLKGEN_GATE_SDMA_AXI)),
    Gate(ClockId::ApbI2c, "clk_apb_i2c", ClockId::Axi4, Field(CLKGEN_GATE_APB_I2C)),
    Gate(ClockId::ApbWatchdog, "clk_apb_wdt", ClockId::Oscillator,
         Field(CLKGEN_GATE_APB_WDT)),
    MuxDividerGate(
        ClockId::PwmSource, "clk_pwm_src",
        Parents(ClockId::Oscillator, ClockId::Fpll, ClockId::DisplayPll),
        Field(CLKGEN_GATE_PWM_SRC), Field(CLKGEN_BYPASS_PWM_SRC),
        Field(CLKGEN_DIV_OFF_PWM_SRC, CLKGEN_DIV_SRC_SHIFT, CLKGEN_DIV_SRC_WIDTH),
        Divide(CLKGEN_DIV_OFF_PWM_SRC, CLKGEN_DIV_FACTOR_SHIFT,
               CLKGEN_DIV_PWM_SRC_FACTOR_WIDTH, CLKGEN_PWM_SRC_INITIAL_FACTOR)),
    Gate(ClockId::Pwm, "clk_pwm", ClockId::PwmSource, Field(CLKGEN_GATE_APB_PWM)),
    Gate(ClockId::ApbSpi0, "clk_apb_spi0", ClockId::Axi4, Field(CLKGEN_GATE_APB_SPI0)),
    Gate(ClockId::ApbSpi1, "clk_apb_spi1", ClockId::Axi4, Field(CLKGEN_GATE_APB_SPI1)),
    Gate(ClockId::ApbSpi2, "clk_apb_spi2", ClockId::Axi4, Field(CLKGEN_GATE_APB_SPI2)),
    Gate(ClockId::ApbSpi3, "clk_apb_spi3", ClockId::Axi4, Field(CLKGEN_GATE_APB_SPI3)),
    MuxDividerGate(ClockId::Clock1M, "clk_1m", Parents(ClockId::Oscillator),
                   Field(CLKGEN_GATE_1M), {}, {},
                   Divide(CLKGEN_DIV_OFF_1M, CLKGEN_DIV_FACTOR_SHIFT,
                          CLKGEN_DIV_1M_FACTOR_WIDTH, CLKGEN_1M_INITIAL_FACTOR)),
    MuxDividerGate(ClockId::Spi, "clk_spi", Parents(ClockId::Oscillator, ClockId::Fpll),
                   Field(CLKGEN_GATE_SPI), Field(CLKGEN_BYPASS_SPI), {},
                   Divide(CLKGEN_DIV_OFF_SPI, CLKGEN_DIV_FACTOR_SHIFT,
                          CLKGEN_DIV_SPI_FACTOR_WIDTH, CLKGEN_SPI_INITIAL_FACTOR)),
    MuxDividerGate(ClockId::I2c, "clk_i2c", Parents(ClockId::Oscillator, ClockId::Axi6),
                   Field(CLKGEN_GATE_I2C), Field(CLKGEN_BYPASS_I2C), {},
                   Divide(CLKGEN_DIV_OFF_I2C, CLKGEN_DIV_FACTOR_SHIFT,
                          CLKGEN_DIV_I2C_FACTOR_WIDTH, CLKGEN_I2C_INITIAL_FACTOR)),
    Gate(ClockId::Timer0, "clk_timer0", ClockId::XtalMisc,
         Field(CLKGEN_CLK_EN3_OFFSET, 9u)),
    Gate(ClockId::Timer1, "clk_timer1", ClockId::XtalMisc,
         Field(CLKGEN_CLK_EN3_OFFSET, 10u)),
    Gate(ClockId::Timer2, "clk_timer2", ClockId::XtalMisc,
         Field(CLKGEN_CLK_EN3_OFFSET, 11u)),
    Gate(ClockId::Timer3, "clk_timer3", ClockId::XtalMisc,
         Field(CLKGEN_CLK_EN3_OFFSET, 12u)),
    Gate(ClockId::Timer4, "clk_timer4", ClockId::XtalMisc,
         Field(CLKGEN_CLK_EN3_OFFSET, 13u)),
    Gate(ClockId::Timer5, "clk_timer5", ClockId::XtalMisc,
         Field(CLKGEN_CLK_EN3_OFFSET, 14u)),
    Gate(ClockId::Timer6, "clk_timer6", ClockId::XtalMisc,
         Field(CLKGEN_CLK_EN3_OFFSET, 15u)),
    Gate(ClockId::Timer7, "clk_timer7", ClockId::XtalMisc,
         Field(CLKGEN_CLK_EN3_OFFSET, 16u)),
    Gate(ClockId::ApbI2c0, "clk_apb_i2c0", ClockId::Axi4,
         Field(CLKGEN_CLK_EN3_OFFSET, 17u)),
    Gate(ClockId::ApbI2c1, "clk_apb_i2c1", ClockId::Axi4,
         Field(CLKGEN_CLK_EN3_OFFSET, 18u)),
    Gate(ClockId::ApbI2c2, "clk_apb_i2c2", ClockId::Axi4,
         Field(CLKGEN_CLK_EN3_OFFSET, 19u)),
    Gate(ClockId::ApbI2c3, "clk_apb_i2c3", ClockId::Axi4,
         Field(CLKGEN_CLK_EN3_OFFSET, 20u)),
    Gate(ClockId::ApbI2c4, "clk_apb_i2c4", ClockId::Axi4,
         Field(CLKGEN_CLK_EN3_OFFSET, 21u)),
    DualPathMuxDividerGate(
        ClockId::C906_0, "clk_c906_0",
        Parents(ClockId::Oscillator, ClockId::Fpll, ClockId::Tpll, ClockId::A0Pll,
                ClockId::MipiMpll, ClockId::Mpll),
        Field(CLKGEN_CLK_EN4_OFFSET, 13u), Field(CLKGEN_CLK_BYP1_OFFSET, 6u),
        Field(CLKGEN_CLK_SEL0_OFFSET, CLKGEN_SEL0_C906_0_SHIFT),
        Field(CLKGEN_DIV_OFF_C906_0_0, CLKGEN_DIV_SRC_SHIFT, CLKGEN_DIV_SRC_WIDTH),
        Divide(CLKGEN_DIV_OFF_C906_0_0, CLKGEN_DIV_FACTOR_SHIFT,
               CLKGEN_DIV_C906_FACTOR_WIDTH, CLKGEN_C906_0_0_INITIAL_FACTOR),
        Divide(CLKGEN_DIV_OFF_C906_0_1, CLKGEN_DIV_FACTOR_SHIFT,
               CLKGEN_DIV_C906_FACTOR_WIDTH, CLKGEN_C906_0_1_INITIAL_FACTOR)),
    DualPathMuxDividerGate(
        ClockId::C906_1, "clk_c906_1",
        Parents(ClockId::Oscillator, ClockId::Fpll, ClockId::Tpll, ClockId::A0Pll,
                ClockId::DisplayPll, ClockId::Mpll),
        Field(CLKGEN_CLK_EN4_OFFSET, 14u), Field(CLKGEN_CLK_BYP1_OFFSET, 7u),
        Field(CLKGEN_CLK_SEL0_OFFSET, CLKGEN_SEL0_C906_1_SHIFT),
        Field(CLKGEN_DIV_OFF_C906_1_0, CLKGEN_DIV_SRC_SHIFT, CLKGEN_DIV_SRC_WIDTH),
        Divide(CLKGEN_DIV_OFF_C906_1_0, CLKGEN_DIV_FACTOR_SHIFT,
               CLKGEN_DIV_C906_FACTOR_WIDTH, CLKGEN_C906_1_0_INITIAL_FACTOR),
        Divide(CLKGEN_DIV_OFF_C906_1_1, CLKGEN_DIV_FACTOR_SHIFT,
               CLKGEN_DIV_C906_FACTOR_WIDTH, CLKGEN_C906_1_1_INITIAL_FACTOR)),
};

inline constexpr std::array<PeripheralResources, 18u> PERIPHERALS{
    PeripheralResources{PeripheralId::Sdma, {ClockId::SdmaAxi, ClockId::None}, 1u, ResetId::Sdma, ClockId::SdmaAxi},
    PeripheralResources{PeripheralId::Spi0, {ClockId::Spi, ClockId::ApbSpi0}, 2u, ResetId::Spi0, ClockId::Spi},
    PeripheralResources{PeripheralId::Spi1, {ClockId::Spi, ClockId::ApbSpi1}, 2u, ResetId::Spi1, ClockId::Spi},
    PeripheralResources{PeripheralId::Spi2, {ClockId::Spi, ClockId::ApbSpi2}, 2u, ResetId::Spi2, ClockId::Spi},
    PeripheralResources{PeripheralId::Spi3, {ClockId::Spi, ClockId::ApbSpi3}, 2u, ResetId::Spi3, ClockId::Spi},
    PeripheralResources{PeripheralId::I2c0, {ClockId::I2c, ClockId::ApbI2c0}, 2u, ResetId::I2c0, ClockId::I2c},
    PeripheralResources{PeripheralId::I2c1, {ClockId::I2c, ClockId::ApbI2c1}, 2u, ResetId::I2c1, ClockId::I2c},
    PeripheralResources{PeripheralId::I2c2, {ClockId::I2c, ClockId::ApbI2c2}, 2u, ResetId::I2c2, ClockId::I2c},
    PeripheralResources{PeripheralId::I2c3, {ClockId::I2c, ClockId::ApbI2c3}, 2u, ResetId::I2c3, ClockId::I2c},
    PeripheralResources{PeripheralId::I2c4, {ClockId::I2c, ClockId::ApbI2c4}, 2u, ResetId::I2c4, ClockId::I2c},
    PeripheralResources{PeripheralId::Pwm0, {ClockId::Pwm, ClockId::None}, 1u, ResetId::Pwm0, ClockId::Pwm},
    PeripheralResources{PeripheralId::Pwm1, {ClockId::Pwm, ClockId::None}, 1u, ResetId::Pwm1, ClockId::Pwm},
    PeripheralResources{PeripheralId::Pwm2, {ClockId::Pwm, ClockId::None}, 1u, ResetId::Pwm2, ClockId::Pwm},
    PeripheralResources{PeripheralId::Pwm3, {ClockId::Pwm, ClockId::None}, 1u, ResetId::Pwm3, ClockId::Pwm},
    PeripheralResources{PeripheralId::SarAdc, {ClockId::SarAdc, ClockId::None}, 1u, ResetId::SarAdc, ClockId::SarAdc},
    PeripheralResources{PeripheralId::Watchdog, {ClockId::ApbWatchdog, ClockId::None}, 1u, ResetId::Watchdog, ClockId::ApbWatchdog},
    PeripheralResources{PeripheralId::Sdio0, {ClockId::Axi4Sd0, ClockId::Sd0}, 2u, ResetId::Sd0, ClockId::Sd0},
    PeripheralResources{PeripheralId::Sdio1, {ClockId::Axi4Sd1, ClockId::Sd1}, 2u, ResetId::Sd1, ClockId::Sd1},
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

enum class RatePolicy : uint8_t
{
  Nearest,
  AtMost,
  Exact,
};

enum class PlanStatus : uint8_t
{
  Ok,
  InvalidClock,
  ReadOnlyClock,
  InvalidParent,
  InvalidRate,
  UnattainableRate,
  UnknownParentRate,
  Busy,
};

struct ClockRatePlan
{
  ClockId clock = ClockId::None;       // Requested clock, possibly a gate leaf.
  ClockId rate_clock = ClockId::None;  // Mux/divider that actually sets the rate.
  ClockId parent = ClockId::None;
  uint32_t parent_rate_hz = 0u;
  uint32_t target_rate_hz = 0u;
  uint32_t actual_rate_hz = 0u;
  uint8_t parent_index = 0u;
  uint8_t divider = 0u;
  RatePolicy policy = RatePolicy::Nearest;
  PlanStatus status = PlanStatus::InvalidClock;

  [[nodiscard]] constexpr bool IsValid() const noexcept { return status == PlanStatus::Ok; }
  [[nodiscard]] constexpr explicit operator bool() const noexcept { return IsValid(); }
};

[[nodiscard]] constexpr std::size_t NodeIndex(ClockId clock) noexcept
{
  for (std::size_t index = 0u; index < NODES.size(); ++index)
  {
    if (NODES[index].id == clock) return index;
  }
  return NODES.size();
}

[[nodiscard]] constexpr uint8_t ParentIndex(const ClockNode& node, ClockId parent) noexcept
{
  for (uint8_t index = 0u; index < node.parents.count; ++index)
  {
    if (node.parents.ids[index] == parent) return index;
  }
  return 0xFFu;
}

[[nodiscard]] constexpr uint32_t DividerMaximum(const Divider& divider) noexcept
{
  return divider.field.width == 0u || divider.field.width > 32u
             ? 0u : static_cast<uint32_t>((uint64_t{1u} << divider.field.width) - 1u);
}

[[nodiscard]] constexpr uint64_t RateError(uint32_t actual, uint32_t target) noexcept
{
  return actual >= target ? static_cast<uint64_t>(actual - target)
                          : static_cast<uint64_t>(target - actual);
}

// Resolve transparent gates without changing their parent clocks implicitly.
[[nodiscard]] constexpr const ClockNode* RateControlNode(ClockId clock) noexcept
{
  const ClockNode* node = Find(clock);
  for (uint8_t depth = 0u; node != nullptr && depth < MAX_CLOCK_DEPTH; ++depth)
  {
    if (node->kind != NodeKind::Gate) return node;
    if (node->parents.count != 1u) return nullptr;
    node = Find(node->parents.ids[0]);
  }
  return nullptr;
}

[[nodiscard]] constexpr bool IsParentValid(ClockId clock, ClockId parent) noexcept
{
  const ClockNode* node = RateControlNode(clock);
  if (node == nullptr || node->kind != NodeKind::MuxDividerGate || node->path_select.Exists())
    return false;
  const uint8_t index = ParentIndex(*node, parent);
  if (index == 0xFFu) return false;
  if (node->bypass.Exists() && index == 0u) return true;
  const uint8_t selection = index - (node->bypass.Exists() ? 1u : 0u);
  return node->source_select.Exists()
             ? selection < (uint64_t{1u} << node->source_select.width)
             : selection == 0u;
}

[[nodiscard]] constexpr bool IsDividerValid(ClockId clock, uint32_t divider) noexcept
{
  const ClockNode* node = RateControlNode(clock);
  return node != nullptr && node->divider0.Exists() && divider != 0u &&
         divider <= DividerMaximum(node->divider0);
}

// These buses also carry Linux or the controller's own register accesses.
// Startup may apply the agreed board profile; runtime changes must be no-ops.
[[nodiscard]] constexpr bool IsSystemBusClock(ClockId clock) noexcept
{
  return clock == ClockId::CpuAxi0 || clock == ClockId::Axi4 || clock == ClockId::Axi6;
}

/** Value-parameter planner: usable with runtime input, or optionally in a constant
 * expression. No hardware access, allocation, exceptions or template arguments.
 * Rates outside the hardware range are rejected instead of silently clamped.
 * Bypass routes carry the parent directly and never divide it. */
[[nodiscard]] constexpr ClockRatePlan MakeRatePlan(
    ClockId clock, ClockId parent, uint32_t parent_rate_hz, uint32_t target_rate_hz,
    RatePolicy policy = RatePolicy::Nearest) noexcept
{
  ClockRatePlan plan{};
  plan.clock = clock;
  plan.parent = parent;
  plan.parent_rate_hz = parent_rate_hz;
  plan.target_rate_hz = target_rate_hz;
  plan.policy = policy;
  const ClockNode* node = RateControlNode(clock);
  if (node == nullptr) return plan;
  plan.rate_clock = node->id;
  if (node->ownership != Ownership::C906LManaged || node->kind != NodeKind::MuxDividerGate ||
      node->path_select.Exists() || !node->divider0.Exists())
  {
    plan.status = PlanStatus::ReadOnlyClock;
    return plan;
  }
  if (!IsParentValid(clock, parent))
  {
    plan.status = PlanStatus::InvalidParent;
    return plan;
  }
  plan.parent_index = ParentIndex(*node, parent);
  if (parent_rate_hz == 0u || target_rate_hz == 0u ||
      (parent == ClockId::Oscillator && parent_rate_hz != OSCILLATOR_HZ) ||
      static_cast<uint8_t>(policy) > static_cast<uint8_t>(RatePolicy::Exact))
  {
    plan.status = PlanStatus::InvalidRate;
    return plan;
  }
  const bool bypass = node->bypass.Exists() && plan.parent_index == 0u;
  const uint32_t maximum = bypass ? 1u : DividerMaximum(node->divider0);
  plan.status = PlanStatus::UnattainableRate;
  if (maximum == 0u || maximum > UINT8_MAX || target_rate_hz > parent_rate_hz ||
      static_cast<uint64_t>(target_rate_hz) * maximum < parent_rate_hz) return plan;

  uint64_t best_error = UINT64_MAX;
  for (uint32_t divider = 1u; divider <= maximum; ++divider)
  {
    const uint64_t desired = static_cast<uint64_t>(target_rate_hz) * divider;
    if (policy == RatePolicy::AtMost && parent_rate_hz > desired) continue;
    if (policy == RatePolicy::Exact && parent_rate_hz != desired) continue;
    const uint64_t error = desired >= parent_rate_hz ? desired - parent_rate_hz
                                                     : parent_rate_hz - desired;
    if (plan.divider == 0u || error * plan.divider < best_error * divider ||
        (error * plan.divider == best_error * divider && divider > plan.divider))
    {
      best_error = error;
      plan.divider = static_cast<uint8_t>(divider);
    }
  }
  if (plan.divider != 0u)
  {
    plan.actual_rate_hz = parent_rate_hz / plan.divider;
    plan.status = PlanStatus::Ok;
  }
  return plan;
}

[[nodiscard]] constexpr bool IsClockPlanWellFormed(const ClockRatePlan& plan) noexcept
{
  if (!plan.IsValid()) return false;
  const ClockRatePlan expected = MakeRatePlan(plan.clock, plan.parent, plan.parent_rate_hz,
                                             plan.target_rate_hz, plan.policy);
  return expected.IsValid() && plan.rate_clock == expected.rate_clock &&
         plan.parent_index == expected.parent_index && plan.divider == expected.divider &&
         plan.actual_rate_hz == expected.actual_rate_hz;
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
    MakeRatePlan(ClockId::Axi4, ClockId::Fpll, C906L_FPLL_HZ, 300000000u);
inline constexpr ClockRatePlan C906L_AXI6_PLAN =
    MakeRatePlan(ClockId::Axi6, ClockId::Fpll, C906L_FPLL_HZ, 100000000u);
inline constexpr ClockRatePlan C906L_1M_PLAN =
    MakeRatePlan(ClockId::Clock1M, ClockId::Oscillator, OSCILLATOR_HZ, 1000000u);
inline constexpr ClockRatePlan C906L_SPI_PLAN =
    MakeRatePlan(ClockId::Spi, ClockId::Fpll, C906L_FPLL_HZ, 187500000u);
inline constexpr ClockRatePlan C906L_I2C_PLAN =
    MakeRatePlan(ClockId::I2c, ClockId::Axi6, C906L_AXI6_PLAN.actual_rate_hz,
                 100000000u);

// The whole C906L peripheral domain is visible here. The RCC applies only a
// setting that is needed by the peripheral being prepared, recursively
// applying planned C906L parents first.
inline constexpr C906LClockPlan DEFAULT_C906L_CLOCK_PLAN{
    C906L_FPLL_HZ,
    {C906L_AXI4_PLAN, C906L_AXI6_PLAN, C906L_1M_PLAN, C906L_SPI_PLAN, C906L_I2C_PLAN},
};

static_assert(C906L_SPI_PLAN.divider == 8u && C906L_SPI_PLAN.actual_rate_hz == 187500000u,
              "C906L SPI profile must produce the documented 187.5 MHz SSI clock");
static_assert(C906L_I2C_PLAN.divider == 1u && C906L_I2C_PLAN.actual_rate_hz == 100000000u,
              "C906L I2C profile must retain its documented 100 MHz input clock");

[[nodiscard]] constexpr bool IsClockRegisterOffset(uint16_t offset) noexcept
{
  switch (offset)
  {
#define SG2002_CLOCK_OFFSET(member) case offsetof(CLKGEN_Type, member): return true;
    SG2002_CLKGEN_REGISTERS(SG2002_CLOCK_OFFSET)
#undef SG2002_CLOCK_OFFSET
    default: return false;
  }
}

[[nodiscard]] constexpr bool IsFieldWellFormed(const RegisterField& field) noexcept
{
  if (!field.Exists())
  {
    return field.shift == 0u && field.width == 0u;
  }
  return IsClockRegisterOffset(field.offset) &&
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
  for (std::size_t index = 0u; index < parents.ids.size(); ++index)
  {
    if ((index < parents.count) != (parents.ids[index] != ClockId::None))
    {
      return false;
    }
    for (std::size_t previous = 0u; previous < index; ++previous)
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
      !IsFieldWellFormed(node.source_select) || !IsFieldWellFormed(node.divider0.field) ||
      !IsFieldWellFormed(node.divider1.field) ||
      (node.gate.Exists() && node.gate.width != 1u) ||
      (node.bypass.Exists() && node.bypass.width != 1u) ||
      (node.path_select.Exists() && node.path_select.width != 1u) ||
      (node.source_select.Exists() && node.source_select.width > CLKGEN_DIV_SRC_WIDTH) ||
      (node.divider0.field.width > 8u || node.divider1.field.width > 8u) ||
      (node.bypass.Exists() && node.parents.ids[0] != ClockId::Oscillator) ||
      (node.divider0.reset_value > DividerMaximum(node.divider0)) ||
      (node.divider1.reset_value > DividerMaximum(node.divider1)) ||
      (node.divider0.reset_value != 0u && node.divider0.field.shift <= 3u &&
       static_cast<uint16_t>(node.divider0.field.shift) + node.divider0.field.width >
           3u) ||
      (node.divider1.reset_value != 0u && node.divider1.field.shift <= 3u &&
       static_cast<uint16_t>(node.divider1.field.shift) + node.divider1.field.width > 3u))
  {
    return false;
  }

  switch (node.kind)
  {
    case NodeKind::Fixed:
      return node.parents.count == 0u && !node.gate.Exists() && !node.bypass.Exists() &&
             !node.path_select.Exists() && !node.source_select.Exists() &&
             node.pll_csr == NO_REGISTER && !node.divider0.Exists() &&
             !node.divider1.Exists() && node.ownership == Ownership::RuntimeOnly;
    case NodeKind::G6Pll:
      return node.parents.count == 1u && node.parents.ids[0] == ClockId::Oscillator &&
             !node.gate.Exists() && !node.bypass.Exists() && !node.path_select.Exists() &&
             !node.source_select.Exists() && node.pll_csr != NO_REGISTER &&
             (node.pll_csr == PLL_G6_MPLL_OFFSET || node.pll_csr == PLL_G6_TPLL_OFFSET ||
              node.pll_csr == PLL_G6_FPLL_OFFSET) &&
             (node.pll_csr % sizeof(uint32_t)) == 0u && !node.divider0.Exists() &&
             !node.divider1.Exists() && node.ownership == Ownership::RuntimeOnly;
    case NodeKind::G2Pll:
      return node.parents.count == 1u && !node.gate.Exists() && !node.bypass.Exists() &&
             !node.path_select.Exists() && !node.source_select.Exists() &&
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
              (!node.divider1.Exists() && node.parents.count <= single_path_capacity)) &&
             node.ownership == (node.path_select.Exists() ? Ownership::RuntimeOnly
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
        ReachesClock(node->parents.ids[index], target, static_cast<uint8_t>(depth + 1u)))
    {
      return true;
    }
  }
  return false;
}

[[nodiscard]] constexpr bool FitsRuntimeDepth(ClockId current, uint8_t depth) noexcept
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
    if (!FitsRuntimeDepth(node->parents.ids[index], static_cast<uint8_t>(depth + 1u)))
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
        resource.clock_count == 0u || resource.clock_count > resource.clocks.size() ||
        Find(resource.rate_clock) == nullptr || resource.reset == ResetId::None ||
        static_cast<uint16_t>(resource.reset) >= RESET_LINE_COUNT)
    {
      return false;
    }
    bool has_rate_clock = false;
    for (std::size_t clock_index = 0u; clock_index < resource.clocks.size();
         ++clock_index)
    {
      if ((clock_index < resource.clock_count) !=
              (resource.clocks[clock_index] != ClockId::None) ||
          (clock_index < resource.clock_count &&
           Find(resource.clocks[clock_index]) == nullptr))
      {
        return false;
      }
      has_rate_clock = has_rate_clock || resource.clocks[clock_index] == resource.rate_clock;
      for (std::size_t previous = 0u; previous < clock_index; ++previous)
      {
        if (resource.clocks[clock_index] != ClockId::None &&
            resource.clocks[clock_index] == resource.clocks[previous])
        {
          return false;
        }
      }
    }
    if (!has_rate_clock) return false;
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
    if (!IsClockPlanWellFormed(plan) || plan.rate_clock != plan.clock || node == nullptr)
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
