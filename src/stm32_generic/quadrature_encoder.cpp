#include <libhal/initializers.hpp>
#include <utility>

#include <libhal-arm-mcu/stm32_generic/quadrature_encoder.hpp>
#include <libhal-util/bit.hpp>

namespace hal::stm32_generic {
struct timer_reg_t
{
  /// Offset: 0x00 Control Register (R/W)
  u32 volatile control_register;  // sets up timers
  /// Offset: 0x04 Control Register 2 (R/W)
  u32 volatile control_register_2;
  /// Offset: 0x08 Peripheral Mode Control Register (R/W)
  u32 volatile peripheral_control_register;
  /// Offset: 0x0C DMA/Interrupt enable register (R/W)
  u32 volatile interuupt_enable_register;
  /// Offset: 0x10 Status Register register (R/W)
  u32 volatile status_register;
  /// Offset: 0x14 Event Generator Register register (R/W)
  u32 volatile event_generator_register;
  /// Offset: 0x18 Capture/Compare mode register (R/W)
  u32 volatile capture_compare_mode_register;  // set up modes for
                                               // channel
  /// Offset: 0x1C Capture/Compare mode register (R/W)
  u32 volatile capture_compare_mode_register_2;
  /// Offset: 0x20 Capture/Compare Enable register (R/W)
  u32 volatile cc_enable_register;
  /// Offset: 0x24 Counter (R/W)
  u32 volatile counter_register;
  /// Offset: 0x28 Prescalar (R/W)
  u32 volatile prescale_register;
  /// Offset: 0x2C Auto Reload Register (R/W)
  u32 volatile auto_reload_register;  // affects frequency
  /// Offset: 0x30 Repetition Counter Register (R/W)
  u32 volatile repetition_counter_register;
  /// Offset: 0x34 Capture Compare Register (R/W)
  u32 volatile capture_compare_register;  // affects duty cycles
  /// Offset: 0x38 Capture Compare Register (R/W)
  u32 volatile capture_compare_register_2;
  // Offset: 0x3C Capture Compare Register (R/W)
  u32 volatile capture_compare_register_3;
  // Offset: 0x40 Capture Compare Register (R/W)
  u32 volatile capture_compare_register_4;
  /// Offset: 0x44 Break and dead-time register
  u32 volatile break_and_deadtime_register;
  /// Offset: 0x48 DMA control register
  u32 volatile dma_control_register;
  /// Offset: 0x4C DMA address for full transfer
  u32 volatile dma_address_register;
};

[[nodiscard]] inline timer_reg_t* get_timer_reg(void* p_reg)
{
  return reinterpret_cast<timer_reg_t*>(p_reg);
}
void setup_channel(int channel, timer_reg_t* p_reg)
{

  constexpr auto odd_channel_input_mode = bit_mask::from<0, 1>();
  constexpr auto odd_channel_filter_select = bit_mask::from<4, 7>();

  constexpr auto even_channel_input_mode = bit_mask::from<8, 9>();
  constexpr auto even_channel_filter_select = bit_mask::from<12, 15>();

  constexpr auto input_select = 0b01U;
  constexpr auto input_capture_filter = 0b0000U;

  // Select the TI1 and TI2 polarity by programming the CC1P and CC2P bits in
  // the TIMx_CCER
  // register. When needed, the user can program the input filter as well.
  // find out which pin is what channel
  switch (channel) {
    case 1:
      bit_modify(p_reg->capture_compare_mode_register)
        .insert<odd_channel_input_mode>(input_select)
        .insert<odd_channel_filter_select>(input_capture_filter);
      break;
    case 2:
      bit_modify(p_reg->capture_compare_mode_register)
        .insert<even_channel_input_mode>(input_select)
        .insert<even_channel_filter_select>(input_capture_filter);
      break;
    default:
      std::unreachable();
  }
}

void setup_enable_register(int channel, timer_reg_t* p_reg)
{
  // this function must occur after CCMR registers are written too, otherwise,
  // they will not get written.
  auto const polarity_start_pos = ((channel - 1) * 4) + 1;
  auto const input_capture_enable = ((channel - 1) * 4);

  auto const input_start_pos = bit_mask::from(input_capture_enable);
  auto const polarity_inverted = bit_mask::from(polarity_start_pos);

  bit_modify(p_reg->cc_enable_register).clear(polarity_inverted);
  bit_modify(p_reg->cc_enable_register).set(input_start_pos);
}

quadrature_encoder::quadrature_encoder(hal::unsafe,
                                       encoder_channels channels,
                                       void* p_reg)
{
  initialize(hal::unsafe{}, channels, p_reg);
}
quadrature_encoder::quadrature_encoder(hal::unsafe)
{
}
void quadrature_encoder::initialize(unsafe,
                                    encoder_channels channels,
                                    void* p_reg)
{
  m_reg = p_reg;

  timer_reg_t* timer_register = get_timer_reg(m_reg);
  constexpr auto set_encoder_mode = bit_mask::from<0, 2>();
  // encoder counts up/down on both TI1FP1 and TI2FP1 level
  constexpr auto encoder_mode_3 = 0b011U;

  setup_channel(channels.channel_a, timer_register);
  setup_channel(channels.channel_b, timer_register);
  setup_enable_register(channels.channel_a, timer_register);
  setup_enable_register(channels.channel_b, timer_register);
  timer_register->auto_reload_register = 0xFFFF;  // Set max counter value
  timer_register->counter_register = 0x8000;      // Start at middle value
  bit_modify(timer_register->peripheral_control_register)
    .insert<set_encoder_mode>(encoder_mode_3);
  constexpr auto counter_enable = bit_mask::from<0>();
  bit_modify(timer_register->control_register).set(counter_enable);
}
quadrature_encoder::read_t quadrature_encoder::driver_read()
{
  read_t reading;
  timer_reg_t* timer_register = get_timer_reg(m_reg);

  reading.angle = static_cast<float>(timer_register->counter_register);
  return reading;
}
}  // namespace hal::stm32_generic
