#include <utility>

#include "libhal-arm-mcu/stm32f1/constants.hpp"
#include "libhal-arm-mcu/stm32f1/timer.hpp"
#include "stm32f1/pin.hpp"
#include "stm32f1/power.hpp"
#include <libhal-arm-mcu/stm32f1/quadrature_encoder.hpp>
#include <libhal-util/bit.hpp>

namespace hal::stm32f1 {
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

[[nodiscard]] timer_reg_t* get_timer_reg(void* p_reg)
{
  return reinterpret_cast<timer_reg_t*>(p_reg);
}
// [[nodiscard]] peripheral get_peripheral(void* p_reg)
// {
// }
void setup_channel(int channel, timer_reg_t* p_reg)
{

  constexpr auto odd_channel_input_mode = bit_mask::from<0, 1>();
  constexpr auto odd_channel_filter_select = bit_mask::from<4, 7>();

  constexpr auto even_channel_input_mode = bit_mask::from<8, 9>();
  constexpr auto even_channel_filter_select = bit_mask::from<12, 15>();

  constexpr auto input_select = 0b01U;
  constexpr auto input_capture_filter = 0b0000U;

  auto const polarity_start_pos = ((channel - 1) * 4) + 1;
  auto const input_capture_enable = ((channel - 1) * 4);

  auto const input_start_pos = bit_mask::from(input_capture_enable);
  auto const polarity_inverted = bit_mask::from(polarity_start_pos);

  
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
    case 3:
      bit_modify(p_reg->capture_compare_mode_register_2)
        .insert<odd_channel_input_mode>(input_select)
        .insert<odd_channel_filter_select>(input_capture_filter);
      break;
    case 4:
      bit_modify(p_reg->capture_compare_mode_register_2)
        .insert<even_channel_input_mode>(input_select)
        .insert<even_channel_filter_select>(input_capture_filter);
      break;
    default:
      std::unreachable();
  }
  bit_modify(p_reg->cc_enable_register).clear(polarity_inverted);
  bit_modify(p_reg->cc_enable_register).set(input_start_pos);
}
int get_channel_from_pin(hal::stm32f1::pins p_pin,
                         hal::stm32f1::peripheral p_select)
{
  // a generic pwm class requires a pin and a channel in order to use the right
  // registers, therefore we pass in the channel as an argument in the generic
  // pwm class's constructor
  u8 channel = 0;
  switch (p_pin) {
    case pins::pa0:
      configure_pin({ .port = 'A', .pin = 0 }, input_float);
      channel = 1;
      break;
    case pins::pa1:
      configure_pin({ .port = 'A', .pin = 1 }, input_float);
      channel = 2;
      break;
    case pins::pa2:
      configure_pin({ .port = 'A', .pin = 2 }, input_float);
      channel = p_select == peripheral::timer9 ? 1 : 3;
      break;
    case pins::pa3:
      configure_pin({ .port = 'A', .pin = 3 }, input_float);
      channel = p_select == peripheral::timer9 ? 2 : 4;
      break;
    case pins::pa6:
      configure_pin({ .port = 'A', .pin = 6 }, input_float);
      channel = 1;
      break;
    case pins::pa7:
      configure_pin({ .port = 'A', .pin = 7 }, input_float);
      channel = p_select == peripheral::timer14 ? 1 : 2;
      break;
    case pins::pb0:
      configure_pin({ .port = 'B', .pin = 0 }, input_float);
      channel = 3;
      break;
    case pins::pb1:
      configure_pin({ .port = 'B', .pin = 1 }, input_float);
      channel = 4;
      break;
    case pins::pb6:
      configure_pin({ .port = 'B', .pin = 6 }, input_float);
      channel = 1;
      break;
    case pins::pb7:
      configure_pin({ .port = 'B', .pin = 7 }, input_float);
      channel = 2;
      break;
    case pins::pb8:
      configure_pin({ .port = 'B', .pin = 8 }, input_float);
      channel = p_select == peripheral::timer10 ? 1 : 3;
      break;
    case pins::pb9:
      configure_pin({ .port = 'B', .pin = 9 }, input_float);
      channel = p_select == peripheral::timer11 ? 1 : 4;
      break;
    case pins::pa8:
      configure_pin({ .port = 'A', .pin = 8 }, input_float);
      channel = 1;
      break;
    case pins::pa9:
      configure_pin({ .port = 'A', .pin = 9 }, input_float);
      channel = 2;
      break;
    case pins::pa10:
      configure_pin({ .port = 'A', .pin = 10 }, input_float);
      channel = 3;
      break;
    case pins::pa11:
      configure_pin({ .port = 'A', .pin = 11 }, input_float);
      channel = 4;
      break;
    case pins::pc6:
      configure_pin({ .port = 'C', .pin = 6 }, input_float);
      channel = 1;
      break;
    case pins::pc7:
      configure_pin({ .port = 'C', .pin = 7 }, input_float);
      channel = 2;
      break;
    case pins::pc8:
      configure_pin({ .port = 'C', .pin = 8 }, input_float);
      channel = 3;
      break;
    case pins::pc9:
      configure_pin({ .port = 'C', .pin = 9 }, input_float);
      channel = 4;
      break;
    case pins::pb14:
      configure_pin({ .port = 'B', .pin = 14 }, input_float);
      channel = 1;
      break;
    case pins::pb15:
      configure_pin({ .port = 'B', .pin = 15 }, input_float);
      channel = 2;
      break;
    default:
      std::unreachable();
  }
  return channel;
}
quadrature_encoder::quadrature_encoder(hal::stm32f1::pins p_pin1,
                                       hal::stm32f1::pins p_pin2,
                                       hal::stm32f1::peripheral p_select,
                                       void* p_reg)
{
  m_reg = p_reg;
  // setup
  power_on(p_select);
  timer_reg_t* timer_register = get_timer_reg(m_reg);
  // peripheral peripheral = get_peripheral_reg(m_reg);
  int channel_a = get_channel_from_pin(p_pin1, p_select);
  int channel_b = get_channel_from_pin(p_pin2, p_select);
  constexpr auto set_encoder_mode = bit_mask::from<0, 2>();
  // encoder counts up/down on both TI1FP1 and TI2FP1 level
  constexpr auto encoder_mode_3 = 0b011U;
  bit_modify(timer_register->peripheral_control_register)
    .insert<set_encoder_mode>(encoder_mode_3);
  setup_channel(channel_a, timer_register);
  setup_channel(channel_b, timer_register);

  timer_register->auto_reload_register = 0xFFFF;  // Set max counter value
  timer_register->counter_register = 0x8000;      // Start at middle value

  constexpr auto counter_enable = bit_mask::from<0>();
  // timer_register->counter_register = 1000;  // test
  bit_modify(timer_register->control_register).set(counter_enable);
  // maybe set counter to something in the middle.
}
quadrature_encoder::read_t quadrature_encoder::driver_read()
{
  read_t reading;
  timer_reg_t* timer_register = get_timer_reg(m_reg);
  [[maybe_unused]] u32 direction = timer_register->control_register;

  reading.angle = static_cast<float>(timer_register->counter_register);
  return reading;
}
}  // namespace hal::stm32f1
