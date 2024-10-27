#include "pin.hpp"
#include "power.hpp"
#include "pwm_reg.hpp"
#include <cmath>
#include <libhal-arm-mcu/stm32f1/clock.hpp>
#include <libhal-arm-mcu/stm32f1/constants.hpp>
#include <libhal-arm-mcu/stm32f1/pin.hpp>
#include <libhal-arm-mcu/stm32f1/pwm.hpp>
#include <libhal-util/bit.hpp>

namespace hal::stm32f1 {
namespace {
[[nodiscard]] pwm_reg_t* get_pwm_reg(peripheral p_id)
{
  if (p_id == peripheral::timer1) {
    return pwm_timer1;
  }
  // other timers allow pwm so i need to double check if they are allowed
  return pwm_timer8;
}
[[nodiscard]] peripheral get_peripheral_id(pwm_pins p_pin)
{
  if (p_pin == pwm_pins::pa8 || p_pin == pwm_pins::pa9 ||
      p_pin == pwm_pins::pa10 || p_pin == pwm_pins::pa11) {

    return peripheral::timer1;
    // if they chose remap pins, we need to remap the pins (but Not sure how to
    // do that)
  } else {
    hal::safe_throw(hal::operation_not_supported(
      nullptr));  // this could also be not permitted but not sure.
  }
  return peripheral::timer8;  // just to remove a compiler complain
}

[[nodiscard]] float get_duty_cycle(pwm_pins p_pin,
                                   uint32_t volatile* compare_register)
{
  // ccr / arr
  auto peripheral_id = get_peripheral_id(p_pin);
  pwm_reg_t* p_reg = get_pwm_reg(peripheral_id);

  static constexpr auto first_nonreserved_half = bit_mask::from<0, 15>();

  std::uint16_t compare_value = hal::bit_extract<first_nonreserved_half>(
    *compare_register);  // gets the first 16 bits of CCR register

  std::uint16_t arr_value = hal::bit_extract<first_nonreserved_half>(
    p_reg->auto_reload_register);  // gets the first 16 bits of CCR register

  return (compare_value / static_cast<float>(arr_value));
}
void setup_channel(pwm_reg_t* p_reg, uint8_t p_channel)
{

  // uint32_t start_pos = 4 * (p_channel - 1);
  static constexpr auto main_output_enable = bit_mask::from<15>();
  // bit_modify(p_reg->break_and_deadtime_register)
  //   .insert<main_output_enable>(0b1U);
  static constexpr auto ossr = bit_mask::from<11>();
  switch (
    p_channel) {  // not proud of this code but the from was marked consteexpr
                  // and it didn't let me do from(start_pos, start_pos + 1);
    case 1:
      static constexpr auto ccer_polarity_enable_mask = bit_mask::from(0, 1);
      // static constexpr auto ccer_comp_polarity_enable_mask =
      //   bit_mask::from(2, 3);

      bit_modify(p_reg->cc_enable_register)
        .insert<ccer_polarity_enable_mask>(0b01U);
      // bit_modify(p_reg->cc_enable_register)
      //   .clear(ccer_comp_polarity_enable_mask);

      break;
    case 2:
      static constexpr auto ccer_polarity_enable_mask_2 = bit_mask::from(4, 5);
      // static constexpr auto ccer_comp_polarity_enable_mask_2 =
        // bit_mask::from(6, 7);

      bit_modify(p_reg->cc_enable_register)
        .insert<ccer_polarity_enable_mask_2>(0b01U);
      // bit_modify(p_reg->cc_enable_register)
      //   .clear(ccer_comp_polarity_enable_mask_2);

      break;
    case 3:
      static constexpr auto ccer_polarity_enable_mask_3 = bit_mask::from(8, 9);
      // static constexpr auto ccer_comp_polarity_enable_mask_3 =
        // bit_mask::from(10, 11);

      bit_modify(p_reg->cc_enable_register)
        .insert<ccer_polarity_enable_mask_3>(0b01U);
      // bit_modify(p_reg->cc_enable_register)
      //   .clear(ccer_comp_polarity_enable_mask_3);

      break;
    case 4:
      static constexpr auto ccer_polarity_enable_mask_4 =
        bit_mask::from(12, 13);
      bit_modify(p_reg->cc_enable_register)
        .insert<ccer_polarity_enable_mask_4>(0b01U);
      break;
  }

  // MOE = 1 and OSSR = 0 in the breakdeadtime register

  // bit_modify(p_reg->break_and_deadtime_register)
  //   .insert<main_output_enable>(0b1U);
  bit_modify(p_reg->break_and_deadtime_register)
    .clear(ossr)
    .set(main_output_enable);  // complementary channel stuff

  return;
}
void setup(pwm_pins p_pin)
{
  // control register
  // set clock division to 0
  // set arpe bit to 1 in the cr
  // upcounting active when dir bit in cr1 is low
  // CR1 - set cms bit for edge aligned ness
  static constexpr auto clock_division = bit_mask::from<8, 9>();
  // static constexpr auto auto_reload_preload_enable = bit_mask::from<7>();
  static constexpr auto edge_aligned_mode = bit_mask::from<5, 6>();  // set to
                                                                     // 00
  static constexpr auto direction =
    bit_mask::from<4>();  // set to 1 for upcounting mdoe
  static constexpr auto output_compare_odd =
    bit_mask::from<4, 6>();  // set to 111 for channel1/3 is inactive as long as
                             // timx_cnt < tmx_ccr1
  static constexpr auto output_compare_even =
    bit_mask::from<12, 14>();  // set to 111 for channel2/4 is inactive as long
                               // as timx_cnt < tmx_ccr1
  static constexpr auto counter_enable = bit_mask::from<0>();  // set this to 1
  static constexpr auto odd_channel_preload_enable = bit_mask::from<3>();
  static constexpr auto even_channel_preload_enable = bit_mask::from<11>();
  // static constexpr auto update_generation_enable = bit_mask::from<0>();

  peripheral peripheral_id = get_peripheral_id(p_pin);
  pwm_reg_t* reg = get_pwm_reg(peripheral_id);

  bit_modify(reg->control_register).insert<clock_division>(0b00U);  //
  bit_modify(reg->control_register).insert<edge_aligned_mode>(0b00U);
  bit_modify(reg->control_register).clear(direction);

  // OCxM bit set : PWM mode 2 - In upcounting, channel 1 is inactive as long as
  // TIMx_CNT<TIMx_CCR1 else active.
  if (p_pin == pwm_pins::pa8) {  // channel 1 // index in this case corresponds
                                 // to channel
                                 //  set oc1m bit to
    // Preload enable must be done for corresponding OCxPE
    bit_modify(reg->capture_compare_mode_register)
      .insert<output_compare_odd>(0b110U);
    bit_modify(reg->capture_compare_mode_register)
      .set(odd_channel_preload_enable);
    setup_channel(reg, 1);
    // leaving polarity active low for each channel
  } else if (p_pin == pwm_pins::pa9) {  // channel 2
    bit_modify(reg->capture_compare_mode_register)
      .insert<output_compare_even>(0b110U);
    bit_modify(reg->capture_compare_mode_register)
      .set(even_channel_preload_enable);
    setup_channel(reg, 2);

  } else if (p_pin == pwm_pins::pa10) {  // channel 3
    bit_modify(reg->capture_compare_mode_register_2)
      .insert<output_compare_odd>(0b110U);
    bit_modify(reg->capture_compare_mode_register_2)
      .set(odd_channel_preload_enable);
    setup_channel(reg, 3);

  } else if (p_pin == pwm_pins::pa11) {  // channel 4
    bit_modify(reg->capture_compare_mode_register_2)
      .insert<output_compare_even>(0b110U);
    bit_modify(reg->capture_compare_mode_register_2)
      .set(even_channel_preload_enable);
    setup_channel(reg, 4);
  }

  // Sets pre-scalar to 0 because (CK_CNT) is equal to fCK_PSC / (PSC[15:0] + 1)
  // This makes is to that initially the frequency of PWM is: clock_freq / ARR
  // value
  // bit_modify(reg->control_register).set(auto_reload_preload_enable);
  bit_modify(reg->control_register).set(counter_enable);

  reg->prescale_register = 0x0U;

  // Set counter value to 0 because it upcounts.
  reg->counter_register = 0x0U;

  //
  reg->auto_reload_register = 0xFFFF;
}

}  // namespace

pwm::pwm(pwm_pins p_pin)
  : m_pin{ p_pin }
{
  // config pin to alternate function push - pull
  auto p_id = get_peripheral_id(p_pin);
  auto p_reg = get_pwm_reg(p_id);
  power_on(p_id);
  // power_on(peripheral::afio);
  switch (p_pin) {
    case pwm_pins::pa10:
      configure_pin({ .port = 'A', .pin = 10 }, push_pull_alternative_output);
      m_compare_register_addr = &p_reg->capture_compare_register_3;
      m_channel = 3;
      break;
    case pwm_pins::pa11:
      configure_pin({ .port = 'A', .pin = 11 }, push_pull_alternative_output);
      m_compare_register_addr = &p_reg->capture_compare_register_4;
      m_channel = 4;
      break;
    case pwm_pins::pa8:
      configure_pin({ .port = 'A', .pin = 8 }, push_pull_alternative_output);
      m_compare_register_addr = &p_reg->capture_compare_register;
      m_channel = 1;
      break;
    case pwm_pins::pa9:
      configure_pin({ .port = 'A', .pin = 9 }, push_pull_alternative_output);
      m_compare_register_addr = &p_reg->capture_compare_register_2;
      m_channel = 2;
      break;
    default:
      safe_throw(hal::operation_not_supported(this));
  }
  setup(p_pin);
}
void pwm::driver_frequency(hertz p_frequency)
{
  auto peripheral_id = get_peripheral_id(m_pin);
  pwm_reg_t* reg = get_pwm_reg(peripheral_id);

  auto const current_clock = hal::stm32f1::frequency(peripheral_id);

  float previous_duty_cycle = get_duty_cycle(m_pin, m_compare_register_addr);

  if (p_frequency >= current_clock) {
    safe_throw(hal::operation_not_supported(this));
  }

  float possible_prescaler_value =
    current_clock / (p_frequency * std::pow(2, 16));

  std::uint16_t prescale = 0;
  std::uint16_t autoreload = 0xFFFF;

  if (possible_prescaler_value > 1) {
    prescale = std::floor(possible_prescaler_value);
    // set ARR to 2^16 and set prescale value to prescale
  } else {
    // we have to reduce the ARR value.
    autoreload = current_clock / p_frequency;
  }
  reg->auto_reload_register = autoreload;
  reg->prescale_register = prescale;

  // Update duty cycle according to new frequency
  driver_duty_cycle(previous_duty_cycle);
}
void pwm::driver_duty_cycle(float p_duty_cycle)
{
  auto peripheral_id = get_peripheral_id(m_pin);
  pwm_reg_t* reg = get_pwm_reg(peripheral_id);

  std::uint16_t desired_ccr_value = reg->auto_reload_register * p_duty_cycle;

  *m_compare_register_addr = desired_ccr_value;
  return;
}
}  // namespace hal::stm32f1
