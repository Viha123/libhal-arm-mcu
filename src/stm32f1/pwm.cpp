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
    //if they chose remap pins, we need to remap the pins (but Not sure how to do that)
  } else {
    hal::safe_throw(hal::operation_not_supported(nullptr));  // this could also be not permitted but not sure.
  }
  return peripheral::timer8; //just to remove a compiler complain
}
[[nodiscard]] uint32_t volatile& get_capture_compare_register(pwm_pins p_pin)
{
  auto p_id = get_peripheral_id(p_pin);
  auto p_reg = get_pwm_reg(p_id);
  if (p_pin == pwm_pins::pa8) {
    return p_reg->capture_compare_register;
  } else if (p_pin == pwm_pins::pa9) {
    return p_reg->capture_compare_register_2;
  } else if (p_pin == pwm_pins::pa10) {
    return p_reg->capture_compare_register_3;
  } else if (p_pin == pwm_pins::pa11) {
    return p_reg->capture_compare_register_4;
  } else {
    hal::safe_throw(hal::operation_not_supported(nullptr));  // this could also be not permitted but not sure.
  }
  return p_reg->capture_compare_mode_register; //shouldn't ever reach here (I think)
}
[[nodiscard]] float get_duty_cycle(pwm_pins p_pin)
{
  // ccr / arr
  auto peripheral_id = get_peripheral_id(p_pin);
  pwm_reg_t* p_reg = get_pwm_reg(peripheral_id);
  std::uint32_t compare_register =
    get_capture_compare_register(p_pin);

  static constexpr auto first_nonreserved_half = bit_mask::from<0, 15>();

  std::uint16_t compare_value = hal::bit_extract<first_nonreserved_half>(
    compare_register);  // gets the first 16 bits of CCR register

  std::uint16_t arr_value = hal::bit_extract<first_nonreserved_half>(
    p_reg->auto_reload_register);  // gets the first 16 bits of CCR register

  return compare_value / arr_value;
}
void setup(pwm_pins p_pin)
{
  // control register
  // set clock division to 0
  // set arpe bit to 1 in the cr
  // upcounting active when dir bit in cr1 is low
  // CR1 - set cms bit for edge aligned ness
  // before starting the counter, the user has to initialize all the registers
  // by setting the UG bit in the TIMx_EGR register.
  static constexpr auto clock_division = bit_mask::from<8, 9>();
  static constexpr auto auto_reload_preload_enable = bit_mask::from<7>();
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
  peripheral peripheral_id = get_peripheral_id(p_pin);
  power_on(peripheral_id);

  pwm_reg_t* reg = get_pwm_reg(peripheral_id);

  bit_modify(reg->control_register).clear(clock_division);  //
  bit_modify(reg->control_register).set(auto_reload_preload_enable);
  bit_modify(reg->control_register).clear(edge_aligned_mode);
  bit_modify(reg->control_register).set(direction);
  bit_modify(reg->control_register).set(counter_enable);

  // OCxM bit set : PWM mode 2 - In upcounting, channel 1 is inactive as long as
  // TIMx_CNT<TIMx_CCR1 else active.

  if (p_pin == pwm_pins::pa8) { //index in this case corresponds to channel
    // set oc1m bit to
    bit_modify(reg->capture_compare_mode_register).set(output_compare_odd);
  } else if (p_pin == pwm_pins::pa9) {
    bit_modify(reg->capture_compare_mode_register_2).set(output_compare_odd);
  } else if (p_pin == pwm_pins::pa10) {
    bit_modify(reg->capture_compare_mode_register).set(output_compare_even);
  } else if (p_pin == pwm_pins::pa11) {
    bit_modify(reg->capture_compare_mode_register_2).set(output_compare_even);
  }

  // Sets pre-scalar to 0 because (CK_CNT) is equal to fCK_PSC / (PSC[15:0] + 1)
  // This makes is to that initially the frequency of PWM is: clock_freq / ARR
  // value
  reg->prescale_register = 0x0U;

  // Set counter value to 0 because it upcounts.
  reg->counter_register = 0x0U;

  //
}
}  // namespace

pwm::pwm(pwm_pins p_pin)
  : m_pin{p_pin}
{
  // config pin to alternate function push - pull
  switch (p_pin) {
    case pwm_pins::pa10:
      configure_pin({ .port = 'A', .pin = 10 }, push_pull_alternative_output);
      break;
    case pwm_pins::pa11:
      configure_pin({ .port = 'A', .pin = 11 }, push_pull_alternative_output);
      break;
    case pwm_pins::pa8:
      configure_pin({ .port = 'A', .pin = 8 }, push_pull_alternative_output);
      break;
    case pwm_pins::pa9:
      configure_pin({ .port = 'A', .pin = 9 }, push_pull_alternative_output);
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
  // mask for the ARR that needs to be set
  static constexpr auto register_mask = bit_mask::from<0, 15>();
  // current clock frequency
  auto const current_clock = hal::stm32f1::frequency(peripheral_id);

  float previous_duty_cycle = get_duty_cycle(m_pin);

  if (p_frequency >= current_clock) {
    safe_throw(hal::operation_not_supported(this));
  }

  float possible_prescaler_value =
    current_clock / (p_frequency * std::pow(2, 16));
  
  std::uint16_t prescale = 1;
  std::uint16_t autoreload = 0xFFFF;

  if (possible_prescaler_value > 1) {
    prescale = std::ceil(possible_prescaler_value);
    // set ARR to 2^16 and set prescale value to prescale
  } else {
    // we have to reduce the ARR value.
    autoreload = current_clock / p_frequency;
  }
  bit_modify(reg->auto_reload_register).insert<register_mask>(autoreload);
  bit_modify(reg->prescale_register).insert<register_mask>(prescale);

  // Update duty cycle according to new frequency
  driver_duty_cycle(previous_duty_cycle);
}
void pwm::driver_duty_cycle(float p_duty_cycle)
{
  auto peripheral_id = get_peripheral_id(m_pin);
  pwm_reg_t* reg = get_pwm_reg(peripheral_id);
  // current ARR value
  static constexpr auto ccr_mask = bit_mask::from<0, 15>();

  std::uint16_t desired_ccr_value = reg->auto_reload_register * p_duty_cycle;

  std::uint32_t compare_register = get_capture_compare_register(m_pin);

  bit_modify(compare_register).insert<ccr_mask>(desired_ccr_value);
}
}  // namespace hal::stm32f1
