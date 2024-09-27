#include "pwm_reg.hpp"
#include <libhal-arm-mcu/stm32f1/clock.hpp>
#include <libhal-arm-mcu/stm32f1/constants.hpp>
#include <libhal-arm-mcu/stm32f1/pwm.hpp>
#include <libhal-util/bit.hpp>
#include "power.hpp"

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

[[nodiscard]] uint32_t volatile& get_capture_compare_register(
  pwm::channel p_channel,
  pwm_reg_t* p_reg)
{
  if (p_channel.index == 1) {
    return p_reg->capture_compare_register;
  } else if (p_channel.index == 2) {
    return p_reg->capture_compare_register_2;
  } else if (p_channel.index == 3) {
    return p_reg->capture_compare_register_3;
  } else {
    return p_reg->capture_compare_register_4;
  }
}
[[nodiscard]] float get_duty_cycle(pwm::channel p_channel, pwm_reg_t* p_reg)
{
  // ccr / arr
  std::uint32_t compare_register =
    get_capture_compare_register(p_channel, p_reg);

  static constexpr auto first_nonreserved_half = bit_mask::from<0, 15>();

  std::uint16_t compare_value = hal::bit_extract<first_nonreserved_half>(
    compare_register);  // gets the first 16 bits of CCR register

  std::uint16_t arr_value = hal::bit_extract<first_nonreserved_half>(
    p_reg->auto_reload_register);  // gets the first 16 bits of CCR register

  return compare_value / arr_value;
}
void setup(pwm::channel& p_channel)
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

  power_on(p_channel.peripheral_id);

  pwm_reg_t* reg = get_pwm_reg(p_channel.peripheral_id);

  bit_modify(reg->control_register).clear(clock_division);  //
  bit_modify(reg->control_register).set(auto_reload_preload_enable);
  bit_modify(reg->control_register).clear(edge_aligned_mode);
  bit_modify(reg->control_register).set(direction);
  bit_modify(reg->control_register).set(counter_enable);

  // OCxM bit set : PWM mode 2 - In upcounting, channel 1 is inactive as long as
  // TIMx_CNT<TIMx_CCR1 else active.
  std::uint8_t index = p_channel.index;

  if (index == 1) {
    // set oc1m bit to
    bit_modify(reg->capture_compare_mode_register).set(output_compare_odd);
  } else if (index == 3) {
    bit_modify(reg->capture_compare_mode_register_2).set(output_compare_odd);
  } else if (index == 2) {
    bit_modify(reg->capture_compare_mode_register).set(output_compare_even);
  } else if (index == 4) {
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

pwm::pwm(std::uint8_t p_peripheral, std::uint8_t p_channel)
  : m_channel{}
{ 
  // need to map peripheral to output pins

  setup(p_channel);
}
void pwm::driver_frequency(hertz p_frequency)
{
  pwm_reg_t* reg = get_pwm_reg(m_channel.peripheral_id);
  // mask for the ARR that needs to be set
  static constexpr auto arr_mask = bit_mask::from<0, 15>();
  // current clock frequency
  auto const current_clock = hal::stm32f1::frequency(m_channel.peripheral_id);

  float previous_duty_cycle = get_duty_cycle(m_channel, reg);

  if (p_frequency >= current_clock) {
    safe_throw(hal::operation_not_supported(this));
  }

  // change ARR register
  std::uint16_t desired_arr_value = current_clock / p_frequency;

  // update ARR register to have new calculated ARR value
  bit_modify(reg->auto_reload_register).insert<arr_mask>(desired_arr_value);

  // need to update duty cycle according to new frequency
  driver_duty_cycle(previous_duty_cycle);
}
void pwm::driver_duty_cycle(float p_duty_cycle)
{
  pwm_reg_t* reg = get_pwm_reg(m_channel.peripheral_id);
  // current ARR value
  static constexpr auto ccr_mask = bit_mask::from<0, 15>();

  std::uint16_t desired_ccr_value = reg->auto_reload_register * p_duty_cycle;

  std::uint32_t compare_register = get_capture_compare_register(m_channel, reg);

  bit_modify(compare_register).insert<ccr_mask>(desired_ccr_value);
}
}  // namespace hal::stm32f1
