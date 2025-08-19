#include <libhal-arm-mcu/stm32_generic/pwm.hpp>
#include <libhal-arm-mcu/stm32f1/constants.hpp>
#include <libhal-arm-mcu/stm32f1/pwm.hpp>
#include <libhal-arm-mcu/stm32f1/quadrature_encoder.hpp>
#include <libhal-arm-mcu/stm32f1/timer.hpp>
#include <libhal-util/bit.hpp>
#include <libhal-util/enum.hpp>
#include <libhal/error.hpp>
#include <libhal/units.hpp>

#include "power.hpp"

namespace hal::stm32f1 {
// Advanced timer
inline void* timer1 = reinterpret_cast<void*>(0x4001'2C00);
// General purpose timers 2 - 5
inline void* timer2 = reinterpret_cast<void*>(0x4000'0000);
inline void* timer3 = reinterpret_cast<void*>(0x4000'0400);
inline void* timer4 = reinterpret_cast<void*>(0x4000'0800);
inline void* timer5 = reinterpret_cast<void*>(0x4000'0C00);
// Advanced timer
inline void* timer8 = reinterpret_cast<void*>(0x4001'3400);
// General purpose timers 9 - 14
inline void* timer9 = reinterpret_cast<void*>(0x4001'4C00);
inline void* timer10 = reinterpret_cast<void*>(0x4001'5000);
inline void* timer11 = reinterpret_cast<void*>(0x4001'5400);
inline void* timer12 = reinterpret_cast<void*>(0x4000'1800);
inline void* timer13 = reinterpret_cast<void*>(0x4000'1C00);
inline void* timer14 = reinterpret_cast<void*>(0x4000'2000);

namespace {
hal::u16 encoder_availability =
  0;  // only quadrature_encoder will change the value of this.
hal::u16 pwm_availability;  // pwm will change the value of this

template<peripheral select>
void* peripheral_to_advanced_register()
{
  void* reg;
  if constexpr (select == peripheral::timer1) {
    reg = timer1;
  } else {
    reg = timer8;
  }
  return reg;
}

template<peripheral select>
void* peripheral_to_general_register()
{
  void* reg;
  if constexpr (select == peripheral::timer2) {
    reg = timer2;
  } else if constexpr (select == peripheral::timer3) {
    reg = timer3;
  } else if constexpr (select == peripheral::timer4) {
    reg = timer4;
  } else if constexpr (select == peripheral::timer5) {
    reg = timer5;
  } else if constexpr (select == peripheral::timer9) {
    reg = timer9;
  } else if constexpr (select == peripheral::timer10) {
    reg = timer10;
  } else if constexpr (select == peripheral::timer11) {
    reg = timer11;
  } else if constexpr (select == peripheral::timer12) {
    reg = timer12;
  } else if constexpr (select == peripheral::timer13) {
    reg = timer13;
  } else {
    reg = timer14;
  }
  return reg;
}

template<peripheral select>
u16 peripheral_to_bit_index()
{
  u16 index = 0;
  if constexpr (select == peripheral::timer1) {
    index = 0;
  } else if constexpr (select == peripheral::timer2) {
    index = 1;
  } else if constexpr (select == peripheral::timer3) {
    index = 2;
  } else if constexpr (select == peripheral::timer4) {
    index = 3;
  } else if constexpr (select == peripheral::timer5) {
    index = 4;
  } else if constexpr (select == peripheral::timer8) {
    index = 5;
  } else if constexpr (select == peripheral::timer9) {
    index = 6;
  } else if constexpr (select == peripheral::timer10) {
    index = 7;
  } else if constexpr (select == peripheral::timer11) {
    index = 8;
  } else if constexpr (select == peripheral::timer12) {
    index = 9;
  } else if constexpr (select == peripheral::timer13) {
    index = 10;
  } else {
    index = 11;
  }
  return index;
}
template<peripheral select>
bool can_acquire_pwm(bit_mask const timer_idx)
{
  // pwm can be acquired as long as an encoder is not being used for that
  // peripheral further channel specific checks are done in pwm
  // class.
  if (hal::bit_extract(timer_idx, encoder_availability))
    return false;
  return true;
}

template<peripheral select>
bool can_acquire_encoder(bit_mask const timer_idx)
{
  if (hal::bit_extract(timer_idx, encoder_availability))
    return false;
  if (hal::bit_extract(timer_idx, pwm_availability))
    return false;
  return true;
}
}  // namespace

template<peripheral select>
advanced_timer<select>::advanced_timer()
{
  power_on(select);
}

template<peripheral select>
general_purpose_timer<select>::general_purpose_timer()
{
  power_on(select);
}

template<peripheral select>
hal::stm32f1::pwm advanced_timer<select>::acquire_pwm(pin_type p_pin)
{
  auto const timer_idx = bit_mask::from(peripheral_to_bit_index<select>());
  if (can_acquire_pwm<select>(timer_idx)) {
    bit_modify(pwm_availability).set(timer_idx);
  } else {
    hal::safe_throw(hal::device_or_resource_busy(this));
  }
  return { peripheral_to_advanced_register<select>(),
           select,
           true,
           static_cast<pins>(p_pin) };
}

template<peripheral select>
hal::stm32f1::pwm general_purpose_timer<select>::acquire_pwm(pin_type p_pin)
{
  auto const timer_idx = bit_mask::from(peripheral_to_bit_index<select>());
  if (can_acquire_pwm<select>(timer_idx)) {
    bit_modify(pwm_availability).set(timer_idx);
  } else {
    hal::safe_throw(hal::device_or_resource_busy(this));
  }
  return { peripheral_to_general_register<select>(),
           select,
           false,
           static_cast<pins>(p_pin) };
}

template<peripheral select>
hal::stm32f1::pwm16_channel advanced_timer<select>::acquire_pwm16_channel(
  pin_type p_pin)
{
  auto const timer_idx = bit_mask::from(peripheral_to_bit_index<select>());
  if (can_acquire_pwm<select>(timer_idx)) {
    bit_modify(pwm_availability).set(timer_idx);
  } else {
    hal::safe_throw(hal::device_or_resource_busy(this));
  }
  return { peripheral_to_advanced_register<select>(),
           select,
           true,
           static_cast<pins>(p_pin) };
}

template<peripheral select>
hal::stm32f1::pwm16_channel
general_purpose_timer<select>::acquire_pwm16_channel(pin_type p_pin)
{
  auto const timer_idx = bit_mask::from(peripheral_to_bit_index<select>());
  if (can_acquire_pwm<select>(timer_idx)) {
    bit_modify(pwm_availability).set(timer_idx);
  } else {
    hal::safe_throw(hal::device_or_resource_busy(this));
  }
  return { peripheral_to_general_register<select>(),
           select,
           false,
           static_cast<pins>(p_pin) };
}

template<peripheral select>
hal::stm32f1::pwm_group_frequency
advanced_timer<select>::acquire_pwm_group_frequency()
{
  auto const timer_idx = bit_mask::from(peripheral_to_bit_index<select>());
  if (can_acquire_pwm<select>(timer_idx)) {
    bit_modify(pwm_availability).set(timer_idx);
  } else {
    hal::safe_throw(hal::device_or_resource_busy(this));
  }
  return { peripheral_to_advanced_register<select>(), select };
}

template<peripheral select>
hal::stm32f1::pwm_group_frequency
general_purpose_timer<select>::acquire_pwm_group_frequency()
{
  auto const timer_idx = bit_mask::from(peripheral_to_bit_index<select>());
  if (can_acquire_pwm<select>(timer_idx)) {
    bit_modify(pwm_availability).set(timer_idx);
  } else {
    hal::safe_throw(hal::device_or_resource_busy(this));
  }
  return { peripheral_to_general_register<select>(), select };
}

template<peripheral select>
hal::stm32f1::quadrature_encoder
general_purpose_timer<select>::acquire_quadrature_encoder(pin_type channel_a,
                                                          pin_type channel_b)
{
  auto const idx = bit_mask::from(peripheral_to_bit_index<select>());
  if (can_acquire_encoder<select>(idx)) {
    bit_modify(encoder_availability).set(idx);
  } else {
    hal::safe_throw(hal::device_or_resource_busy(nullptr));
  }
  return { static_cast<pins>(channel_a),
           static_cast<pins>(channel_b),
           select,
           peripheral_to_general_register<select>() };
}

template<peripheral select>
hal::stm32f1::quadrature_encoder
advanced_timer<select>::acquire_quadrature_encoder(pin_type channel_a,
                                                   pin_type channel_b)
{
  auto const idx = bit_mask::from(peripheral_to_bit_index<select>());
  if (can_acquire_encoder<select>(idx)) {
    bit_modify(encoder_availability).set(idx);
  } else {
    hal::safe_throw(hal::device_or_resource_busy(nullptr));
  }
  return { static_cast<pins>(channel_a),
           static_cast<pins>(channel_b),
           select,
           peripheral_to_advanced_register<select>() };
}
// Tell the compiler which instances to generate
template class advanced_timer<peripheral::timer1>;
template class advanced_timer<peripheral::timer8>;
template class general_purpose_timer<peripheral::timer2>;
template class general_purpose_timer<peripheral::timer3>;
template class general_purpose_timer<peripheral::timer4>;
template class general_purpose_timer<peripheral::timer5>;
template class general_purpose_timer<peripheral::timer9>;
template class general_purpose_timer<peripheral::timer10>;
template class general_purpose_timer<peripheral::timer11>;
template class general_purpose_timer<peripheral::timer12>;
template class general_purpose_timer<peripheral::timer13>;
template class general_purpose_timer<peripheral::timer14>;
}  // namespace hal::stm32f1
