#include <libhal/error.hpp>
#include <utility>

#include "libhal-arm-mcu/stm32f1/constants.hpp"
#include "libhal-arm-mcu/stm32f1/timer.hpp"
#include "stm32f1/pin.hpp"
#include <libhal-arm-mcu/stm32f1/quadrature_encoder.hpp>
#include <libhal-util/bit.hpp>

namespace hal::stm32f1 {
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
  : m_encoder(hal::unsafe{})
{

  u8 channel_a = get_channel_from_pin(p_pin1, p_select);
  u8 channel_b = get_channel_from_pin(p_pin2, p_select);
  if (channel_a >= 3 || channel_b >= 3) {
    // only channels 1 and 2 are allowed for quadrature encoder mode.
    hal::safe_throw(hal::operation_not_permitted(this));
  }
  m_encoder.initialize(
    hal::unsafe{}, { .channel_a = channel_a, .channel_b = channel_b }, p_reg);
}

quadrature_encoder::quadrature_encoder(hal::unsafe,
                                       hal::stm32f1::pins p_pin1,
                                       hal::stm32f1::pins p_pin2,
                                       hal::stm32f1::peripheral p_select,
                                       void* p_reg)
  : m_encoder(hal::unsafe{})
{
  u8 channel_a = get_channel_from_pin(p_pin1, p_select);
  u8 channel_b = get_channel_from_pin(p_pin2, p_select);

  m_encoder.initialize(
    hal::unsafe{}, { .channel_a = channel_a, .channel_b = channel_b }, p_reg);
}

quadrature_encoder::read_t quadrature_encoder::driver_read()
{
  return m_encoder.read();
}

}  // namespace hal::stm32f1
