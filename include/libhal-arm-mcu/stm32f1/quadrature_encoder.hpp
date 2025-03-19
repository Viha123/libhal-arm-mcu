#pragma once

#include "libhal-arm-mcu/stm32_generic/quadrature_encoder.hpp"
#include "libhal-arm-mcu/stm32f1/pwm.hpp"
#include <libhal/rotation_sensor.hpp>
namespace hal::stm32f1 {
class quadrature_encoder : public hal::rotation_sensor
{

public:
  // should be able to take any timer pin, but the timer should be the same pin.
  // when this encoder is acquired by gptimer or advanced timer, it will ensure
  // that the channels are part of the same timer.
  template<hal::stm32f1::peripheral select>
  friend class hal::stm32f1::advanced_timer;

  template<hal::stm32f1::peripheral select>
  friend class hal::stm32f1::general_purpose_timer;

  /** @brief This is the unsafe way to construct the encoder object.
   *
   * It is marked unsafe because the quadrature_encoder must be constructed
   * through the general_purpose_timer or the advanced_timer classes.
   *
   * If the user is instantiating it without the timer classes, they must take
   * care to ensure that both pins are part of the same timer channel.
   *
   * Finally, no other timer related object must be running on the other timer
   * channels.
   */
  quadrature_encoder(hal::unsafe,
                     hal::stm32f1::pins channel1,
                     hal::stm32f1::pins channel2,
                     hal::stm32f1::peripheral p_select,
                     void* p_reg);

private:
  quadrature_encoder(hal::stm32f1::pins channel1,
                     hal::stm32f1::pins channel2,
                     hal::stm32f1::peripheral p_select,
                     void* p_reg);

  read_t driver_read() override;
  hal::stm32_generic::quadrature_encoder m_encoder;
};

}  // namespace hal::stm32f1
