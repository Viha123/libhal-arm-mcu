#include "libhal-arm-mcu/stm32f1/pwm.hpp"
#include <libhal/rotation_sensor.hpp>
namespace hal::stm32f1 {
class quadrature_encoder : public hal::rotation_sensor
{

public:
  // should be able to take any timer pin, but the timer should be the same pin.
  // when this encoder is acquired by gptimer or advanced timer, it will ensure
  // that the channels are part of the same timer.
  quadrature_encoder(hal::stm32f1::pins channel1,
                     hal::stm32f1::pins channel2,
                     hal::stm32f1::peripheral p_select,
                     void* p_reg);

private:
  read_t driver_read() override;
  void* m_reg = nullptr;
};

}  // namespace hal::stm32f1
