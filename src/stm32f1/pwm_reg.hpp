// Copyright 2024 Khalil Estell
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <cstdint>
namespace hal::stm32f1 {
/**
 * @brief Register map ofr the stm32f1 PWM (TIM1 and TIM8)
 */
struct pwm_reg_t
{
  ///Offset: 0x00 Control Register (R/W)
  std::uint32_t volatile control_register; //sets up timers
  /// Offset: 0x04 Control Register 2 (R/W)
  std::uint32_t volatile control_register_2;
  /// Offset: 0x08 Peripheral Mode Control Register (R/W)
  std::uint32_t volatile peripheral_control_register;
  /// Offset: 0x0C DMA/Interrupt enable register (R/W)
  std::uint32_t volatile interuupt_enable_register;
  /// Offset: 0x10 Status Register register (R/W)
  std::uint32_t volatile status_register;
  /// Offset: 0x14 Event Generator Register register (R/W)
  std::uint32_t volatile event_generator_register;
  /// Offset: 0x18 Capture/Compare mode register (R/W)
  std::uint32_t volatile capture_compare_mode_register; // set up modes for the ccrs
  /// Offset: 0x1C Capture/Compare mode register (R/W)
  std::uint32_t volatile capture_compare_mode_register_2;
  /// Offset: 0x20 Capture/Compare Enable register (R/W)
  std::uint32_t volatile cc_enable_register;
  /// Offset: 0x24 Counter (R/W)
  std::uint32_t volatile counter_register;
  /// Offset: 0x28 Prescalar (R/W)
  std::uint32_t volatile prescale_register;
   /// Offset: 0x2C Auto Reload Register (R/W)
  std::uint32_t volatile auto_reload_register; //affects frequency
  /// Offset: 0x30 Repetition Counter Register (R/W)
  std::uint32_t volatile repetition_counter_register;
  /// Offset: 0x34 Capture Compare Register (R/W)
  std::uint32_t volatile capture_compare_register; //affects duty cycles
  /// Offset: 0x38 Capture Compare Register (R/W)
  std::uint32_t volatile capture_compare_register_2;
  // Offset: 0x3C Capture Compare Register (R/W)
  std::uint32_t volatile capture_compare_register_3;
  // Offset: 0x40 Capture Compare Register (R/W)
  std::uint32_t volatile capture_compare_register_4;
  /// Offset: 0x44 Break and dead-time register
  std::uint32_t volatile break_and_deadtime_register;
  /// Offset: 0x48 DMA control register
  std::uint32_t volatile dma_control_register;
  /// Offset: 0x4C DMA address for full transfer
  std::uint32_t volatile dma_address_register;
};
inline pwm_reg_t* pwm_timer8 = reinterpret_cast<pwm_reg_t*>(0x4001'3400); //TIM8 timer (Does not exist on stm32f1? maybe?)
inline pwm_reg_t* pwm_timer1 =
  reinterpret_cast<pwm_reg_t*>(0x4001'2C00);  // TIM1 timer
// inline pwm_reg_t* pwm_timer2 = reinterpret_cast<pwm_reg_t*>(0x4000'0000);
// inline pwm_reg_t* pwm_timer9 = reinterpret_cast<pwm_reg_t*>(0x4001'4C00);
// inline pwm_reg_t* pwm_timer14 = reinterpret_cast<pwm_reg_t*>(0x4000'2000); 


}