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

#include <array>
#include <cstdint>

#include <libhal-util/bit.hpp>
#include <libhal/error.hpp>

#include <libhal-stm32f1/pin.hpp>

namespace hal::stm32f1 {

struct alternative_function_io_t
{
  std::uint32_t volatile evcr;
  std::uint32_t volatile mapr;
  std::array<std::uint32_t, 4> volatile exticr;
  std::uint32_t reserved0;
  std::uint32_t volatile mapr2;
};

/**
 * @brief GPIO register map
 *
 */
struct gpio_t
{
  std::uint32_t volatile crl;
  std::uint32_t volatile crh;
  std::uint32_t volatile idr;
  std::uint32_t volatile odr;
  std::uint32_t volatile bsrr;
  std::uint32_t volatile brr;
  std::uint32_t volatile lckr;
};

/**
 * @brief Map the CONFIG flags for each pin use case
 *
 */
struct pin_config_t
{
  /// Configuration bit 1
  std::uint8_t CNF1;
  /// Configuration bit 0
  std::uint8_t CNF0;
  /// Mode bits
  std::uint8_t MODE;
  /// Output data register
  std::uint8_t PxODR;
};

/**
 * @brief Pin select structure
 *
 */
struct pin_select_t
{
  /// @brief Port letter, must be a capitol letter from 'A' to 'G'
  std::uint8_t port;
  /// @brief Pin number, must be between 0 to 15
  std::uint8_t pin;
};

static constexpr pin_config_t push_pull_gpio_output = {
  .CNF1 = 0,
  .CNF0 = 0,
  .MODE = 0b11,  // Default to high speed 50 MHz
  .PxODR = 0b0,  // Default to 0 LOW Voltage
};

static constexpr pin_config_t open_drain_gpio_output = {
  .CNF1 = 0,
  .CNF0 = 1,
  .MODE = 0b11,  // Default to high speed 50 MHz
  .PxODR = 0b0,  // Default to 0 LOW Voltage
};

static constexpr pin_config_t push_pull_alternative_output = {
  .CNF1 = 1,
  .CNF0 = 0,
  .MODE = 0b11,  // Default to high speed 50 MHz
  .PxODR = 0b0,  // Default to 0 LOW Voltage
};

static constexpr pin_config_t open_drain_alternative_output = {
  .CNF1 = 1,
  .CNF0 = 1,
  .MODE = 0b11,  // Default to high speed 50 MHz
  .PxODR = 0b0,  // Default to 0 LOW Voltage
};

static constexpr pin_config_t input_analog = {
  .CNF1 = 0,
  .CNF0 = 0,
  .MODE = 0b00,
  .PxODR = 0b0,  // Don't care
};

static constexpr pin_config_t input_float = {
  .CNF1 = 0,
  .CNF0 = 1,
  .MODE = 0b00,
  .PxODR = 0b0,  // Don't care
};

static constexpr pin_config_t input_pull_down = {
  .CNF1 = 1,
  .CNF0 = 0,
  .MODE = 0b00,
  .PxODR = 0b0,  // Pull Down
};

static constexpr pin_config_t input_pull_up = {
  .CNF1 = 1,
  .CNF0 = 0,
  .MODE = 0b00,
  .PxODR = 0b1,  // Pull Up
};

/**
 * @brief Construct pin manipulation object
 *
 * @param p_pin_select - the pin to configure
 * @param p_config - Configuration to set the pin to
 */
void configure_pin(pin_select_t p_pin_select, pin_config_t p_config);

/**
 * @brief Remap can pins
 *
 * @param p_pin_select - pair of pins to select
 */
void remap_pins(can_pins p_pin_select);

/**
 * @brief Returns the gpio register based on the port
 *
 * @param p_port - port letter, must be from 'A' to 'G'
 * @return gpio_t& - gpio register map
 */
gpio_t& gpio(std::uint8_t p_port);

inline auto* alternative_function_io =
  reinterpret_cast<alternative_function_io_t*>(0x4001'0000);
inline auto* gpio_a_reg = reinterpret_cast<gpio_t*>(0x4001'0800);
inline auto* gpio_b_reg = reinterpret_cast<gpio_t*>(0x4001'0c00);
inline auto* gpio_c_reg = reinterpret_cast<gpio_t*>(0x4001'1000);
inline auto* gpio_d_reg = reinterpret_cast<gpio_t*>(0x4001'1400);
inline auto* gpio_e_reg = reinterpret_cast<gpio_t*>(0x4001'1800);
inline auto* gpio_f_reg = reinterpret_cast<gpio_t*>(0x4001'1c00);
inline auto* gpio_g_reg = reinterpret_cast<gpio_t*>(0x4001'2000);

struct pin_remap
{
  static constexpr auto adc2_etrgreg_remap = hal::bit_mask::from<20>();
  static constexpr auto adc2_etrginj_remap = hal::bit_mask::from<19>();
  static constexpr auto adc1_etrgreg_remap = hal::bit_mask::from<18>();
  static constexpr auto adc1_etrginj_remap = hal::bit_mask::from<17>();
  static constexpr auto tim5ch4_iremap = hal::bit_mask::from<16>();
  static constexpr auto pd01_remap = hal::bit_mask::from<15>();
  static constexpr auto can1_remap = hal::bit_mask::from<13, 14>();
  static constexpr auto tim4_rempap = hal::bit_mask::from<12>();
  static constexpr auto tim3_rempap = hal::bit_mask::from<10, 11>();
  static constexpr auto tim2_rempap = hal::bit_mask::from<8, 9>();
  static constexpr auto tim1_rempap = hal::bit_mask::from<6, 7>();
  static constexpr auto usart3_remap = hal::bit_mask::from<4, 5>();
  static constexpr auto usart2_remap = hal::bit_mask::from<3>();
  static constexpr auto usart1_remap = hal::bit_mask::from<2>();
  static constexpr auto i2c1_remap = hal::bit_mask::from<1>();
  static constexpr auto spi1_remap = hal::bit_mask::from<0>();
};

struct pin_remap2
{
  static constexpr auto ptp_pps_remap = hal::bit_mask::from<30>();
  static constexpr auto tim2itr1_iremap = hal::bit_mask::from<29>();
  static constexpr auto spi3_remap = hal::bit_mask::from<28>();
  static constexpr auto swj_cfg = hal::bit_mask::from<26, 24>();
  static constexpr auto mii_rmii_sel = hal::bit_mask::from<23>();
  static constexpr auto can2_remap = hal::bit_mask::from<22>();
  static constexpr auto eth_remap = hal::bit_mask::from<21>();

  static constexpr auto tim5ch4_iremap = hal::bit_mask::from<16>();
  static constexpr auto pd01_remap = hal::bit_mask::from<15>();
  static constexpr auto can1_remap = hal::bit_mask::from<14, 13>();
  static constexpr auto tim4_rempap = hal::bit_mask::from<12>();
  static constexpr auto tim3_rempap = hal::bit_mask::from<11, 10>();
  static constexpr auto tim2_rempap = hal::bit_mask::from<9, 8>();
  static constexpr auto tim1_rempap = hal::bit_mask::from<7, 6>();
  static constexpr auto usart3_remap = hal::bit_mask::from<5, 4>();
  static constexpr auto usart2_remap = hal::bit_mask::from<3>();
  static constexpr auto usart1_remap = hal::bit_mask::from<2>();
  static constexpr auto i2c1_remap = hal::bit_mask::from<1>();
  static constexpr auto spi1_remap = hal::bit_mask::from<0>();
};
}  // namespace hal::stm32f1
