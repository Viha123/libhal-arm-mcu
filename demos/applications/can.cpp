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

#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/can.hpp>

#include <resource_list.hpp>

void print_can_message(hal::serial& p_console,
                       hal::can_message const& p_message)
{
  hal::print<256>(p_console,
                  "Received Message from ID: 0x%lX, length: %u \n"
                  "payload = [ 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, "
                  "0x%02X, 0x%02X, 0x%02X ]\n",
                  p_message.id(),
                  p_message.length,
                  p_message.payload[0],
                  p_message.payload[1],
                  p_message.payload[2],
                  p_message.payload[3],
                  p_message.payload[4],
                  p_message.payload[5],
                  p_message.payload[6],
                  p_message.payload[7]);
}

void application(resource_list& p_map)
{
  using namespace hal::literals;

  auto& clock = *p_map.clock.value();
  auto& can_transceiver = *p_map.can_transceiver.value();
  auto& can_bus_manager = *p_map.can_bus_manager.value();
  auto& can_interrupt = *p_map.can_interrupt.value();
  auto& console = *p_map.console.value();

  // Change the CAN baudrate here.
  static constexpr auto baudrate = 100.0_kHz;

  hal::print(console, "Starting CAN demo!\n");

  can_bus_manager.baud_rate(baudrate);

  can_interrupt.on_receive([&console](hal::can_interrupt::on_receive_tag,
                                      hal::can_message const& p_message) {
    hal::print(console, "Printing can message from interrupt!\n");
    print_can_message(console, p_message);
  });

  hal::u32 receive_cursor = 0;

  while (true) {
    using namespace std::chrono_literals;
    hal::can_message standard_message;

    standard_message.id(0x112).extended(false).remote_request(false);
    standard_message.length = 8;
    standard_message.payload = {
      0xAA, 0xBB, 0xCC, 0xDD, 0xDE, 0xAD, 0xBE, 0xEF,
    };

    hal::can_message standard_message2 = standard_message;
    standard_message2.id(0x333);
    standard_message2.length = 1;

    hal::can_message extended_message = standard_message;
    extended_message.extended(true).id(0x0123'4567);
    extended_message.length = 3;

    hal::can_message extended_message2 = standard_message;
    extended_message2.extended(true).id(0x0222'0005);
    extended_message.length = 5;

    hal::print(console, "Sending payload(s)...\n");

    can_transceiver.send(standard_message);
    can_transceiver.send(standard_message2);
    can_transceiver.send(extended_message);
    can_transceiver.send(extended_message2);

    hal::delay(clock, 1s);

    hal::print(console,
               "Printing received messages stored in circular buffer...\n");
    auto const buffer = can_transceiver.receive_buffer();
    auto cursor = can_transceiver.receive_cursor();
    for (; receive_cursor != cursor;
         receive_cursor = (receive_cursor + 1) % buffer.size()) {
      print_can_message(console, buffer[receive_cursor]);
      cursor = can_transceiver.receive_cursor();
    }

    hal::print(console, "Printing done.\n\n");
  }
}
