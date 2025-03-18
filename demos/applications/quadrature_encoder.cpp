#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>

#include <resource_list.hpp>

void application(resource_list& p_map)
{
  using namespace std::chrono_literals;
  using namespace hal::literals;

  auto& encoder = *p_map.quad_encoder.value();
  auto& console = *p_map.console.value();
  auto& clock = *p_map.clock.value();
  hal::print<1024>(console, "here!!\n");

  while (true) {
    auto readings = encoder.read();
    hal::print<1024>(console, "Reading: %f\n", readings.angle);
    hal::delay(clock, 1s);
  }
}