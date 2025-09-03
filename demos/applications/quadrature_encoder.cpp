#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>

#include <resource_list.hpp>

void application()
{
  using namespace std::chrono_literals;
  using namespace hal::literals;

  auto encoder = resources::quadrature_encoder();
  auto clock = resources::clock();
  auto console = resources::console();
  hal::print(*console, "hi starting");
  while (true) {
    auto readings = encoder->read();
    hal::print<28>(*console, "readings: %f\n", readings.angle);
    hal::delay(*clock, 1s);

  }

}