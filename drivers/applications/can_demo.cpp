#include "../hardware_map.hpp"
#include <libhal-armcortex/dwt_counter.hpp>
#include <libhal-armcortex/startup.hpp>
#include <libhal-armcortex/system_control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/units.hpp>
#include <libhal-arm-mcu/stm32f1/can.hpp>
#include <libhal-arm-mcu/stm32f1/can2.hpp>

#include <libhal-util/can.hpp>
#include <libhal/can.hpp>

namespace sjsu::drivers {

  void print_can_message(hal::serial& p_console,
                       hal::can_message const& p_message)
{
  hal::print<96>(p_console,
                 "Received new hal::can_message { \n"
                 "    id: 0x%lX,\n"
                 "    length: %u \n"
                 "    payload = [ ",
                 p_message.id,
                 p_message.length);

  for (auto const& byte : p_message.payload) {
    hal::print<8>(p_console, "0x%02X, ", byte);
  }

  hal::print(p_console, "]\n}\n");
}

void application()
{
  using namespace hal::literals;

  auto clock = resources::clock();
  auto can_transceiver = resources::can_transceiver();
  auto can_bus_manager = resources::can_bus_manager();
  auto can_interrupt = resources::can_interrupt();
  auto can_id_filter = resources::can_identifier_filter();
  auto console = resources::console();

  // Change the CAN baudrate here.
  static constexpr auto baudrate = 100.0_kHz;

  hal::print(*console, "Starting CAN demo!\n");

  can_bus_manager->baud_rate(baudrate);
  can_interrupt->on_receive([&console](hal::can_interrupt::on_receive_tag,
                                       hal::can_message const& p_message) {
    hal::print<64>(
      *console, "Can message with id = 0x%lX from interrupt!\n", p_message.id);
  });

  hal::print<32>(*console,
                 "Receiver buffer size = %zu\n",
                 can_transceiver->receive_buffer().size());

  constexpr auto allowed_id = 0x111;
  can_id_filter->allow(allowed_id);
  hal::print<64>(
    *console, "Allowing ID [0x%lX] through the filter!\n", allowed_id);

  hal::can_message_finder message_finder(*can_transceiver, 0x111);

  while (true) {
    using namespace std::chrono_literals;
    hal::can_message standard_message {
      .id=0x112,
      .extended=false,
      .remote_request=false,
      .length = 8,
      .payload = {
        0xAA, 0xBB, 0xCC, 0xDD, 0xDE, 0xAD, 0xBE, 0xEF,
      },
    };

    hal::can_message standard_message2{
      .id = 0x333,
      .length = 0,
    };

    hal::can_message extended_message{
      .id = 0x0123'4567,
      .extended = true,
      .length = 3,
      .payload = { 0xAA, 0xBB, 0xCC },
    };

    hal::can_message extended_message2 {
      .id = 0x0222'0005,
      .extended = true,
      .length = 3,
      .payload = {
        0xAA, 0xBB, 0xCC,
      },
    };

    hal::print(*console, "Sending 4x payloads...\n");

    can_transceiver->send(standard_message);
    can_transceiver->send(standard_message2);
    can_transceiver->send(extended_message);
    can_transceiver->send(extended_message2);

    hal::delay(*clock, 1s);

    for (auto msg = message_finder.find(); msg.has_value();
         msg = message_finder.find()) {
      print_can_message(*console, *msg);
    }
  }
}
// void application()
// {
//   using namespace std::chrono_literals;
//   using namespace hal::literals;  
//   auto console = resources::console(); 
//   auto can = resources::can_transceiver(); 
//   hal::can_message_finder reader(can, 0x240);
//   auto receive_handler = [&console](hal::can::message_t const& p_message) {
//    hal::print<1024>(console,
//                      "Received Message from ID: 0x%lX, length: %u \n"
//                      "payload = [ 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, "
//                      "0x%02X, 0x%02X, 0x%02X ]\n",
//                      p_message.id,
//                      p_message.length,
//                      p_message.payload[0],
//                      p_message.payload[1],
//                      p_message.payload[2],
//                      p_message.payload[3],
//                      p_message.payload[4],
//                      p_message.payload[5],
//                      p_message.payload[6],
//                      p_message.payload[7]);
//   };
//   can.on_receive(receive_handler);
//   while (true) {
//     continue;
//   }
// }
}  // namespace sjsu::drivers