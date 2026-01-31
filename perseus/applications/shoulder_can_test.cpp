// derived from drivers/applications/velocity_test.cpp

#include <libhal-util/can.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/can.hpp>
#include <libhal/error.hpp>
#include <libhal/pointers.hpp>

#include <bldc_servo.hpp>
#include <can_messaging.hpp>
#include <resource_list.hpp>


using namespace std::chrono_literals;
namespace sjsu::perseus {

enum servo_address : hal::u16
{
  track_servo = 0x120,
  shoulder_servo = 0x121,
  elbow_servo = 0x122,
  wrist_pitch = 0x123,
  wrist_roll = 0x124,
  clamp = 0x125
};

void print_can_message(hal::serial& p_console,
                       hal::can_message const& p_message)
{
  hal::print<96>(p_console,
                 "📩 Received new hal::can_message { \n"
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

// each rotation of the output shaft of the track servo is 8 mm of linear travel
// so 1 degree of rotation is 8mm / 360 = 0.0222 mm of linear travel
// 188:1 is for shoulder servo 5281.1 * 28
// 188:1 elbow 1-2 reduction. 5281.1 * 2
void application()
{
  using namespace hal::literals;

  // gen
  auto clock = resources::clock();
  auto console = resources::console();
  // can
  auto can_transceiver = resources::can_transceiver();
  auto can_bus_manager = resources::can_bus_manager();
  auto can_interrupt = resources::can_interrupt();
  auto can_id_filter = resources::can_identifier_filter();
  // bldc
  auto h_bridge = resources::h_bridge();
  auto encoder = resources::encoder();
  bldc_perseus servo(h_bridge, encoder);
  hal::print(*console, "BLDC Servo created...\n");
  auto servo_ptr = hal::v5::make_strong_ptr<decltype(servo)>(resources::driver_allocator(), std::move(servo));
  // can pt2
  constexpr servo_address allowed_id = shoulder_servo; 
  can_perseus servo_can(allowed_id); 
  hal::print(*console, "Servo can creature setup...\n");


  // Change the CAN baudrate here.
  static constexpr auto baudrate = 100.0_kHz;

  hal::print(*console, "CAN IT\n");

  can_bus_manager->baud_rate(baudrate);
  can_interrupt->on_receive([&console](hal::can_interrupt::on_receive_tag,
                                       hal::can_message const& p_message) {
    hal::print<64>(
      *console, "Can message with id = 0x%lX from interrupt!\n", p_message.id);
  });

  hal::print<32>(*console,
                 "Receiver buffer size = %zu\n",
                 can_transceiver->receive_buffer().size());


//   constexpr auto allowed_id = servo_address;
  hal::can_message_finder message_finder(*can_transceiver, allowed_id);
  can_id_filter->allow(allowed_id);
  hal::print<64>(
    *console, "🆔 Allowing ID [0x%lX] through the filter!\n", allowed_id);


//   hal::can_message response{
//     .id = 0x333,
//     .extended=false,
//     .remote_request=false,
//     .length = 0,
//     .payload = {},
//   };
  
  int set = 0; 

  while (true) {
    using namespace std::chrono_literals;
    

    auto msg = message_finder.find();
    
    if (msg) {
      print_can_message(*console, *msg);
      auto response = hal::v5::make_strong_ptr<hal::can_message>(resources::driver_allocator(), hal::can_message{});
      servo_can.can_perseus::process_can_message(*msg, servo_ptr, response); 
      message_finder.transceiver().send(*response);
      print_can_message(*console, *response);
      hal::print<64>(*console, "finished transmission\n");
      set = 2;
    }

    if (set == 2) { 
        servo_ptr->update_position(); 
        set = 1; 
    } 
    else if (set == 1) servo_ptr->update_position(); 

  }
  
}
}  // namespace sjsu::perseus