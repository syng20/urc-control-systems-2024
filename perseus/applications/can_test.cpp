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

bool action_loop(hal::v5::strong_ptr<bldc_perseus> bldc,
                        hal::v5::strong_ptr<can_perseus> can,
                        hal::v5::strong_ptr<hal::can_message> response,
                        uint32_t action, 
                        bool new_action, 
                        int delay_counter)
{   
  bool send = false; 
  auto console = resources::console();
  switch (static_cast<can_perseus::action>(action)) {
    case can_perseus::action::homing: {
      bldc->home_encoder(); 
      if (delay_counter >= 10) {
        delay_counter = 0; 
        hal::u16 t = can->can_perseus::floating_to_fixed_point(bldc->get_reading_position(), 6); 
        response->length = 8;
        response->payload[0] = 0x20 + 0x50; 
        response->payload[1] = static_cast<hal::byte>(t >> 8) & 0xFF; // HIGH BYTE FIRST 
        response->payload[2] = static_cast<hal::byte>(t >> 0) & 0xFF;  // LOW BYTE SECOND
        send = true; 
      }
      break; 
    }
    case can_perseus::action::set_position: {
      if (new_action) {
        bldc->update_position(1); 
      }
      else {
        bldc->update_position(0); 
      }
      if (delay_counter >= 10) {
        delay_counter = 0; 
        hal::u16 t = can->can_perseus::floating_to_fixed_point(bldc->get_reading_position(), 6); 
        response->length = 8;
        response->payload[0] = 0x20 + 0x50; 
        response->payload[1] = static_cast<hal::byte>(t >> 8) & 0xFF; // HIGH BYTE FIRST 
        response->payload[2] = static_cast<hal::byte>(t >> 0) & 0xFF;  // LOW BYTE SECOND
        send = true; 
      }
      break;
    }
    default:
      break; 
  }
  return send; 
}



// each rotation of the output shaft of the track servo is 8 mm of linear travel
// so 1 degree of rotation is 8mm / 360 = 0.0222 mm of linear travel
// 188:1 is for shoulder servo 5281.1 * 28
// 188:1 elbow 1-2 reduction. 5281.1 * 2
void application()
{
  using namespace hal::literals;
  using namespace std::chrono_literals;

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
  servo_ptr->set_pid_clamped_power(0.3); 
  // can pt2
  constexpr servo_address allowed_id = elbow_servo; 
  can_perseus servo_can(allowed_id); 
  auto can_ptr = hal::v5::make_strong_ptr<decltype(servo_can)>(resources::driver_allocator(), std::move(servo_can));
  hal::print(*console, "Servo can creature setup...\n");


  // Change the CAN baudrate here.
  static constexpr auto baudrate = 1_MHz;

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
  hal::can_message_finder message_finder(*can_transceiver, allowed_id);
  auto message_finder_pointer = hal::v5::make_strong_ptr<hal::can_message_finder>(resources::driver_allocator(), hal::can_message_finder(*can_transceiver, allowed_id));
  can_id_filter->allow(allowed_id);
  hal::print<64>(
    *console, "🆔 Allowing ID [0x%lX] through the filter!\n", allowed_id);

/*
  //  hal::can_message spam_message{
  //    .id = 0x333,
  //    .extended=false,
  //    .remote_request=false,
  //    .length = 3,
  //    .payload = {0x12,0x23,0x38},
  //  };

  //  hal::can_message na_message{
  //    .id = 0x333,
  //    .extended=false,
  //    .remote_request=false,
  //    .length = 3,
  //    .payload = {0xaa,0xbb,0xcc},
  // };
*/

  // elbow
  bldc_perseus::PID_settings pid_settings = {
    .kp = 0.05, //0.001, //0.05,
    .ki = 0.0015,//0.00001, //0.015, 
    .kd = 0.005,
  };
  // // shoulder
  // bldc_perseus::PID_settings pid_settings = {
  //   .kp = 5.0,
  //   .ki = 0.01,
  //   .kd = 0.00,
  // };
  servo_ptr->update_pid_position(pid_settings);
  
  
  volatile uint32_t action = 0x00;
  hal::u16 t = 0;
  bool new_action = false; 
  int delay_counter = 0; 

  // hal::can_message response{
  //   .id = 0x000,
  //   .extended=false,
  //   .remote_request=false,
  //   .length = 0,
  //   .payload = {},
  // };


  while (true) {
    
    // received and response 
    auto msg = message_finder.find();
    std::optional<hal::can_message> response_ptr;
    
    // if message found 
    if (msg) {
      // process message 
      print_can_message(*console, *msg);
      can_ptr->can_perseus::process_can_message(*msg, allowed_id, servo_ptr, response_ptr); 
      // send response 
      message_finder.transceiver().send(*response_ptr);
      print_can_message(*console, *response_ptr);
      hal::print<64>(*console, "finished transmission\n");
      // set action 
      action = servo_ptr->bldc_perseus::get_reading_action(); 
      hal::print<64>(*console, "Action: %x \n", action);
    }

    // continue action if necessary 
    switch (static_cast<can_perseus::action>(action)) {
      case can_perseus::action::homing: {
        servo_ptr->home_encoder(); 
        if (delay_counter >= 6) {
          delay_counter = 0; 
          t = can_ptr->can_perseus::floating_to_fixed_point(servo_ptr->get_reading_position(), 6); 
          response_ptr->length = 8;
          response_ptr->payload[0] = 0x20 + 0x50; 
          response_ptr->payload[1] = static_cast<hal::byte>(t >> 8) & 0xFF; // HIGH BYTE FIRST 
          response_ptr->payload[2] = static_cast<hal::byte>(t >> 0) & 0xFF;  // LOW BYTE SECOND
          message_finder.transceiver().send(*response_ptr);
        }
        break; 
      }
      case can_perseus::action::set_position: {
        if (new_action) {
          servo_ptr->update_position(1); 
        }
        else {
          servo_ptr->update_position(0); 
        }
        if (delay_counter >= 6) {
          delay_counter = 0; 
          t = can_ptr->can_perseus::floating_to_fixed_point(servo_ptr->get_reading_position(), 6); 
          response_ptr->length = 8;
          response_ptr->payload[0] = 0x20 + 0x50; 
          response_ptr->payload[1] = static_cast<hal::byte>(t >> 8) & 0xFF; // HIGH BYTE FIRST 
          response_ptr->payload[2] = static_cast<hal::byte>(t >> 0) & 0xFF;  // LOW BYTE SECOND
          message_finder.transceiver().send(*response_ptr);
        }
        break;
      }
      default:
        break; 
    }
    /*
    // if (action_loop(servo_ptr, can_ptr, response_c, action, new_action, delay_counter) == true) {
    //   message_finder.transceiver().send(*response_c);
    //   print_can_message(*console, *response_c);
    //   hal::print<64>(*console, "finished transmission\n");
    // }
    */

    /*
    // for (int i = 0; i < response_ptr->length; i++) {
    //   response_ptr->payload[i] = 0x00; 
    // }
    // response_ptr->length = 0; 
    // response_ptr->id=0x000; 
    */

    new_action = false; 
    delay_counter++; 
    // if (new_action) delay_counter++;
    hal::delay(*clock, 50ms); 

/*
    // action continuation 
    // 0x12 = update position 
    if (action == 0x12) { 
      if (yepyep == 0) { 
        servo_ptr->update_position(1); 
        hal::print(*console, "From Scratch = 1 \n");
        yepyep = 1; 
      }
      servo_ptr->update_position(0); 
      hal::print(*console, "From Scratch = 0 \n");
      float d = fabs(fabs(servo_ptr->get_reading_position()) - fabs(servo_ptr->get_target_position())); 
      
      hal::print<64>(*console, "fabs(fabs(%.2f) - fabs(%.2f)) = %.2f -- power: %.2f \n", servo_ptr->get_reading_position(), servo_ptr->get_target_position(), d, servo_ptr->get_power());
      hal::delay(*clock, 100ms);
      
    } 
*/


  }
  
}
}  // namespace sjsu::perseus