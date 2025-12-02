// derived from drivers/applications/velocity_test.cpp

#include <libhal-util/can.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/can.hpp>
#include <libhal/error.hpp>
#include <libhal/pointers.hpp>

#include <bldc_servo.hpp>
#include <type_traits>

#include "../hardware_map.hpp"
#include "../include/bldc_servo.hpp" 
#include "../../drivers/include/serial_commands.hpp"

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

// each rotation of the output shaft of the track servo is 8 mm of linear travel
// so 1 degree of rotation is 8mm / 360 = 0.0222 mm of linear travel
// 188:1 is for shoulder servo 5281.1 * 28
// 188:1 elbow 1-2 reduction. 5281.1 * 2
void application()
{
  using namespace std::chrono_literals;
  using namespace hal::literals;
  auto console = resources::console();
  auto clock = resources::clock();
  auto h_bridge = resources::h_bridge();
  auto encoder = resources::encoder();
  
  bldc_perseus servo(h_bridge, encoder);
  hal::print(*console, "BLDC Servo created...\n");

  auto servo_ptr = hal::v5::make_strong_ptr<decltype(servo)>(resources::driver_allocator(), std::move(servo));
  
  hal::print(*console, "Track figure 8\n");
  float far_val = 25; 
  float close_val = 0; 
//   float p = 0.25; 
  int dir = 0; 
//   servo_ptr->set_target_position(close_val);
//   servo_ptr->set_power(-0.3); 

//   bldc_perseus::PID_settings pid_settings = {
//     .kp = 1,
//     .ki = 0,
//     .kd = 0,
//   };
//   servo_ptr->update_pid_position(pid_settings);
//   servo_ptr->set_target_position(start_val);
  std::array cmd_defs = {
    drivers::serial_commands::def{
      "setpos",
      [&console, &servo_ptr](auto params) {
        if (params.size() != 1) {
          throw hal::argument_out_of_domain(nullptr);
        }
        float position = drivers::serial_commands::parse_float(params[0]);
        servo_ptr->set_target_position(position);
        hal::print<32>(*console, "Set Position to: %f\n", position);
      },
    },
    drivers::serial_commands::def{
      "setkp",
      [&console, &servo_ptr](auto params) {
        if (params.size() != 1) {
          throw hal::argument_out_of_domain(nullptr);
        }
        float kp = drivers::serial_commands::parse_float(params[0]);
        auto current_settings = servo_ptr->get_pid_settings();
        current_settings.kp = kp;
        servo_ptr->update_pid_position(current_settings);
        hal::print<32>(*console, "Set Kp to: %f\n", kp);
      },
    },
    drivers::serial_commands::def{
      "setki",
      [&console, &servo_ptr](auto params) {
        if (params.size() != 1) {
          throw hal::argument_out_of_domain(nullptr);
        }
        float ki = drivers::serial_commands::parse_float(params[0]);
        auto current_settings = servo_ptr->get_pid_settings();
        current_settings.ki = ki;
        servo_ptr->update_pid_position(current_settings);
        hal::print<32>(*console, "Set Ki to: %f\n", ki);
      },
    },
    drivers::serial_commands::def{
      "setkd",
      [&console, &servo_ptr](auto params) {
        if (params.size() != 1) {
          throw hal::argument_out_of_domain(nullptr);
        }
        float kd = drivers::serial_commands::parse_float(params[0]);
        auto current_settings = servo_ptr->get_pid_settings();
        current_settings.kd = kd;
        servo_ptr->update_pid_position(current_settings);
        hal::print<32>(*console, "Set Kd to: %f\n", kd);
      },
    },
    drivers::serial_commands::def{
      "maxpower",
      [&console, &servo_ptr](auto params) {
        if (params.size() != 1) {
          throw hal::argument_out_of_domain(nullptr);
        }
        float power = drivers::serial_commands::parse_float(params[0]);
        servo_ptr->set_pid_clamped_power(power);
        hal::print<32>(*console, "Set max power: %f\n", power);
      },
    },
    // drivers::serial_commands::def{ "" },
  };
  sjsu::drivers::serial_commands::handler cmd{ console };

  // encoder returns amount rotated
  // need to convert to linear distance (*8/(360*10) to get cm)
  
  while (true) {
    // servo_ptr->update_position();
    auto reading = servo.get_current_position();
    hal::print<128>(*console, "Encoder degrees: %.2f -- ", reading);
    reading = reading * 8 / 3600; 
    if (reading <= close_val) {
        servo_ptr->set_target_position(far_val);
        servo_ptr->set_power(0.167); 
        dir = 1; 
        hal::print<128>(*console, "SWITCH TO FAR\n");
    }
    if (reading >= far_val) { 
        servo_ptr->set_target_position(close_val);
        servo_ptr->set_power(-0.167); 
        dir = 0; 
        hal::print<128>(*console, "SWITCH TO CLOSE\n");
    }

    hal::print<128>(*console, "Encoder reading: %.2f -- Dir: %d\n", reading, dir);
    hal::delay(*clock, 100ms);

  } 
}
}  // namespace sjsu::perseus