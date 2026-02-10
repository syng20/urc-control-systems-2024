// derived from drivers/applications/velocity_test.cpp

#include <libhal-util/can.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/can.hpp>
#include <libhal/error.hpp>
#include <libhal/pointers.hpp>

#include <serial_commands.hpp>
#include <h_bridge.hpp>
#include <bldc_servo.hpp>
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
  
  hal::print(*console, "Track move by time\n");
  
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
        auto current_settings = servo_ptr->get_position_pid_settings();
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
        auto current_settings = servo_ptr->get_position_pid_settings();
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
        auto current_settings = servo_ptr->get_position_pid_settings();
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

  float pow = -0.3; 
  servo_ptr->set_pid_clamped_power(pow);
  auto reading = servo.get_target_position();
  
  while (true) {

    try {
      cmd.handle(cmd_defs);
    } catch (hal::exception e) {
      switch (e.error_code()) {
        case std::errc::argument_out_of_domain:
          hal::print(*console, "Error: invalid argument length or type\n");
          break;
        default:
          hal::print<32>(*console, "Error code: %d\n", e.error_code());
          break;
      }
    }

    // back and forth by timing 
    // set (-1) for away from motor 
    // set (+1) for towards motor 
    servo_ptr->set_power(pow);
    reading = servo.get_target_position();
    hal::print<128>(*console, "Power: %.2f\n", pow);
    hal::print<128>(*console, "Encoder reading: %.2f -- -- Power: %.2f\n", reading, pow);
    hal::delay(*clock, 2000ms);


    // // move back and forth only via 'maxpower' command
    // auto reading = servo.get_current_position();
    // float pow = servo_ptr->get_pid_clamped_power(); 
    // servo_ptr->set_power(pow); 
    // hal::print<128>(*console, "Encoder reading: %.2f -- Dir: %d -- Power: %.2f\n", reading, dir, pow);
    // hal::delay(*clock, 100ms);

  } 
}
}  // namespace sjsu::perseus