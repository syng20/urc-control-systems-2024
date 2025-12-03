// derived from drivers/applications/velocity_test.cpp

#include <libhal-util/can.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/can.hpp>
#include <libhal/error.hpp>
#include <libhal/pointers.hpp>

#include <bldc_servo.hpp>
#include <type_traits>
#include <array>

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
  
  hal::print(*console, "Shoulder figure 8\n");
  float high_val = -100; 
  float mid_val = -70; 
  float low_val = -15; 
  float bounds = 1.50; 
  int status = 0; 

  // float running_average = 0.0f; 
  // float prev_running_average = 0.0f; 
  // const int ra_total = 10; 
  // std::array<float, 10> ra_array{0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  // int ra_index = 0; 
  float ra_check = 0; 

  bldc_perseus::PID_settings pid_settings = {
    // .kp = 0.015,
    // .ki = 0.002,
    // .kd = 0.02,
    .kp = 0.05,
    .ki = 0.015, // 0.01
    .kd = 0.005,
  };
  bldc_perseus::PID_settings pid_set_zero = {
    .kp = 0.00,
    .ki = 0.00,
    .kd = 0.00,
  };  
  servo_ptr->update_pid_position(pid_settings);
  servo_ptr->set_target_position(mid_val);
  servo_ptr->update_position(1);
  // hal::print<128>(*console, "Target: %.2f\n", servo.get_target_position());
  hal::print<128>(*console, "Target: %.2f\n", servo_ptr->get_target_position());
  servo_ptr->set_pid_clamped_power(0.5); 

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
    // drivers::serial_commands::def{
    //   "setstatus",
    //   [&console, &servo_ptr](auto params) {
    //     if (params.size() != 1) {
    //       throw hal::argument_out_of_domain(nullptr);
    //     }
    //     int im = drivers::serial_commands::parse_float(params[0]);
    //     &status = im; 
    //     switch(im) {
    //       case 0: 
    //       case 2: 
    //       case 4: 
    //         servo.set_target_position(mid_val); 
    //         break; 
    //       case 1: 
    //         servo.set_target_position(high_val); 
    //         break; 
    //       case 3: 
    //         servo.set_target_position(low_val); 
    //         break; 
    //       default: 
    //         servo.set_target_position(mid_val); 
    //         status = 0; 
    //         break; 
    //     }
    //     hal::print<32>(*console, "Set status: %f\n", im);
    //   },
    // },
    // drivers::serial_commands::def{
    //   "reset",
    //   [&console, &servo_ptr](auto params) {
    //     if (params.size() != 1) {
    //       throw hal::argument_out_of_domain(nullptr);
    //     }
    //     servo_ptr->set_target_position(mid_val); 
    //     status = 0; 
    //     servo_ptr->update_position();
    //     hal::print<32>(*console, "Reset to center");
    //   },
    // },
    // drivers::serial_commands::def{ "" },
  };
  sjsu::drivers::serial_commands::handler cmd{ console };
  


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
    servo_ptr->update_position(0);
    
    float reading = servo_ptr->get_current_position();
    float pos = servo_ptr->get_target_position();
    hal::print<128>(*console, "Target: %.2f\n", servo_ptr->get_target_position());

    // prev_running_average = running_average; 
    // running_average = (running_average*ra_total - ra_array[ra_index] + reading) / ra_total; 
    // ra_check = abs(running_average - servo.get_target_position()); 
    // ra_array[ra_index] = reading; 
    // ra_index = (ra_index >= ra_total) ? 0 : ra_index + 1;
    // (ra_check < bounds) && (prev_running_average - running_average < bounds)
    ra_check = abs(abs(reading) - abs(pos)); 
    
    // case
    switch(status) {
      // set to mid 
      case 0: 
        if ((ra_check < bounds) && (reading != 0)) { 
          servo_ptr->set_target_position(high_val); 
          status = 1; 
          servo_ptr->update_pid_position(pid_set_zero);
          servo_ptr->update_position(1);
          hal::print<128>(*console, "Encoder reading: %.2f -- Target: %.2f -- Difference: %.2f -- Status: %d\n", reading, servo_ptr->get_target_position(), ra_check, status);
          hal::delay(*clock, 1000ms);
          servo_ptr->update_pid_position(pid_settings);
        }
        break; 
      // mid to high 
      case 1: 
        if ((ra_check < bounds)) { 
          servo_ptr->set_target_position(mid_val); 
          status = 2; 
          servo_ptr->update_pid_position(pid_set_zero);
          servo_ptr->update_position(1);
          hal::print<128>(*console, "Target: %.2f -- Difference: %.2f -- Encoder reading: %.2f -- Status: %d\n", servo_ptr->get_target_position(), ra_check, reading, status);
          hal::delay(*clock, 1000ms);
          servo_ptr->update_pid_position(pid_settings);
        }
        break; 
      // high to mid 
      case 2: 
        if ((ra_check < bounds)) { 
          servo_ptr->set_target_position(low_val); 
          status = 3; 
          servo_ptr->update_pid_position(pid_set_zero);
          servo_ptr->update_position(1);
          hal::print<128>(*console, "Target: %.2f -- Difference: %.2f -- Encoder reading: %.2f -- Status: %d\n", servo_ptr->get_target_position(), ra_check, reading, status);
          hal::delay(*clock, 1000ms);
          servo_ptr->update_pid_position(pid_settings);
        }
        break; 
      // mid to low 
      case 3: 
        if ((ra_check < bounds)) { 
          servo_ptr->set_target_position(mid_val); 
          status = 4; 
          servo_ptr->update_pid_position(pid_set_zero);
          servo_ptr->update_position(1);
          hal::print<128>(*console, "Target: %.2f -- Difference: %.2f -- Encoder reading: %.2f -- Status: %d\n", servo_ptr->get_target_position(), ra_check, reading, status);
          hal::delay(*clock, 1000ms);
          servo_ptr->update_pid_position(pid_settings);
        }
        break; 
      // low to mid 
      case 4: 
        if ((ra_check < bounds)) { 
          servo_ptr->set_target_position(high_val); 
          status = 1; 
          servo_ptr->update_pid_position(pid_set_zero);
          servo_ptr->update_position(1);
          hal::print<128>(*console, "Target: %.2f -- Difference: %.2f -- Encoder reading: %.2f -- Status: %d\n", servo_ptr->get_target_position(), ra_check, reading, status);
          hal::delay(*clock, 1000ms);
          servo_ptr->update_pid_position(pid_settings);
        }
        break; 
      default: 
        servo_ptr->set_target_position(mid_val); 
        status = 0; 
        servo_ptr->update_pid_position(pid_set_zero);
        servo_ptr->update_position(1);
          hal::print<128>(*console, "Target: %.2f -- Difference: %.2f -- Encoder reading: %.2f -- Status: %d\n", servo_ptr->get_target_position(), ra_check, reading, status);
        hal::delay(*clock, 1000ms);
        servo_ptr->update_pid_position(pid_settings);
        break; 
    }

    // hal::print<128>(*console, "Encoder reading: %.2f -- Target: %.2f -- Difference: %.2f -- Status: %d\n", reading, servo_ptr->get_target_position(), ra_check, status);
    hal::delay(*clock, 100ms);

  } 
}
}  // namespace sjsu::perseus