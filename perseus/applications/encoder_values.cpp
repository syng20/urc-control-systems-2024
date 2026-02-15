// derived from drivers/applications/velocity_test.cpp

#include <libhal-util/can.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/can.hpp>
#include <libhal/error.hpp>
#include <libhal/pointers.hpp>
#include <libhal/output_pin.hpp>

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
  
  hal::print(*console, "ENCODER TESTING\n");
  

  float pow = -0.3; 
  servo_ptr->set_pid_clamped_power(pow);
  auto reading = servo_ptr->get_reading_position();

  while (true) {

    reading = servo_ptr->get_reading_position();
    hal::print<128>(*console, "Encoder direct reading: %.2f -- Encoder divided reading: %.2f  -- Power: %.2f\n", servo_ptr->read_angle(), reading, pow);

    
    hal::delay(*clock, 100ms);

  } 
}


}  // namespace sjsu::perseus