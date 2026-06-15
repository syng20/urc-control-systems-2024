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

#include "can_test_copy.cpp"


using namespace std::chrono_literals;
namespace sjsu::perseus {

enum servo_address : hal::u16
{
track_servo = 0x121,
  shoulder_servo = 0x122,
  elbow_servo = 0x123,
  wrist_left = 0x124,
  wrist_right = 0x125,
    clamp = 0x126,
};


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
  // CHANGE SERVO
  // track 
  constexpr servo_address allowed_id = track_servo; 
  // pid
  bldc_perseus::PID_settings pid_settings = {
    .kp = 0.04, 
    .ki = 0.00, 
    .kd = 0.00,
  };
  // servo_values 
  bldc_perseus::servo_values servo_values = {
    .gear_ratio = 16915.5, // 751.8 * 1 / 2 * 360 / 8 (for mm) 
    .feedforward_clamp = 0,
    .length = 0, 
    .angle_offset = 0, 
    .weight_beam = 0, 
    .weight_end = 0 
  }; 
  hal::print(*console, "Starting Track\n");
  can_application(allowed_id, pid_settings, servo_values); 
  
}
}  // namespace sjsu::perseus