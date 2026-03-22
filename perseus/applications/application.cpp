//copied from drivers/applications/h_bridge_demo.cpp

#include <h_bridge.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/can.hpp>
#include <libhal/error.hpp>
#include <libhal/output_pin.hpp>
#include <libhal/pointers.hpp>

#include <bldc_servo.hpp>
#include <resource_list.hpp>


using namespace std::chrono_literals;
namespace sjsu::perseus {

// each rotation of the output shaft of the track servo is 8 mm of linear travel
// so 1 degree of rotation is 8mm / 360 = 0.0222 mm of linear travel
// 188:1 is for shoulder servo 5281.1 * 28
// 188:1 elbow 1-2 reduction. 5281.1 * 2
void application()
{
  using namespace hal::literals;
  using namespace std::chrono_literals;
  // auto pwm_pin = resources::pwm_pin(); 
  // pwm_pin->level(false); 

  // gen
  auto clock = resources::clock();
  auto console = resources::console();
  // bldc
  auto h_bridge = resources::h_bridge();
  auto encoder = resources::encoder();
  bldc_perseus servo(h_bridge, encoder);
  hal::print(*console, "BLDC Servo created...\n");
  auto servo_ptr = hal::v5::make_strong_ptr<decltype(servo)>(resources::driver_allocator(), std::move(servo));
  
  while(true) {
  
    servo_ptr->set_power(0.3); 
    hal::delay(*clock, 5000ms);
    hal::print(*console, "Switch -0.3\n");
    servo_ptr->set_power(-0.3); 
    hal::delay(*clock, 5000ms);
    hal::print(*console, "Switch 0.3\n");
  

  }

}
}  // namespace sjsu::perseus