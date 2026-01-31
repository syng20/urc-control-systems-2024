#include <cmath>
#include <libhal-util/can.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/can.hpp>
#include <libhal/error.hpp>
#include <libhal/pointers.hpp>

#include <resource_list.hpp>
#include <bldc_servo.hpp>
#include <can_messaging.hpp>

using namespace std::chrono_literals;
namespace sjsu::perseus {

can_perseus::can_perseus(hal::u16 curr_servo_addr) : m_curr_servo_addr(curr_servo_addr) {}

// decode message from mission control 
float can_perseus::fixed_to_floating_point(hal::byte msb, hal::byte lsb, int exponent) {
  return static_cast<float>(((msb << 8) | lsb) >> exponent); 
}

// endcode message to mission control 
hal::u16 can_perseus::floating_to_fixed_point(float n, int exponent) {
  return (static_cast<hal::u16>(n) << exponent); 
}



void can_perseus::print_can_message(hal::serial& p_console,
                       hal::can_message const& p_message)
{
  hal::print<256>(p_console,
                  "Received Message from ID: 0x%lX, length: %u \n"
                  "payload = [ 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, "
                  "0x%02X, 0x%02X, 0x%02X ]\n",
                  p_message.id,
                  p_message.length,
                  p_message.payload[0],
                  p_message.payload[1],
                  p_message.payload[2],
                  p_message.payload[3],
                  p_message.payload[4],
                  p_message.payload[5],
                  p_message.payload[6],
                  p_message.payload[7]);
}

void can_perseus::process_can_message(hal::can_message const& p_message,
                        hal::v5::strong_ptr<bldc_perseus> bldc,
                        hal::v5::strong_ptr<hal::can_message> response)
{   
  auto console = resources::console();
  switch (static_cast<action>(p_message.payload[0])) {
    // case action::read_position: {
    //   auto current_position = bldc->get_current_position();
    //   response->length = 3;
    //   response->payload[0] =
    //     static_cast<hal::byte>(action::read_position) + 0x50;
    //   response->payload[1] = (current_position >> 8) & 0xff;
    //   response->payload[1] = current_position & 0xff;
    //   break;
    // }
    case action::freeze:{
      bldc_perseus::PID_settings pos_saved = {
        .kp = bldc->get_pid_settings().kp,
        .ki = bldc->get_pid_settings().ki,
        .kd = bldc->get_pid_settings().kd
      };
      bldc_perseus::PID_settings vel_saved = {
        .kp = bldc->get_pid_settings().kp,
        .ki = bldc->get_pid_settings().ki,
        .kd = bldc->get_pid_settings().kd
      };
      bldc_perseus::PID_settings hard_stop = {
        .kp = 0,
        .ki = 0,
        .kd = 0
      };
      bldc->update_pid_position(hard_stop);
      // SET FOR ELBOW RIGHT NOW
      // FIX FOR OTHERS
      for (int i = 0; i < 4; i++) bldc->update_position(); 
      bldc->update_pid_position(pos_saved); 
      bldc->update_pid_position(vel_saved); 
      break;
    }
    case action::heartbeat: {
      response->id = static_cast<hal::byte>(action::heartbeat) + 0x100;
      response->length = 1;
      response->payload[0] = m_curr_servo_addr + 0x50;
      break; 
    }
    case action::homing: {
      response->id = static_cast<hal::byte>(action::homing) + 0x100;
      response->length = 1;
      response->payload[0] = 0x01;
      response->payload[1] = 0x11;
      break; 
    }
    case action::set_position: {
      auto target_position = fixed_to_floating_point(p_message.payload[1], p_message.payload[2], 6);
      bldc->set_target_position(target_position);
      hal::print<64>(*console, "Target = %d\n", target_position);
      response->id = static_cast<hal::byte>(action::set_position) + 0x100;
      response->length = 1;
      response->payload[0] = static_cast<hal::byte>(action::set_position) + 0x50;;
      break;
    }
    case action::read_position: {
      auto reading_position = bldc->get_reading_position();
      hal::u16 t = can_perseus::floating_to_fixed_point(reading_position, 6); 
      response->id = static_cast<hal::byte>(action::read_position) + 0x100;
      response->length = 8;
      response->payload[0] = 0x20 + 0x50; 
      response->payload[1] = static_cast<hal::byte>(t >> 8) & 0xFF; // HIGH BYTE FIRST 
      response->payload[2] = static_cast<hal::byte>(t >> 0) & 0xFF;  // LOW BYTE SECOND
      break;
    }
    case action::read_velocity: {
      auto reading_velocity = bldc->get_reading_velocity();
      hal::u16 t = can_perseus::floating_to_fixed_point(reading_velocity, 6); 
      response->id = static_cast<hal::byte>(action::read_velocity) + 0x100;
      response->length = 8;
      response->payload[0] = 0x20 + 0x50; 
      response->payload[1] = static_cast<hal::byte>(t >> 8) & 0xFF; // HIGH BYTE FIRST 
      response->payload[2] = static_cast<hal::byte>(t >> 0) & 0xFF;  // LOW BYTE SECOND
      break;
    }
    case action::set_pid_position: {
      bldc_perseus::PID_settings settings = {
        .kp = fixed_to_floating_point(p_message.payload[1], p_message.payload[2], 14),
        .ki = fixed_to_floating_point(p_message.payload[3], p_message.payload[4], 14),
        .kd = fixed_to_floating_point(p_message.payload[5], p_message.payload[6], 14)
      };
      bldc->update_pid_position(settings);
      response->id = static_cast<hal::byte>(action::set_pid_position) + 0x100;
      response->length = 1;
      response->payload[0] = static_cast<hal::byte>(action::set_pid_position) + 0x50;
      break;
    }
    case action::set_pid_velocity: {
      bldc_perseus::PID_settings settings = {
        .kp = fixed_to_floating_point(p_message.payload[1], p_message.payload[2], 14),
        .ki = fixed_to_floating_point(p_message.payload[3], p_message.payload[4], 14),
        .kd = fixed_to_floating_point(p_message.payload[5], p_message.payload[6], 14)
      };
      bldc->update_pid_velocity(settings);
      response->id = static_cast<hal::byte>(action::set_pid_velocity) + 0x100;
      response->length = 1;
      response->payload[0] = static_cast<hal::byte>(action::set_pid_velocity) + 0x50;
      break;
    }
    default:
      hal::operation_not_supported(nullptr);
  }
}


} // namespace sjsu::perseus